#include "SimulationEngine.h"
#include <chrono>
#include <thread>
#include <cstdlib>
#include <ctime>
#include <mutex>

SimulationEngine::SimulationEngine(QObject* parent) 
    : QObject(parent), m_running(false), m_workerThread(nullptr) {
    // Initialize with 5 dummy drones
    for(int i = 0; i < 5; ++i) m_drones.emplace_back(i);
    std::srand(std::time(nullptr)); // Seed random for GPS drift
}

void SimulationEngine::startSimulation() {
    if (m_running) return;
    m_running = true;
    // Initialize drones at a safe altitude above the terrain
    for(auto& drone : m_drones) {
        drone.z = 50.0; // Set initial altitude to 50 units above ground
    }
    m_workerThread = QThread::create([this] { run(); });
    m_workerThread->start();
}

void SimulationEngine::stopSimulation() {
    m_running = false;
    if (m_workerThread) {
        m_workerThread->quit();
        m_workerThread->wait();
        m_workerThread->deleteLater();
        m_workerThread = nullptr;
    }
}

void SimulationEngine::run() {
    while (m_running) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            for (auto& drone : m_drones) {
                if (drone.status == DroneStatus::Crashed) continue;

                // Simple physics: update position based on velocity
                drone.x += drone.vx * 0.05; 
                drone.y += drone.vy * 0.05;
                drone.z += drone.vz * 0.05;

                // Add minor random noise to simulate GPS drift
                drone.x += (rand() % 100 - 50) / 1000.0;
                drone.y += (rand() % 100 - 50) / 1000.0;
                drone.z += (rand() % 100 - 50) / 1000.0; // Simulate altitude changes

                // Check for terrain collision
                double groundHeight = m_terrain.getHeightAt(drone.x, drone.y);
                if (drone.z <= groundHeight) {
                    drone.z = groundHeight;
                    drone.status = DroneStatus::Crashed;
                    drone.vx = drone.vy = drone.vz = 0;
                }

                // Signal strength simulation (linear decay from origin)
                double dist = std::sqrt(drone.x * drone.x + drone.y * drone.y + drone.z * drone.z);
                const double maxRange = 500.0;
                drone.signalStrength = std::max(0.0, 1.0 - (dist / maxRange));

                // Simulate battery drain
                drone.batteryLevel -= 0.1;
                if (drone.batteryLevel < 0) drone.batteryLevel = 0;
            }
        }
        emit simulationUpdated(); // Notify UI that data is ready
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

std::vector<Drone> SimulationEngine::getDroneData() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_drones;
}