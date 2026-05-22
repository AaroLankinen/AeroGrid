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
    // Initialize drones at a safe altitude and spacing to avoid collision
    for(size_t i = 0; i < m_drones.size(); ++i) {
        m_drones[i].x = i * 10.0; // Provide horizontal spacing buffer
        m_drones[i].y = 0.0;
        m_drones[i].z = 50.0; // Set initial altitude to 50 units
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

            // Drone-to-drone collision detection: check all pairs
            for (size_t i = 0; i < m_drones.size(); ++i) {
                for (size_t j = i + 1; j < m_drones.size(); ++j) {
                    auto& d1 = m_drones[i];
                    auto& d2 = m_drones[j];
                    
                    if (d1.status == DroneStatus::Crashed || d2.status == DroneStatus::Crashed) 
                        continue;

                    double dx = d1.x - d2.x;
                    double dy = d1.y - d2.y;
                    double dz = d1.z - d2.z;
                    double distSq = dx*dx + dy*dy + dz*dz;
                    double minDist = d1.radius + d2.radius;

                    if (distSq < (minDist * minDist)) {
                        d1.status = d2.status = DroneStatus::Crashed;
                        d1.vx = d1.vy = d1.vz = 0;
                        d2.vx = d2.vy = d2.vz = 0;
                    }
                }
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