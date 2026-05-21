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
    // Create a thread and start the run loop
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
                // Simple physics: update position based on velocity
                drone.x += drone.vx * 0.05; 
                drone.y += drone.vy * 0.05;
                // Add minor random noise to simulate GPS drift
                drone.x += (rand() % 100 - 50) / 1000.0;
                drone.y += (rand() % 100 - 50) / 1000.0;
                drone.z += (rand() % 100 - 50) / 1000.0; // Simulate altitude changes
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