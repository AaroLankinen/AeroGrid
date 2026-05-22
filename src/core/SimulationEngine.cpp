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
    const double gravity = 9.81;
    const double dt = 0.05;
    while (m_running) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            for (auto& drone : m_drones) {
                double groundHeight = m_terrain.getHeightAt(drone.x, drone.y);

                // Handle Landed State (Solar Charging)
                if (drone.status == DroneStatus::Landed) {
                    drone.batteryLevel = std::min(100.0, drone.batteryLevel + 0.05);
                    continue;
                }

                if (drone.status == DroneStatus::Crashed) continue;

                double altitudeAGL = std::max(0.0, drone.z - groundHeight);
                // Calculate battery needed for a safe descent.
                // Aggressive descent at 5m/s (2.0%/s cost) until 5m AGL, then 1.5m/s (1.3%/s cost).
                double t_fast = std::max(0.0, (altitudeAGL - 5.0) / 5.0);
                double t_slow = std::min(altitudeAGL, 5.0) / 1.5;
                double batteryRequiredToLand = (t_fast * 2.0) + (t_slow * 1.3) + 5.0; // 5% safety margin

                if (drone.status == DroneStatus::Flying && drone.batteryLevel <= batteryRequiredToLand) {
                    drone.status = DroneStatus::EmergencyLanding;
                    drone.navQueue.clear();
                }

                if (drone.batteryLevel > 0) {
                    // Landing logic (User-initiated or Emergency)
                    if (drone.status == DroneStatus::Landing || drone.status == DroneStatus::EmergencyLanding) {
                        drone.vx = drone.vy = 0;
                        if (altitudeAGL > 5.0) {
                            drone.vz = -5.0; // Aggressive altitude shed
                        } else {
                            drone.vz = -1.5; // Final safe approach
                        }
                    }
                    // Navigation Program
                    else if (!drone.navQueue.empty()) {
                        auto& target = drone.navQueue.front();
                        double tx = target.x;
                        double ty = target.y;
                        double tz = target.z;

                    // Adjust Z based on flight mode
                    if (drone.navMode == NavigationMode::MaxAltitude) {
                        tz = 80.0; // Standard cruise altitude
                    } else if (drone.navMode == NavigationMode::TerrainSkimming) {
                        tz = m_terrain.getHeightAt(drone.x, drone.y) + 5.0; // 5m AGL
                    }

                    double dx = tx - drone.x;
                    double dy = ty - drone.y;
                    double dz = tz - drone.z;
                    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

                    if (dist > 1.0) {
                        drone.vx = (dx / dist) * 5.0; // 5 m/s speed
                        drone.vy = (dy / dist) * 5.0;
                        drone.vz = (dz / dist) * 5.0;
                    } else {
                        // Reached point, pop from queue
                        drone.navQueue.erase(drone.navQueue.begin());
                        drone.vx = drone.vy = drone.vz = 0; // Hover
                    }
                    } else {
                        // No active target: Maintain minimum safe hover altitude (5m AGL)
                        double minSafeZ = groundHeight + 5.0;
                        if (drone.z < minSafeZ - 0.1) {
                            drone.vx = drone.vy = 0;
                            drone.vz = 1.5; // Controlled ascent to safe hover height
                        } else {
                            drone.vx = drone.vy = drone.vz = 0; // Hover at current height (if safe)
                            if (drone.z < minSafeZ) drone.z = minSafeZ; // Stabilize height
                        }
                    }

                    // Battery consumption: Hovering + extra for velocity
                    double speed = std::sqrt(drone.vx*drone.vx + drone.vy*drone.vy + drone.vz*drone.vz);
                    drone.batteryLevel -= (0.05 + speed * 0.01);
                } else {
                    // Power lost: Fall under gravity
                    drone.vz -= gravity * dt;
                    // Simple air resistance for horizontal momentum
                    drone.vx *= 0.99;
                    drone.vy *= 0.99;
                }

                // Apply velocity vectors
                drone.x += drone.vx * dt; 
                drone.y += drone.vy * dt;
                drone.z += drone.vz * dt;

                // Add noise only if flying (prevents jittering on ground)
                if (drone.status == DroneStatus::Flying) {
                    drone.x += (rand() % 100 - 50) / 1000.0;
                    drone.y += (rand() % 100 - 50) / 1000.0;
                    drone.z += (rand() % 100 - 50) / 1000.0;
                }

                // Check for ground contact
                if (drone.z <= groundHeight + 0.05) {
                    // Drones crash if the vertical impact velocity is too high (e.g., free fall)
                    // A controlled landing at -1.5 m/s is considered safe.
                    if (std::abs(drone.vz) < 2.0) {
                        drone.status = DroneStatus::Landed;
                    } else {
                        drone.status = DroneStatus::Crashed;
                    }
                    drone.z = groundHeight;
                    drone.vx = drone.vy = drone.vz = 0;
                }

                // Signal strength simulation (linear decay from origin)
                double distToOrigin = std::sqrt(drone.x * drone.x + drone.y * drone.y + drone.z * drone.z);
                const double maxRange = 500.0;
                drone.signalStrength = std::max(0.0, 1.0 - (distToOrigin / maxRange));

                if (drone.batteryLevel < 0) drone.batteryLevel = 0;
            }

            // Drone-to-drone collision detection: check all pairs
            for (size_t i = 0; i < m_drones.size(); ++i) {
                for (size_t j = i + 1; j < m_drones.size(); ++j) {
                    auto& d1 = m_drones[i];
                    auto& d2 = m_drones[j];
                    
                    if (d1.status == DroneStatus::Crashed || d2.status == DroneStatus::Crashed) 
                        continue;

                    const double safetyMargin = 2.0; // Account for drift/buffeting
                    double dx = d1.x - d2.x;
                    double dy = d1.y - d2.y;
                    double dz = d1.z - d2.z;
                    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                    double minDist = d1.radius + d2.radius;
                    double safeZone = minDist + safetyMargin;

                    // Avoidance (Steering) - Push drones apart if in safety margin
                    if (dist < safeZone && dist > 0.001) {
                        double push = (safeZone - dist) * 0.5;
                        d1.vx += (dx / dist) * push;
                        d1.vy += (dy / dist) * push;
                        d1.vz += (dz / dist) * push;
                    }

                    // Hard Collision
                    double distSq = dist * dist;

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

void SimulationEngine::assignTarget(int id, double x, double y, NavigationMode mode) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto& drone : m_drones) {
        if (drone.id == id) {
            drone.navQueue.push_back({x, y, 0.0}); // Z is calculated by mode in loop
            drone.navMode = mode;
        }
    }
}

void SimulationEngine::takeOff(int id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto& drone : m_drones) {
        if (drone.id == id && drone.status == DroneStatus::Landed) {
            double groundHeight = m_terrain.getHeightAt(drone.x, drone.y);
            // Safety check: Ensure battery is sufficient to reach and land from 5m safe hover altitude
            // Requirement for 5m AGL: (5.0 / 1.5) * 1.3 + 5.0 = ~9.33%
            double batteryRequiredForSafeHover = (5.0 / 1.5) * 1.3 + 5.0;

            if (drone.batteryLevel > batteryRequiredForSafeHover) {
                drone.status = DroneStatus::Flying;
                drone.vz = 2.0; // Initial ascent thrust
                break;
            }
        }
    }
}

void SimulationEngine::landDrone(int id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto& drone : m_drones) {
        if (drone.id == id && drone.status == DroneStatus::Flying) {
            drone.status = DroneStatus::Landing;
            drone.navQueue.clear();
            break;
        }
    }
}

void SimulationEngine::clearNavQueue(int id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto& drone : m_drones) {
        if (drone.id == id) {
            drone.navQueue.clear();
            drone.vx = drone.vy = drone.vz = 0; // Stop moving
            break;
        }
    }
}

void SimulationEngine::removeNavPoint(int id, int index) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto& drone : m_drones) {
        if (drone.id == id) {
            if (index >= 0 && index < (int)drone.navQueue.size()) {
                drone.navQueue.erase(drone.navQueue.begin() + index);
            }
            break;
        }
    }
}