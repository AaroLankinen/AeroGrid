#include "SimulationEngine.h"
#include <chrono>
#include <thread>
#include <cstdlib>
#include <ctime>
#include <mutex>
#include <cmath>

SimulationEngine::SimulationEngine(QObject* parent) 
    : QObject(parent), m_running(false), m_workerThread(nullptr) {
    // Initialize with 5 dummy drones
    for(int i = 0; i < 5; ++i) m_drones.emplace_back(i);
    std::srand(std::time(nullptr)); // Seed random for GPS drift

    // 1. Initialize semi-random base location near the map center
    // This must happen before drones or signal logic are initialized
    m_baseX = (std::rand() % 40) - 20.0;
    m_baseY = (std::rand() % 40) - 20.0;

    // 2. Randomize Dynamic Obstacles (Birds/Unauthorized Drones)
    m_dynamicObstacles.clear();
    int obstacleCount = 3 + (std::rand() % 4);
    for (int i = 0; i < obstacleCount; ++i) {
        m_dynamicObstacles.push_back({
            100 + i,                            // ID
            (double)(std::rand() % 160 - 80),    // X
            (double)(std::rand() % 160 - 80),    // Y
            (double)(std::rand() % 40 + 20),     // Z (Initial Altitude)
            (double)(std::rand() % 6 - 3),       // VX
            (double)(std::rand() % 6 - 3),       // VY
            (double)(std::rand() % 4 - 2) * 0.1, // VZ
            1.0 + (std::rand() % 200) / 100.0    // Radius
        });
    }
}

void SimulationEngine::startSimulation(unsigned int seed) {
    if (m_running) return;
    
    // Re-initialize terrain with the provided seed
    m_terrain = TerrainMap(seed);
    std::srand(seed); 

    // Ensure base location is on land (Height >= 1.0)
    do {
        m_baseX = (std::rand() % 40) - 20.0;
        m_baseY = (std::rand() % 40) - 20.0;
    } while (m_terrain.getHeightAt(m_baseX, m_baseY) < 1.0);

    m_running = true;

    // 3. Randomize Drone Spawns close to base with a safety buffer
    for(size_t i = 0; i < m_drones.size(); ++i) {
        // Spawn in a ring around the base to ensure a clear buffer zone
        double angle = (2.0 * 3.14159 * i) / m_drones.size();
        double r = 10.0 + (std::rand() % 5); 
        
        m_drones[i].x = m_baseX + r * std::cos(angle);
        m_drones[i].y = m_baseY + r * std::sin(angle);
        
        // Spawn drones above the local ground height
        double groundH = m_terrain.getHeightAt(m_drones[i].x, m_drones[i].y);
        m_drones[i].z = groundH + 15.0; 
        
        m_drones[i].status = DroneStatus::Flying;
        m_drones[i].vx = m_drones[i].vy = m_drones[i].vz = 0;
        m_drones[i].batteryLevel = 100.0;
        m_drones[i].navQueue.clear();
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

            // 0. Update Dynamic Obstacles
            for (auto& obs : m_dynamicObstacles) {
                obs.x += obs.vx * dt;
                obs.y += obs.vy * dt;
                obs.z += obs.vz * dt;

                // Simple boundary logic: bounce back if they hit map edges (approx 100m)
                if (std::abs(obs.x) > 90.0) obs.vx *= -1;
                if (std::abs(obs.y) > 90.0) obs.vy *= -1;
                if (obs.z < 10.0 || obs.z > 60.0) obs.vz *= -1;
            }

            for (auto& drone : m_drones) {
                double groundHeight = m_terrain.getHeightAt(drone.x, drone.y);
                drone.proximityAlert = false; // Reset alert state at start of frame

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
                        const auto& target = drone.navQueue.front();

                        // Formation Flying: Apply unique offsets based on ID to avoid mid-air convergence
                        // This creates a loose grid formation around the shared nav point
                        double offsetX = (drone.id % 3 - 1) * 4.0; 
                        double offsetY = (drone.id / 3 - 1) * 4.0;

                        double tx = target.x + offsetX;
                        double ty = target.y + offsetY;
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
                        if (drone.returningToBase && drone.navQueue.empty()) {
                            drone.status = DroneStatus::Landing;
                            drone.returningToBase = false;
                        }
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

                // Passive Obstacle Avoidance & Collision
                const auto& obstacles = m_terrain.getObstacles();
                for (const auto& obs : obstacles) {
                    double dx = drone.x - obs.x;
                    double dy = drone.y - obs.y;
                    double dist2D = std::sqrt(dx*dx + dy*dy);
                    double minDist = drone.radius + obs.radius;
                    const double obsSafetyMargin = 5.0;
                    double safeZone = minDist + obsSafetyMargin;

                    // Only worry about obstacles if we are below their top (plus a margin)
                    // 4. Buildings rest upon the ground: calculate absolute height
                    double obsGroundHeight = m_terrain.getHeightAt(obs.x, obs.y);
                    double absoluteObsHeight = obsGroundHeight + obs.height;
                    if (drone.z < absoluteObsHeight + 2.0) {
                        if (dist2D < safeZone && dist2D > 0.001) {
                            drone.proximityAlert = true;

                            // Repulsion: Steer horizontally away from the obstacle
                            double push = (safeZone - dist2D) * 0.2;
                            double nx = dx / dist2D;
                            double ny = dy / dist2D;
                            
                            if (drone.status == DroneStatus::Flying) {
                                drone.vx += nx * push;
                                drone.vy += ny * push;
                            }
                        }

                        // Hard Collision with static obstacle
                        if (dist2D < minDist && drone.z < absoluteObsHeight) {
                            drone.status = DroneStatus::Crashed;
                            drone.vx = drone.vy = drone.vz = 0;
                            // Nudge slightly outside to prevent continuous collision
                            drone.x = obs.x + (dx / dist2D) * minDist;
                            drone.y = obs.y + (dy / dist2D) * minDist;
                        }
                    }
                }

                // Passive Dynamic Obstacle Avoidance
                for (const auto& obs : m_dynamicObstacles) {
                    double dx = drone.x - obs.x;
                    double dy = drone.y - obs.y;
                    double dz = drone.z - obs.z;
                    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                    double minDist = drone.radius + obs.radius;
                    const double dynamicSafetyMargin = 5.0;
                    double safeZone = minDist + dynamicSafetyMargin;

                    if (dist < safeZone && dist > 0.001) {
                        drone.proximityAlert = true;

                        // Repulsion Force
                        double push = (safeZone - dist) * 0.3;
                        double nx = dx / dist;
                        double ny = dy / dist;
                        double nz = dz / dist;

                        if (drone.status == DroneStatus::Flying) {
                            drone.vx += nx * push;
                            drone.vy += ny * push;
                            drone.vz += nz * push;
                        }
                    }

                    // Hard Collision with dynamic obstacle
                    if (dist < minDist) {
                        drone.status = DroneStatus::Crashed;
                        drone.vx = drone.vy = drone.vz = 0;
                    }
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

                // Signal strength simulation (linear decay from the randomized base)
                double sDx = drone.x - m_baseX;
                double sDy = drone.y - m_baseY;
                double sDz = drone.z; // Height also affects signal
                double distToBase = std::sqrt(sDx*sDx + sDy*sDy + sDz*sDz);
                
                const double maxRange = 300.0; // Reduced range for more realism
                drone.signalStrength = std::max(0.0, 1.0 - (distToBase / maxRange));

                if (drone.batteryLevel < 0) drone.batteryLevel = 0;
            }

            // Drone-to-drone collision detection: check all pairs
            for (size_t i = 0; i < m_drones.size(); ++i) {
                for (size_t j = i + 1; j < m_drones.size(); ++j) {
                    auto& d1 = m_drones[i];
                    auto& d2 = m_drones[j];
                    
                    if (d1.status == DroneStatus::Crashed || d2.status == DroneStatus::Crashed) 
                        continue;

                    const double safetyMargin = 4.0; // Safety buffer to maintain between drones
                    double dx = d1.x - d2.x;
                    double dy = d1.y - d2.y;
                    double dz = d1.z - d2.z;
                    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                    if (dist < 0.001) continue; 

                    double minDist = d1.radius + d2.radius;
                    double safeZone = minDist + safetyMargin;

                    // Passive Collision Avoidance (Repulsion Field)
                    if (dist < safeZone) {
                        d1.proximityAlert = true;
                        d2.proximityAlert = true;

                        double push = (safeZone - dist) * 0.2;
                        double nx = dx / dist;
                        double ny = dy / dist;
                        double nz = dz / dist;

                        if (d1.status != DroneStatus::Landed) {
                            d1.vx += nx * push;
                            d1.vy += ny * push;
                            d1.vz += nz * push;
                        }
                        if (d2.status != DroneStatus::Landed) {
                            d2.vx -= nx * push;
                            d2.vy -= ny * push;
                            d2.vz -= nz * push;
                        }
                    }

                    // Hard Collision
                    if (dist < minDist) {
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

std::vector<DynamicObstacle> SimulationEngine::getDynamicObstacleData() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_dynamicObstacles;
}

void SimulationEngine::assignTarget(int id, double x, double y, NavigationMode mode) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto& drone : m_drones) {
        if (drone.id == id) {
            drone.navQueue.push_back({x, y, 0.0}); // Z is calculated by mode in loop
            drone.navMode = mode;
            drone.returningToBase = false; // Manual target overrides RTB
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

void SimulationEngine::returnToBase(int id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto& drone : m_drones) {
        if (drone.id == id && drone.status == DroneStatus::Flying) {
            drone.navQueue.clear();
            drone.navQueue.push_back({m_baseX, m_baseY, 0.0});
            drone.returningToBase = true;
            break;
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