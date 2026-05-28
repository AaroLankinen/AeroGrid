#include "SimulationEngine.h"
#include <chrono>
#include <thread>
#include <cstdlib>
#include <ctime>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <QtMath>
#include "Constants.h"

SimulationEngine::SimulationEngine(QObject* parent) 
    : QObject(parent), m_running(false), m_workerThread(nullptr) {
    std::srand(std::time(nullptr)); // Seed random for GPS drift
}

SimulationEngine::~SimulationEngine() {
    stopSimulation();
}

void SimulationEngine::startSimulation(unsigned int seed, double landProp, int width, int height) {
    if (m_running) stopSimulation();
    
    // Re-initialize terrain with the provided seed and preset dimensions
    m_terrain = TerrainMap(seed, landProp, width, height);
    std::srand(seed); 

    m_drones.clear();
    m_baseInventory.clear();

    const double halfWorldWidth = m_terrain.getWorldWidth() * 0.5;
    const double halfWorldHeight = m_terrain.getWorldHeight() * 0.5;
    const double basePlacementRangeX = std::max(10.0, halfWorldWidth * 0.25);
    const double basePlacementRangeY = std::max(10.0, halfWorldHeight * 0.25);

    // Ensure base location is on land (Height >= 1.0)
    int safetyCounter = 0;
    do {
        m_baseX = (std::rand() / static_cast<double>(RAND_MAX)) * (2.0 * basePlacementRangeX) - basePlacementRangeX;
        m_baseY = (std::rand() / static_cast<double>(RAND_MAX)) * (2.0 * basePlacementRangeY) - basePlacementRangeY;
        // If the user specified 0% land, we must break to avoid infinite loop
        if (++safetyCounter > 2000) break; 
    } while (m_terrain.getHeightAt(m_baseX, m_baseY) < AeroGrid::World::LAND_THRESHOLD);

    // 2. Randomize Dynamic Obstacles (Birds/Unauthorized Drones)
    m_dynamicObstacles.clear();
    int obstacleCount = 3 + (std::rand() % 4);
    for (int i = 0; i < obstacleCount; ++i) {
        double obsX = (std::rand() / static_cast<double>(RAND_MAX)) * (2.0 * halfWorldWidth) - halfWorldWidth;
        double obsY = (std::rand() / static_cast<double>(RAND_MAX)) * (2.0 * halfWorldHeight) - halfWorldHeight;
        double obsVX = (std::rand() / static_cast<double>(RAND_MAX)) * 6.0 - 3.0;
        double obsVY = (std::rand() / static_cast<double>(RAND_MAX)) * 6.0 - 3.0;
        double obsVZ = ((std::rand() % 4) - 2) * 0.1;
        double obsRadius = 1.0 + (std::rand() % 200) / 100.0;

        m_dynamicObstacles.push_back({
            100 + i,                            // ID
            obsX,
            obsY,
            (double)(std::rand() % 40 + 20),     // Z (Initial Altitude)
            obsVX,
            obsVY,
            obsVZ,
            obsRadius
        });
    }

    // 3. Initialize Hangar with 12 drones
    for(int i = 0; i < AeroGrid::World::DEFAULT_INVENTORY_COUNT; ++i) {
        m_baseInventory.emplace_back(i);
    }

    m_running = true;
    m_workerThread = QThread::create([this] { run(); });
    m_workerThread->start();
}

void SimulationEngine::stopSimulation() {
    m_running = false;
    if (m_workerThread) {
        m_workerThread->quit();
        m_workerThread->wait();
        delete m_workerThread;
        m_workerThread = nullptr;
    }
}

void SimulationEngine::processHangarLogic() {
    auto it = m_drones.begin();
    while (it != m_drones.end()) {
        // If a drone has returned to base and landed, move it to inventory
        if (it->status == DroneStatus::Landed && it->returningToBase) {
            double dx = it->x - m_baseX;
            double dy = it->y - m_baseY;
            double dist = qSqrt(dx*dx + dy*dy);
            
            if (dist < 10.0) { // Within 10m of helipad center (accommodates grid landing)
                it->returningToBase = false;
                it->navQueue.clear();
                
                // Maintain sorted order by ID when returning to inventory
                auto insertPos = std::lower_bound(m_baseInventory.begin(), m_baseInventory.end(), *it,
                    [](const Drone& a, const Drone& b) { return a.id < b.id; });
                m_baseInventory.insert(insertPos, *it);
                it = m_drones.erase(it);
                continue;
            }
        }
        ++it;
    }
}

void SimulationEngine::run() {
    const double dt = AeroGrid::Physics::DT;
    while (m_running) {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            updateDynamicObstacles(dt);
            processHangarLogic();
            updateHangarDrones();
            for (auto& drone : m_drones) {
                updateDroneState(drone, dt);
            }
            handleDroneToDroneCollisions();
        }
        emit simulationUpdated();
        std::this_thread::sleep_for(std::chrono::milliseconds(AeroGrid::Physics::TICK_MS));
    }
}

void SimulationEngine::updateDynamicObstacles(double dt) {
    for (auto& obs : m_dynamicObstacles) {
        obs.x += obs.vx * dt;
        obs.y += obs.vy * dt;
        obs.z += obs.vz * dt;

        const double halfWorldWidth = m_terrain.getWorldWidth() * 0.5;
        const double halfWorldHeight = m_terrain.getWorldHeight() * 0.5;
        if (obs.x < -halfWorldWidth || obs.x > halfWorldWidth) obs.vx *= -1;
        if (obs.y < -halfWorldHeight || obs.y > halfWorldHeight) obs.vy *= -1;
        if (obs.z < 10.0 || obs.z > 60.0) obs.vz *= -1;
    }
}

void SimulationEngine::updateHangarDrones() {
    for (auto& drone : m_baseInventory) {
        drone.batteryLevel = qMin(100.0, drone.batteryLevel + AeroGrid::Physics::HANGAR_CHARGE_RATE); 
    }
}

void SimulationEngine::updateDroneState(Drone& drone, double dt) {
    double groundHeight = m_terrain.getHeightAt(drone.x, drone.y);
    drone.proximityAlert = false; 

    if (drone.status == DroneStatus::Landed) {
        drone.batteryLevel = qMin(100.0, drone.batteryLevel + AeroGrid::Physics::LANDED_CHARGE_RATE);
        return;
    }

    if (drone.status == DroneStatus::Crashed) return;

    calculateDroneMovement(drone, groundHeight, dt);
    applyObstacleAvoidance(drone, groundHeight);

    drone.x += drone.vx * dt; 
    drone.y += drone.vy * dt;
    drone.z += drone.vz * dt;

    // Update Orientation based on movement (Yaw)
    // Drones "look" in the direction of their horizontal velocity
    if (qSqrt(drone.vx * drone.vx + drone.vy * drone.vy) > 0.1) {
        double targetYaw = qRadiansToDegrees(qAtan2(drone.vy, drone.vx));
        // Smoothly interpolate yaw for 3D camera stability
        double diff = targetYaw - drone.yaw;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        drone.yaw += diff * 0.1; 
    }

    if (drone.status == DroneStatus::Flying) {
        drone.x += (rand() % 100 - 50) / 1000.0;
        drone.y += (rand() % 100 - 50) / 1000.0;
        drone.z += (rand() % 100 - 50) / 1000.0;
    }

    checkGroundContact(drone, groundHeight);
    updateDroneSignalStrength(drone);
    
    if (drone.batteryLevel < 0) drone.batteryLevel = 0;
}

void SimulationEngine::calculateDroneMovement(Drone& drone, double groundHeight, double dt) {
    if (drone.batteryLevel <= 0) {
        drone.vz -= AeroGrid::Physics::GRAVITY * dt; 
        drone.vx *= 0.99;
        drone.vy *= 0.99;
        return;
    }

    double altitudeAGL = qMax(0.0, drone.z - groundHeight);
    double batteryRequiredToLand = drone.calculateBatteryRequiredToLand(groundHeight);

    if (drone.status == DroneStatus::Flying && drone.batteryLevel <= batteryRequiredToLand) {
        drone.status = DroneStatus::EmergencyLanding;
        drone.navQueue.clear();
    }

    if (drone.status == DroneStatus::Landing || drone.status == DroneStatus::EmergencyLanding) {
        drone.vx = drone.vy = 0;
        drone.vz = (altitudeAGL > AeroGrid::Physics::MIN_SAFE_ALTITUDE_AGL) ? AeroGrid::Physics::LANDING_FAST_SPEED : AeroGrid::Physics::LANDING_SLOW_SPEED;
    } else if (!drone.navQueue.empty()) {
        const auto& target = drone.navQueue.front();
        double offsetX = (drone.id % 3 - 1) * 4.0;
        double offsetY = (drone.id / 3 - 1) * 4.0;
        double tx = target.x + offsetX;
        double ty = target.y + offsetY;
        double tz = target.z;

        if (drone.navMode == NavigationMode::MaxAltitude) tz = AeroGrid::Physics::STANDARD_CRUISE_ALTITUDE;
        else if (drone.navMode == NavigationMode::TerrainSkimming) tz = groundHeight + AeroGrid::Physics::MIN_SAFE_ALTITUDE_AGL;

        double dx = tx - drone.x;
        double dy = ty - drone.y;
        double dz = tz - drone.z;
        double dist = qSqrt(dx*dx + dy*dy + dz*dz);

        if (dist > AeroGrid::Physics::MIN_DRONE_DIST) {
            drone.vx = (dx / dist) * 5.0;
            drone.vy = (dy / dist) * 5.0;
            drone.vz = (dz / dist) * 5.0;
        } else {
            drone.navQueue.erase(drone.navQueue.begin());
            if (drone.returningToBase && drone.navQueue.empty()) drone.status = DroneStatus::Landing;
            drone.vx = drone.vy = drone.vz = 0;
        }
    } else {
        double minSafeZ = groundHeight + AeroGrid::Physics::MIN_SAFE_ALTITUDE_AGL;
        if (drone.z < minSafeZ - 0.1) {
            drone.vx = drone.vy = 0;
            drone.vz = qAbs(AeroGrid::Physics::LANDING_SLOW_SPEED);
        } else {
            drone.vx = drone.vy = drone.vz = 0;
            if (drone.z < minSafeZ) drone.z = minSafeZ;
        }
    }

    double speed = qSqrt(drone.vx*drone.vx + drone.vy*drone.vy + drone.vz*drone.vz);
    drone.batteryLevel -= (AeroGrid::Physics::HOVER_CONSUMPTION + speed * AeroGrid::Physics::VELOCITY_CONSUMPTION_FACTOR);
}

void SimulationEngine::applyObstacleAvoidance(Drone& drone, double groundHeight) {
    const auto& obstacles = m_terrain.getObstacles();
    for (const auto& obs : obstacles) {
        double dx = drone.x - obs.x;
        double dy = drone.y - obs.y;
        double dist2D = qSqrt(dx*dx + dy*dy);
        double minDist = drone.radius + obs.radius;
        double safeZone = minDist + AeroGrid::Physics::STATIC_OBS_SAFETY_MARGIN;

        double obsGroundHeight = m_terrain.getHeightAt(obs.x, obs.y);
        double absoluteObsHeight = obsGroundHeight + obs.height;
        if (drone.z < absoluteObsHeight + 2.0) {
            if (dist2D < safeZone && dist2D > 0.001) {
                drone.proximityAlert = true;
                double push = (safeZone - dist2D) * AeroGrid::Physics::REPULSION_FORCE_STATIC;
                if (drone.status == DroneStatus::Flying) {
                    drone.vx += (dx / dist2D) * push;
                    drone.vy += (dy / dist2D) * push;
                }
            }

            if (dist2D < minDist && drone.z < absoluteObsHeight) {
                drone.status = DroneStatus::Crashed;
                drone.vx = drone.vy = drone.vz = 0;
                drone.x = obs.x + (dx / dist2D) * minDist;
                drone.y = obs.y + (dy / dist2D) * minDist;
            }
        }
    }

    for (const auto& obs : m_dynamicObstacles) {
        double dx = drone.x - obs.x;
        double dy = drone.y - obs.y;
        double dz = drone.z - obs.z;
        double dist = qSqrt(dx*dx + dy*dy + dz*dz);
        double minDist = drone.radius + obs.radius;
        double safeZone = minDist + AeroGrid::Physics::DYNAMIC_OBS_SAFETY_MARGIN;

        if (dist < safeZone && dist > 0.001) {
            drone.proximityAlert = true;
            double push = (safeZone - dist) * AeroGrid::Physics::REPULSION_FORCE_DYNAMIC;
            if (drone.status == DroneStatus::Flying) {
                drone.vx += (dx / dist) * push;
                drone.vy += (dy / dist) * push;
                drone.vz += (dz / dist) * push;
            }
        }

        if (dist < minDist) {
            drone.status = DroneStatus::Crashed;
            drone.vx = drone.vy = drone.vz = 0;
        }
    }
}

void SimulationEngine::checkGroundContact(Drone& drone, double groundHeight) {
    if (drone.z <= groundHeight + 0.05 && drone.vz < 0) {
        if (qAbs(drone.vz) < AeroGrid::Physics::MAX_SAFE_LANDING_SPEED) {
            drone.status = DroneStatus::Landed;
        } else {
            drone.status = DroneStatus::Crashed;
        }
        drone.z = groundHeight;
        drone.vx = drone.vy = drone.vz = 0;
    }
}

void SimulationEngine::updateDroneSignalStrength(Drone& drone) {
    double sDx = drone.x - m_baseX;
    double sDy = drone.y - m_baseY;
    double sDz = drone.z;
    double distToBase = qSqrt(sDx*sDx + sDy*sDy + sDz*sDz);
    drone.signalStrength = qMax(0.0, 1.0 - (distToBase / AeroGrid::World::SIGNAL_MAX_RANGE));
}

void SimulationEngine::handleDroneToDroneCollisions() {
    for (size_t i = 0; i < m_drones.size(); ++i) {
        for (size_t j = i + 1; j < m_drones.size(); ++j) {
            auto& d1 = m_drones[i];
            auto& d2 = m_drones[j];
            
            if (d1.status == DroneStatus::Crashed || d2.status == DroneStatus::Crashed) continue;

            double dx = d1.x - d2.x;
            double dy = d1.y - d2.y;
            double dz = d1.z - d2.z;
            double dist = qSqrt(dx*dx + dy*dy + dz*dz);
            if (dist < 0.001) continue; 

            double minDist = d1.radius + d2.radius;
            double safeZone = minDist + AeroGrid::Physics::DRONE_TO_DRONE_SAFETY_BUFFER;

            if (dist < safeZone) {
                d1.proximityAlert = true;
                d2.proximityAlert = true;
                double push = (safeZone - dist) * AeroGrid::Physics::REPULSION_FORCE_DRONE;
                double nx = dx / dist; double ny = dy / dist; double nz = dz / dist;

                if (d1.status != DroneStatus::Landed) {
                    d1.vx += nx * push; d1.vy += ny * push; d1.vz += nz * push;
                }
                if (d2.status != DroneStatus::Landed) {
                    d2.vx -= nx * push; d2.vy -= ny * push; d2.vz -= nz * push;
                }
            }

            if (dist < minDist) {
                d1.status = d2.status = DroneStatus::Crashed;
                d1.vx = d1.vy = d1.vz = 0;
                d2.vx = d2.vy = d2.vz = 0;
            }
        }
    }
}

std::vector<Drone> SimulationEngine::getDroneData() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_drones;
}

const Drone* SimulationEngine::getDroneById(int id) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = std::find_if(m_drones.begin(), m_drones.end(), [id](const Drone& d) {
        return d.id == id;
    });
    return (it != m_drones.end()) ? &(*it) : nullptr;
}

std::vector<DynamicObstacle> SimulationEngine::getDynamicObstacleData() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_dynamicObstacles;
}

std::vector<Drone> SimulationEngine::getInventoryData() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_baseInventory;
}

int SimulationEngine::getInventoryCount() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return (int)m_baseInventory.size();
}

void SimulationEngine::launchDrone() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_baseInventory.empty()) return;

    Drone d = m_baseInventory.back();
    m_baseInventory.pop_back();

    // Place drone on a specific launch pad to avoid collisions during simultaneous launch.
    // Matches the formation logic used in flight (3-column grid).
    double offsetX = (d.id % 3 - 1) * 4.0;
    double offsetY = (d.id / 3 - 1) * 4.0;

    d.x = m_baseX + offsetX;
    d.y = m_baseY + offsetY;
    d.z = m_terrain.getHeightAt(d.x, d.y);
    d.status = DroneStatus::Landed;
    d.vx = d.vy = d.vz = 0;
    
    m_drones.push_back(d);
}

void SimulationEngine::launchDrones(const std::vector<int>& ids) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (int id : ids) {
        auto it = std::find_if(m_baseInventory.begin(), m_baseInventory.end(), [id](const Drone& d) {
            return d.id == id;
        });

        if (it != m_baseInventory.end()) {
            Drone d = *it;
            m_baseInventory.erase(it);

            // Place drone on its specific launch pad
            double offsetX = (d.id % 3 - 1) * 4.0;
            double offsetY = (d.id / 3 - 1) * 4.0;

            d.x = m_baseX + offsetX;
            d.y = m_baseY + offsetY;
            d.z = m_terrain.getHeightAt(d.x, d.y);
            d.status = DroneStatus::Landed;
            d.vx = d.vy = d.vz = 0;
            
            m_drones.push_back(d);
        }
    }
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
            double batteryRequiredToTakeoff = drone.calculateBatteryRequiredToLand(groundHeight);

            if (drone.batteryLevel > batteryRequiredToTakeoff) {
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