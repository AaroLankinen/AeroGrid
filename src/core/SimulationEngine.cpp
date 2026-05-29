#include "SimulationEngine.h"
#include <chrono>
#include <QPointF>
#include <thread>
#include <cstdlib>
#include <ctime>
#include <mutex>
#include <shared_mutex>
#include <cmath>
#include <algorithm>
#include <QtMath>
#include "Constants.h"

namespace {
    /**
     * @brief Computes the 2D signed distance (SDF) and outward normal vector from a point to a static obstacle.
     * 
     * Handles Cylinders, oriented Rectangles, and Triangles.
     *
     * @param obs The static obstacle being queried.
     * @param px Query point X coordinate (meters).
     * @param py Query point Y coordinate (meters).
     * @param outNormalX [out] Outward-facing normal vector X component at the closest boundary point.
     * @param outNormalY [out] Outward-facing normal vector Y component at the closest boundary point.
     * @return Signed distance to the obstacle boundary (negative inside, positive outside).
     */
    double getDistanceToObstacle(const StaticObstacle& obs, double px, double py, double& outNormalX, double& outNormalY) {
        if (obs.shape == ObstacleShape::Cylinder) {
            double dx = px - obs.x;
            double dy = py - obs.y;
            double dist2D = qSqrt(dx*dx + dy*dy);
            if (dist2D > 0.001) {
                outNormalX = dx / dist2D;
                outNormalY = dy / dist2D;
                return dist2D - obs.radius;
            } else {
                outNormalX = 1.0;
                outNormalY = 0.0;
                return -obs.radius;
            }
        } else if (obs.shape == ObstacleShape::Rectangle) {
            double dx = px - obs.x;
            double dy = py - obs.y;
            double cosR = qCos(obs.rotation);
            double sinR = qSin(obs.rotation);
            double lx = dx * cosR + dy * sinR;
            double ly = -dx * sinR + dy * cosR;

            double hx = obs.width * 0.5;
            double hy = obs.depth * 0.5;

            double ax = qAbs(lx) - hx;
            double ay = qAbs(ly) - hy;

            double ex = qMax(0.0, ax);
            double ey = qMax(0.0, ay);

            double inDist = qMin(0.0, qMax(ax, ay));
            double dist = qSqrt(ex*ex + ey*ey) + inDist;

            double lnx = 0.0, lny = 0.0;
            if (qMax(ax, ay) > 0.0) {
                if (ax > 0.0 && ay > 0.0) {
                    lnx = lx > 0.0 ? ax : -ax;
                    lny = ly > 0.0 ? ay : -ay;
                    double len = qSqrt(lnx*lnx + lny*lny);
                    if (len > 0.001) {
                        lnx /= len;
                        lny /= len;
                    }
                } else if (ax > 0.0) {
                    lnx = lx > 0.0 ? 1.0 : -1.0;
                    lny = 0.0;
                } else {
                    lnx = 0.0;
                    lny = ly > 0.0 ? 1.0 : -1.0;
                }
            } else {
                if (ax > ay) {
                    lnx = lx > 0.0 ? 1.0 : -1.0;
                    lny = 0.0;
                } else {
                    lnx = 0.0;
                    lny = ly > 0.0 ? 1.0 : -1.0;
                }
            }

            outNormalX = lnx * cosR - lny * sinR;
            outNormalY = lnx * sinR + lny * cosR;
            return dist;
        } else if (obs.shape == ObstacleShape::Triangle) {
            double r = obs.radius;
            double vxs[3] = {0.0, -r * 0.8660254, r * 0.8660254};
            double vys[3] = {r, -r * 0.5, -r * 0.5};

            double dx = px - obs.x;
            double dy = py - obs.y;
            double cosR = qCos(obs.rotation);
            double sinR = qSin(obs.rotation);
            double lx = dx * cosR + dy * sinR;
            double ly = -dx * sinR + dy * cosR;

            QPointF V0(vxs[0], vys[0]);
            QPointF V1(vxs[1], vys[1]);
            QPointF V2(vxs[2], vys[2]);

            auto sqDistToSegment = [](const QPointF& p, const QPointF& a, const QPointF& b, QPointF& outClosest) -> double {
                double abx = b.x() - a.x();
                double aby = b.y() - a.y();
                double apx = p.x() - a.x();
                double apy = p.y() - a.y();
                double ab2 = abx*abx + aby*aby;
                double t = 0.0;
                if (ab2 > 0.0001) {
                    t = (apx*abx + apy*aby) / ab2;
                }
                t = qBound(0.0, t, 1.0);
                outClosest = QPointF(a.x() + t * abx, a.y() + t * aby);
                double cpx = p.x() - outClosest.x();
                double cpy = p.y() - outClosest.y();
                return cpx*cpx + cpy*cpy;
            };

            QPointF P(lx, ly);
            QPointF c0, c1, c2;
            double d0 = sqDistToSegment(P, V1, V2, c0);
            double d1 = sqDistToSegment(P, V2, V0, c1);
            double d2 = sqDistToSegment(P, V0, V1, c2);

            double minDistSq = d0;
            QPointF closest = c0;
            if (d1 < minDistSq) { minDistSq = d1; closest = c1; }
            if (d2 < minDistSq) { minDistSq = d2; closest = c2; }

            double dist = qSqrt(minDistSq);

            double cross0 = (lx - V1.x())*(V0.y() - V1.y()) - (ly - V1.y())*(V0.x() - V1.x());
            double cross1 = (lx - V2.x())*(V1.y() - V2.y()) - (ly - V2.y())*(V1.x() - V2.x());
            double cross2 = (lx - V0.x())*(V2.y() - V0.y()) - (ly - V0.y())*(V2.x() - V0.x());
            bool inside = (cross0 <= 0 && cross1 <= 0 && cross2 <= 0) || (cross0 >= 0 && cross1 >= 0 && cross2 >= 0);

            if (inside) {
                dist = -dist;
            }

            double lnx = lx - closest.x();
            double lny = ly - closest.y();
            double len = qSqrt(lnx*lnx + lny*lny);
            if (len > 0.001) {
                lnx /= len;
                lny /= len;
            } else {
                lnx = 0.0;
                lny = 1.0;
            }
            if (inside) {
                lnx = -lnx;
                lny = -lny;
            }

            outNormalX = lnx * cosR - lny * sinR;
            outNormalY = lnx * sinR + lny * cosR;
            return dist;
        }
        outNormalX = 1.0; outNormalY = 0.0;
        return 1e9;
    }
}

SimulationEngine::SimulationEngine(QObject* parent) 
    : QObject(parent), m_running(false), m_workerThread(nullptr) {
    std::srand(std::time(nullptr)); // Seed random for GPS drift
}

SimulationEngine::~SimulationEngine() {
    stopSimulation();
}

void SimulationEngine::startSimulation(unsigned int seed, double landProp, int width, int height, int numStaticObstacles, int numDynamicObstacles) {
    if (m_running) stopSimulation();
    
    // Re-initialize terrain with the provided seed and preset dimensions
    m_terrain = TerrainMap(seed, landProp, width, height, 1.0, numStaticObstacles);
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
    int obstacleCount = (numDynamicObstacles < 0) ? (3 + (std::rand() % 4)) : numDynamicObstacles;
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
        Drone& d = it->second;
        if (d.status == DroneStatus::Landed && d.returningToBase) {
            double dx = d.x - m_baseX;
            double dy = d.y - m_baseY;
            double dist = qSqrt(dx*dx + dy*dy);
            
            if (dist < 10.0) { // Within 10m of helipad center (accommodates grid landing)
                d.returningToBase = false;
                d.navQueue.clear();
                
                // Maintain sorted order by ID when returning to inventory
                auto insertPos = std::lower_bound(m_baseInventory.begin(), m_baseInventory.end(), d,
                    [](const Drone& a, const Drone& b) { return a.id < b.id; });
                m_baseInventory.insert(insertPos, d);
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
            std::unique_lock<std::shared_mutex> lock(m_mutex);
            updateDynamicObstacles(dt);
            processHangarLogic();
            updateHangarDrones();
            for (auto& pair : m_drones) {
                updateDroneState(pair.second, dt);
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

    if (drone.status == DroneStatus::Crashed) {
        if (drone.batteryLevel > 0.0) {
            drone.batteryLevel -= AeroGrid::Physics::HOVER_CONSUMPTION;
            if (drone.batteryLevel < 0.0) drone.batteryLevel = 0.0;
        }
        return;
    }

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

        double coarseMinDist = drone.radius + obs.radius;
        double coarseSafeZone = coarseMinDist + AeroGrid::Physics::STATIC_OBS_SAFETY_MARGIN;

        double obsGroundHeight = m_terrain.getHeightAt(obs.x, obs.y);
        double absoluteObsHeight = obsGroundHeight + obs.height;

        if (drone.z < absoluteObsHeight + 2.0) {
            if (dist2D < coarseSafeZone) {
                double normalX = 0.0, normalY = 0.0;
                double sdfDist = getDistanceToObstacle(obs, drone.x, drone.y, normalX, normalY);

                double minDist = drone.radius;
                double safeZone = minDist + AeroGrid::Physics::STATIC_OBS_SAFETY_MARGIN;

                if (sdfDist < safeZone) {
                    drone.proximityAlert = true;
                    double push = (safeZone - sdfDist) * AeroGrid::Physics::REPULSION_FORCE_STATIC;
                    if (drone.status == DroneStatus::Flying) {
                        drone.vx += normalX * push;
                        drone.vy += normalY * push;
                    }
                }

                if (sdfDist < minDist && drone.z < absoluteObsHeight) {
                    double speed = qSqrt(drone.vx*drone.vx + drone.vy*drone.vy + drone.vz*drone.vz);
                    if (speed >= AeroGrid::Physics::MAX_SAFE_LANDING_SPEED) {
                        drone.status = DroneStatus::Crashed;
                        drone.vx = drone.vy = drone.vz = 0;
                    }
                    double overlap = minDist - sdfDist;
                    drone.x += normalX * overlap;
                    drone.y += normalY * overlap;
                }
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
            double rvx = drone.vx - obs.vx;
            double rvy = drone.vy - obs.vy;
            double rvz = drone.vz - obs.vz;
            double relativeSpeed = qSqrt(rvx*rvx + rvy*rvy + rvz*rvz);
            if (relativeSpeed >= AeroGrid::Physics::MAX_SAFE_LANDING_SPEED) {
                drone.status = DroneStatus::Crashed;
                drone.vx = drone.vy = drone.vz = 0;
            } else {
                if (dist > 0.001) {
                    double nx = dx / dist; double ny = dy / dist; double nz = dz / dist;
                    drone.x = obs.x + nx * minDist;
                    drone.y = obs.y + ny * minDist;
                    drone.z = obs.z + nz * minDist;
                } else {
                    drone.x += minDist;
                }
            }
        }
    }
}

void SimulationEngine::checkGroundContact(Drone& drone, double groundHeight) {
    if (drone.z <= groundHeight + 0.05) {
        double speed = qSqrt(drone.vx*drone.vx + drone.vy*drone.vy + drone.vz*drone.vz);
        if (groundHeight < AeroGrid::World::LAND_THRESHOLD) {
            // Water collision: always crash
            drone.status = DroneStatus::Crashed;
            drone.z = groundHeight;
            drone.vx = drone.vy = drone.vz = 0;
        } else {
            // Ground contact:
            if (speed < AeroGrid::Physics::MAX_SAFE_LANDING_SPEED && drone.vz <= 0.05) {
                // Controlled safe landing on dry ground
                drone.status = DroneStatus::Landed;
                drone.z = groundHeight;
                drone.vx = drone.vy = drone.vz = 0;
            } else {
                // High-speed contact: crash
                drone.status = DroneStatus::Crashed;
                drone.z = groundHeight;
                drone.vx = drone.vy = drone.vz = 0;
            }
        }
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
    for (auto it1 = m_drones.begin(); it1 != m_drones.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != m_drones.end(); ++it2) {
            auto& d1 = it1->second;
            auto& d2 = it2->second;

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
                double rvx = d1.vx - d2.vx;
                double rvy = d1.vy - d2.vy;
                double rvz = d1.vz - d2.vz;
                double relativeSpeed = qSqrt(rvx*rvx + rvy*rvy + rvz*rvz);
                if (relativeSpeed >= AeroGrid::Physics::MAX_SAFE_LANDING_SPEED) {
                    d1.status = d2.status = DroneStatus::Crashed;
                    d1.vx = d1.vy = d1.vz = 0;
                    d2.vx = d2.vy = d2.vz = 0;
                } else {
                    double nx = dx / dist; double ny = dy / dist; double nz = dz / dist;
                    double overlap = minDist - dist;
                    if (d1.status != DroneStatus::Landed) {
                        d1.x += nx * overlap * 0.5;
                        d1.y += ny * overlap * 0.5;
                        d1.z += nz * overlap * 0.5;
                    }
                    if (d2.status != DroneStatus::Landed) {
                        d2.x -= nx * overlap * 0.5;
                        d2.y -= ny * overlap * 0.5;
                        d2.z -= nz * overlap * 0.5;
                    }
                }
            }
        }
    }
}

std::vector<Drone> SimulationEngine::getDroneData() {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    std::vector<Drone> drones;
    drones.reserve(m_drones.size());
    for (const auto& pair : m_drones) {
        drones.push_back(pair.second);
    }
    return drones;
}

const Drone* SimulationEngine::getDroneById(int id) const {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    auto it = m_drones.find(id);
    return (it != m_drones.end()) ? &(it->second) : nullptr;
}

std::vector<DynamicObstacle> SimulationEngine::getDynamicObstacleData() {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    return m_dynamicObstacles;
}

std::vector<Drone> SimulationEngine::getInventoryData() {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    return m_baseInventory;
}

int SimulationEngine::getInventoryCount() {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    return (int)m_baseInventory.size();
}

void SimulationEngine::launchDrone() {
    std::unique_lock<std::shared_mutex> lock(m_mutex);
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
    
    m_drones.insert_or_assign(d.id, std::move(d));
}

void SimulationEngine::launchDrones(const std::vector<int>& ids) {
    std::unique_lock<std::shared_mutex> lock(m_mutex);
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
            
            m_drones.insert_or_assign(d.id, std::move(d));
        }
    }
}

void SimulationEngine::assignTarget(int id, double x, double y, NavigationMode mode) {
    std::unique_lock<std::shared_mutex> lock(m_mutex);
    auto it = m_drones.find(id);
    if (it != m_drones.end()) {
        Drone& drone = it->second;
        drone.navQueue.push_back({x, y, 0.0}); // Z is calculated by mode in loop
        drone.navMode = mode;
        drone.returningToBase = false; // Manual target overrides RTB
    }
}

void SimulationEngine::takeOff(int id) {
    std::unique_lock<std::shared_mutex> lock(m_mutex);
    auto it = m_drones.find(id);
    if (it != m_drones.end()) {
        Drone& drone = it->second;
        if (drone.status == DroneStatus::Landed) {
            double groundHeight = m_terrain.getHeightAt(drone.x, drone.y);
            double batteryRequiredToTakeoff = drone.calculateBatteryRequiredToLand(groundHeight);

            if (drone.batteryLevel > batteryRequiredToTakeoff) {
                drone.status = DroneStatus::Flying;
                drone.vz = 2.0; // Initial ascent thrust
            }
        }
    }
}

void SimulationEngine::returnToBase(int id) {
    std::unique_lock<std::shared_mutex> lock(m_mutex);
    auto it = m_drones.find(id);
    if (it != m_drones.end()) {
        Drone& drone = it->second;
        if (drone.status == DroneStatus::Flying) {
            drone.navQueue.clear();
            drone.navQueue.push_back({m_baseX, m_baseY, 0.0});
            drone.returningToBase = true;
        }
    }
}

void SimulationEngine::landDrone(int id) {
    std::unique_lock<std::shared_mutex> lock(m_mutex);
    auto it = m_drones.find(id);
    if (it != m_drones.end()) {
        Drone& drone = it->second;
        if (drone.status == DroneStatus::Flying) {
            drone.status = DroneStatus::Landing;
            drone.navQueue.clear();
        }
    }
}

void SimulationEngine::clearNavQueue(int id) {
    std::unique_lock<std::shared_mutex> lock(m_mutex);
    auto it = m_drones.find(id);
    if (it != m_drones.end()) {
        Drone& drone = it->second;
        drone.navQueue.clear();
        drone.vx = drone.vy = drone.vz = 0; // Stop moving
    }
}

void SimulationEngine::removeNavPoint(int id, int index) {
    std::unique_lock<std::shared_mutex> lock(m_mutex);
    auto it = m_drones.find(id);
    if (it != m_drones.end()) {
        Drone& drone = it->second;
        if (index >= 0 && index < (int)drone.navQueue.size()) {
            drone.navQueue.erase(drone.navQueue.begin() + index);
        }
    }
}