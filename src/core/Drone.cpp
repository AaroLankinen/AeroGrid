#include "Drone.h"
#include "Constants.h"
#include <cmath>
#include <algorithm>

DroneController::DroneController(int droneId)
    : m_droneId(droneId) {
    m_sensors = {};
}

void DroneController::updateSensors(const DroneSensors& sensors) {
    m_sensors = sensors;
    if (!m_homeInitialized) {
        m_homeX = sensors.gpsX;
        m_homeY = sensors.gpsY;
        m_homeInitialized = true;
    }
}

void DroneController::receiveCommand(const DroneCommand& cmd) {
    if (m_signalLost) return; // Command lost in RF transit

    switch (cmd.type) {
        case DroneCommand::Type::AssignTarget:
            m_localNavQueue.push_back({cmd.x, cmd.y, 0.0});
            m_navMode = cmd.mode;
            m_returningToBase = false;
            m_failsafeRTBActive = false;
            break;
        case DroneCommand::Type::ClearQueue:
            m_localNavQueue.clear();
            m_returningToBase = false;
            m_failsafeRTBActive = false;
            m_userLanding = false;
            break;
        case DroneCommand::Type::RemoveWaypoint:
            if (cmd.index >= 0 && cmd.index < (int)m_localNavQueue.size()) {
                m_localNavQueue.erase(m_localNavQueue.begin() + cmd.index);
            }
            break;
        case DroneCommand::Type::Land:
            m_userLanding = true;
            m_localNavQueue.clear();
            m_returningToBase = false;
            m_failsafeRTBActive = false;
            break;
        case DroneCommand::Type::RTB:
            m_localNavQueue.clear();
            m_localNavQueue.push_back({cmd.x, cmd.y, 0.0});
            m_returningToBase = true;
            m_failsafeRTBActive = false;
            m_userLanding = false;
            break;
        case DroneCommand::Type::Takeoff:
            m_takingOff = true;
            m_userLanding = false;
            m_emergencyLanding = false;
            break;
    }
}

void DroneController::setSignalLost(bool lost) {
    if (m_signalLost != lost) {
        m_signalLost = lost;
        if (m_signalLost) {
            m_timeSinceSignalLoss = 0.0;
            m_failsafeHoverZ = m_sensors.gpsZ;
        } else {
            m_failsafeRTBActive = false;
        }
    }
}

double DroneController::calculateBatteryRequiredToLand() const {
    using namespace AeroGrid::Physics;
    double altitudeAGL = std::max(0.0, m_sensors.groundAltitudeBelow);
    
    double t_fast = std::max(0.0, (altitudeAGL - MIN_SAFE_ALTITUDE_AGL) / std::abs(LANDING_FAST_SPEED));
    double t_slow = std::min(altitudeAGL, MIN_SAFE_ALTITUDE_AGL) / std::abs(LANDING_SLOW_SPEED);
    
    return (t_fast * FAST_LANDING_BATTERY_COST) + (t_slow * SLOW_LANDING_BATTERY_COST) + LANDING_SAFETY_MARGIN;
}

void DroneController::applyAvoidanceForces(double& vx, double& vy, double& vz) {
    m_proximityAlert = false;
    double repX = 0.0;
    double repY = 0.0;

    const double staticSensorRange = AeroGrid::Physics::STATIC_OBS_SAFETY_MARGIN;
    const double droneSensorRange = AeroGrid::Physics::DRONE_TO_DRONE_SAFETY_BUFFER;
    
    for (const auto& read : m_sensors.proximityPoints) {
        double range = read.isDynamic ? droneSensorRange : staticSensorRange;
        if (read.distance < range && read.distance > 0.01) {
            m_proximityAlert = true;
            
            double push = (range - read.distance) * (read.isDynamic ? AeroGrid::Physics::REPULSION_FORCE_DRONE : AeroGrid::Physics::REPULSION_FORCE_STATIC);
            
            double yawRad = m_sensors.yaw * (M_PI / 180.0);
            double absAngle = yawRad + read.angle;
            
            double dirX = -std::cos(absAngle);
            double dirY = -std::sin(absAngle);
            
            repX += dirX * push;
            repY += dirY * push;
        }
    }
    
    if (!m_userLanding && !m_emergencyLanding) {
        vx += repX;
        vy += repY;
    }
}

void DroneController::getControlOutputs(double& outVx, double& outVy, double& outVz) {
    double batteryRequiredToLand = calculateBatteryRequiredToLand();
    if (m_sensors.batteryLevel <= batteryRequiredToLand && !m_emergencyLanding) {
        m_emergencyLanding = true;
        m_localNavQueue.clear();
    }

    if (m_sensors.batteryLevel <= 0.0) {
        outVx = outVy = outVz = 0.0;
        return;
    }

    if (m_signalLost) {
        m_timeSinceSignalLoss += AeroGrid::Physics::DT;
        
        if (m_timeSinceSignalLoss < 5.0) {
            // Failsafe Phase 1: Hover at recorded Z height
            double dz = m_failsafeHoverZ - m_sensors.gpsZ;
            outVx = 0.0;
            outVy = 0.0;
            if (std::abs(dz) > 0.2) {
                outVz = (dz > 0) ? AeroGrid::Physics::ASCENT_SPEED : AeroGrid::Physics::LANDING_SLOW_SPEED;
            } else {
                outVz = 0.0;
            }
            applyAvoidanceForces(outVx, outVy, outVz);
            return;
        } else if (m_timeSinceSignalLoss < 10.0 && m_sensors.gpsZ < AeroGrid::Physics::STANDARD_CRUISE_ALTITUDE) {
            // Failsafe Phase 2: Climb to standard cruise altitude
            outVx = 0.0;
            outVy = 0.0;
            outVz = AeroGrid::Physics::ASCENT_SPEED;
            applyAvoidanceForces(outVx, outVy, outVz);
            return;
        } else {
            // Failsafe Phase 3: Autonomous Return to Base
            if (!m_failsafeRTBActive) {
                m_failsafeRTBActive = true;
                m_localNavQueue.clear();
                m_localNavQueue.push_back({m_homeX, m_homeY, 0.0});
            }
        }
    }

    if (m_userLanding || m_emergencyLanding) {
        outVx = 0.0;
        outVy = 0.0;
        outVz = (m_sensors.groundAltitudeBelow > AeroGrid::Physics::MIN_SAFE_ALTITUDE_AGL) 
                ? AeroGrid::Physics::LANDING_FAST_SPEED 
                : AeroGrid::Physics::LANDING_SLOW_SPEED;
    }

    if (m_takingOff) {
        if (m_sensors.groundAltitudeBelow < AeroGrid::Physics::MIN_SAFE_ALTITUDE_AGL) {
            outVx = 0.0;
            outVy = 0.0;
            outVz = AeroGrid::Physics::ASCENT_SPEED;
            applyAvoidanceForces(outVx, outVy, outVz);
            return;
        } else {
            m_takingOff = false;
        }
    }

    if (m_userLanding || m_emergencyLanding) {
        outVx = 0.0;
        outVy = 0.0;
        outVz = (m_sensors.groundAltitudeBelow > AeroGrid::Physics::MIN_SAFE_ALTITUDE_AGL) 
                ? AeroGrid::Physics::LANDING_FAST_SPEED 
                : AeroGrid::Physics::LANDING_SLOW_SPEED;
    } else if (!m_localNavQueue.empty()) {
        const auto& target = m_localNavQueue.front();
        double offsetX = (m_droneId % 3 - 1) * 4.0;
        double offsetY = (m_droneId / 3 - 1) * 4.0;
        double tx = target.x + offsetX;
        double ty = target.y + offsetY;
        double tz = target.z;

        if (m_navMode == NavigationMode::MaxAltitude) {
            tz = AeroGrid::Physics::STANDARD_CRUISE_ALTITUDE;
        } else if (m_navMode == NavigationMode::TerrainSkimming) {
            double groundHeightEst = m_sensors.gpsZ - m_sensors.groundAltitudeBelow;
            tz = groundHeightEst + AeroGrid::Physics::MIN_SAFE_ALTITUDE_AGL;
        }

        double dx = tx - m_sensors.gpsX;
        double dy = ty - m_sensors.gpsY;
        double dz = tz - m_sensors.gpsZ;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist > AeroGrid::Physics::MIN_DRONE_DIST) {
            outVx = (dx / dist) * AeroGrid::Physics::CRUISE_SPEED;
            outVy = (dy / dist) * AeroGrid::Physics::CRUISE_SPEED;
            outVz = (dz / dist) * AeroGrid::Physics::CRUISE_SPEED;
        } else {
            m_localNavQueue.erase(m_localNavQueue.begin());
            if (m_returningToBase && m_localNavQueue.empty()) {
                m_userLanding = true;
            }
            outVx = outVy = outVz = 0.0;
        }
    } else {
        double groundHeightEst = m_sensors.gpsZ - m_sensors.groundAltitudeBelow;
        double minSafeZ = groundHeightEst + AeroGrid::Physics::MIN_SAFE_ALTITUDE_AGL;
        if (m_sensors.gpsZ < minSafeZ - 0.1) {
            outVx = outVy = 0.0;
            outVz = std::abs(AeroGrid::Physics::LANDING_SLOW_SPEED);
        } else {
            outVx = outVy = 0.0;
            outVz = 0.0;
        }
    }

    applyAvoidanceForces(outVx, outVy, outVz);
}
