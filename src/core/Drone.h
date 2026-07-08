#pragma once
#include <vector>
#include <algorithm>
#include <QtGlobal>
#include "Constants.h"

/**
 * @brief Defines the possible operational states of a drone.
 */
enum class DroneStatus {
    Flying,             ///< Active flight, consuming battery based on speed.
    Crashed,            ///< Drone has collided with an obstacle or another drone.
    Disconnected,       ///< Drone has lost communication with the base station.
    Landing,            ///< User-initiated controlled descent.
    EmergencyLanding,   ///< Automated descent triggered by low battery.
    Landed              ///< Stationary on ground, solar charging active.
};

/**
 * @brief Defines how Z-axis (altitude) is managed during navigation.
 * 
 * Controls the altitude behavior when a drone is following a waypoint.
 */
enum class NavigationMode {
    Manual,             ///< Use target Z exactly as specified in the nav point.
    MaxAltitude,        ///< Transits at fixed high altitude (80m) for safe clearance.
    TerrainSkimming     ///< Transits at fixed AGL (5m above ground level) following terrain contours.
};

/**
 * @brief Represents a single navigation waypoint in 3D space.
 */
struct NavPoint {
    double x, y, z;     ///< 3D coordinates of the waypoint (meters).
};

/**
 * @brief Represents a single drone in the simulation.
 * 
 * Contains all state information for a drone including position, velocity,
 * battery status, and navigation queue. Drones are uniquely identified by ID.
 */
struct Drone {
    // Identification & State
    int id;                         ///< Unique identifier for the drone.
    
    // 3D Kinematics
    double x, y, z;                 ///< Position in 3D space (meters).
    double vx, vy, vz;              ///< Velocity vectors (meters per second).

    // 3D Orientation (Euler Angles in Degrees)
    double yaw;                     ///< Rotation around Z (heading).
    double pitch;                   ///< Rotation around Y (tilt).
    double roll;                    ///< Rotation around X (bank).
    
    // Power Management
    double batteryLevel;            ///< Percentage (0.0 - 100.0). Landed drones charge at +0.05% per tick.
    
    // Status & Properties
    DroneStatus status;             ///< Current operational state of the drone.
    double signalStrength;          ///< Radio signal strength (0.0 - 1.0), decays with distance.
    double radius;                  ///< Bounding radius in meters for collision detection.
    
    // Navigation
    NavigationMode navMode;         ///< How altitude is managed during flight.
    std::vector<NavPoint> navQueue; ///< Queue of waypoints to navigate to in order.
    
    // Special States
    bool returningToBase;           ///< True if drone is performing RTB (Return To Base) sequence.
    bool proximityAlert;            ///< True if drone is currently inside a repulsion field or near obstacles.
    bool takingOff;                 ///< True if the drone is in its vertical takeoff phase.

    /**
     * @brief Constructs a Drone with default initial state.
     * 
     * Initializes the drone at origin with full battery, hovering in manual mode,
     * and no active navigation targets.
     * 
     * @param _id The unique identifier for this drone.
     */
    Drone(int _id) : id(_id), x(0), y(0), z(0), vx(0), vy(0), vz(0),
                     yaw(0), 
                     pitch(AeroGrid::Physics::CAMERA_PITCH_IDLE), 
                     roll(0),
                     batteryLevel(100.0), status(DroneStatus::Flying),
                     signalStrength(1.0), radius(AeroGrid::Physics::DRONE_RADIUS), navMode(NavigationMode::Manual),
                     returningToBase(false), proximityAlert(false), takingOff(false) {}

    /**
     * @brief Centralized logic for calculating the minimum battery required for a safe descent.
     * 
     * Accounts for different descent speeds at different altitudes (fast/slow phases)
     * and includes a safety margin defined in Physics::LANDING_SAFETY_MARGIN.
     * 
     * @param groundHeight The terrain altitude at current position.
     * @return Required battery percentage.
     */
    double calculateBatteryRequiredToLand(double groundHeight) const {
        using namespace AeroGrid::Physics;
        double altitudeAGL = qMax(0.0, z - groundHeight);
        
        double t_fast = qMax(0.0, (altitudeAGL - MIN_SAFE_ALTITUDE_AGL) / qAbs(LANDING_FAST_SPEED));
        double t_slow = qMin(altitudeAGL, MIN_SAFE_ALTITUDE_AGL) / qAbs(LANDING_SLOW_SPEED);
        
        return (t_fast * FAST_LANDING_BATTERY_COST) + (t_slow * SLOW_LANDING_BATTERY_COST) + LANDING_SAFETY_MARGIN;
    }
};

/**
 * @brief Represents a single reading from a drone's proximity sensors.
 */
struct ProximityReading {
    double distance;     ///< Distance in meters to the obstacle.
    double angle;        ///< Heading bearing in radians relative to drone's yaw.
    bool isDynamic;      ///< True if dynamic obstacle (bird or other drone).
};

/**
 * @brief Telemetry sensors package fed to the DroneController.
 */
struct DroneSensors {
    double gpsX, gpsY, gpsZ;            ///< Estimated position with noise/drift.
    double groundAltitudeBelow;          ///< Altitude Above Ground Level (AGL) in meters.
    double yaw, pitch, roll;            ///< IMU attitude estimation (degrees).
    double vx, vy, vz;                  ///< IMU velocity vector (m/s).
    double batteryLevel;                ///< Battery level percentage (0 - 100).
    std::vector<ProximityReading> proximityPoints; ///< Local lidar/radar hits.
};

/**
 * @brief Represents a physical command transmitted from base station to drone.
 */
struct DroneCommand {
    enum class Type {
        AssignTarget,
        ClearQueue,
        RemoveWaypoint,
        Land,
        RTB,
        Takeoff
    };
    Type type;
    double x = 0.0;
    double y = 0.0;
    int index = -1;
    NavigationMode mode = NavigationMode::Manual;
};

/**
 * @brief Onboard autopilot system that handles local sensory-only navigation and failsafes.
 */
class DroneController {
public:
    explicit DroneController(int droneId = -1);

    /**
     * @brief Feed the autopilot fresh sensory estimations.
     */
    void updateSensors(const DroneSensors& sensors);

    /**
     * @brief Receive a command message over the radio link.
     */
    void receiveCommand(const DroneCommand& cmd);

    /**
     * @brief Process local potential field navigation and failsafes.
     * @return Desired velocity vectors.
     */
    void getControlOutputs(double& outVx, double& outVy, double& outVz);

    // Getters for internal autopilot state
    const std::vector<NavPoint>& getLocalNavQueue() const { return m_localNavQueue; }
    NavigationMode getLocalNavMode() const { return m_navMode; }
    bool isReturningToBase() const { return m_returningToBase; }
    bool isTakingOff() const { return m_takingOff; }
    bool hasProximityAlert() const { return m_proximityAlert; }
    bool isEmergencyLanding() const { return m_emergencyLanding; }
    bool isUserLanding() const { return m_userLanding; }

    void setLocalNavQueue(const std::vector<NavPoint>& queue) { m_localNavQueue = queue; }
    void setLocalNavMode(NavigationMode mode) { m_navMode = mode; }
    void setReturningToBase(bool rtb) { m_returningToBase = rtb; }
    void setTakingOff(bool takingOff) { m_takingOff = takingOff; }
    void setUserLanding(bool landing) { m_userLanding = landing; }
    void setEmergencyLanding(bool landing) { m_emergencyLanding = landing; }

    // Signal/Failsafe state management
    void setSignalLost(bool lost);
    bool isSignalLost() const { return m_signalLost; }

private:
    double calculateBatteryRequiredToLand() const;
    void applyAvoidanceForces(double& vx, double& vy, double& vz);

    int m_droneId;
    DroneSensors m_sensors;
    std::vector<NavPoint> m_localNavQueue;
    NavigationMode m_navMode = NavigationMode::Manual;

    // Autopilot state flags
    bool m_returningToBase = false;
    bool m_takingOff = false;
    bool m_proximityAlert = false;
    bool m_emergencyLanding = false;
    bool m_userLanding = false;

    // Failsafe state variables
    bool m_signalLost = false;
    double m_timeSinceSignalLoss = 0.0;     ///< Time in seconds.
    double m_failsafeHoverZ = 0.0;          ///< Altitude to hover at during signal loss.
    bool m_failsafeRTBActive = false;       ///< True if failsafe RTB has been initiated.
    
    // Home base coordinates (sensory record)
    double m_homeX = 0.0;
    double m_homeY = 0.0;
    bool m_homeInitialized = false;
};