#pragma once
#include <vector>

/**
 * @brief Defines the possible operational states of a drone.
 */
enum class DroneStatus {
    Flying,             ///< Active flight, consuming battery based on speed.
    Crashed,
    Disconnected,
    Landing,            ///< User-initiated controlled descent.
    EmergencyLanding,   ///< Automated descent triggered by low battery.
    Landed              ///< Stationary on ground, solar charging active.
};

/**
 * @brief Defines how Z-axis (altitude) is managed during navigation.
 */
enum class NavigationMode {
    Manual,             ///< Use target Z exactly.
    MaxAltitude,        ///< Transits at fixed high altitude (80m).
    TerrainSkimming     ///< Transits at fixed AGL (5m) following terrain.
};

struct NavPoint {
    double x, y, z;
};

struct Drone {
    int id;
    double x, y, z;        // Position in 3D space
    double vx, vy, vz;     // Velocity vectors (m/s)
    double batteryLevel;   // Percentage (0.0 - 100.0). Landed drones charge +0.05%/tick.
    DroneStatus status;
    double signalStrength; // 0.0 - 1.0
    double radius;         // Bounding radius in meters for hard collisions.
    
    NavigationMode navMode;
    std::vector<NavPoint> navQueue;
    bool returningToBase;  ///< True if drone is performing RTB sequence.
    bool proximityAlert;   ///< True if drone is currently inside a repulsion field.

    Drone(int _id) : id(_id), x(0), y(0), z(0), vx(0), vy(0), vz(0), 
                     batteryLevel(100.0), status(DroneStatus::Flying), 
                     signalStrength(1.0), radius(0.3), navMode(NavigationMode::Manual),
                     returningToBase(false), proximityAlert(false) {}
};