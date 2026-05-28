#pragma once
#include <vector>
#include <algorithm>
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

    /**
     * @brief Constructs a Drone with default initial state.
     * 
     * Initializes the drone at origin with full battery, hovering in manual mode,
     * and no active navigation targets.
     * 
     * @param _id The unique identifier for this drone.
     */
    Drone(int _id) : id(_id), x(0), y(0), z(0), vx(0), vy(0), vz(0), 
                     batteryLevel(100.0), status(DroneStatus::Flying), 
                     signalStrength(1.0), radius(AeroGrid::Physics::DRONE_RADIUS), navMode(NavigationMode::Manual),
                     returningToBase(false), proximityAlert(false) {}

    /**
     * @brief Calculates the battery percentage required to perform a safe landing from current altitude.
     * 
     * @param groundHeight The terrain altitude at current position.
     * @return Required battery percentage.
     */
    double calculateBatteryRequiredToLand(double groundHeight) const {
        using namespace AeroGrid::Physics;
        double altitudeAGL = std::max(0.0, z - groundHeight);
        
        double t_fast = std::max(0.0, (altitudeAGL - MIN_SAFE_ALTITUDE_AGL) / std::abs(LANDING_FAST_SPEED));
        double t_slow = std::min(altitudeAGL, MIN_SAFE_ALTITUDE_AGL) / std::abs(LANDING_SLOW_SPEED);
        
        return (t_fast * FAST_LANDING_BATTERY_COST) + (t_slow * SLOW_LANDING_BATTERY_COST) + LANDING_SAFETY_MARGIN;
    }
};