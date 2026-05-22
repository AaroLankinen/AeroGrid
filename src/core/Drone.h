#pragma once
#include <vector>

enum class DroneStatus {
    Flying,
    Crashed,
    Disconnected,
    Landing,
    EmergencyLanding,
    Landed
};

enum class NavigationMode {
    Manual,
    MaxAltitude,
    TerrainSkimming
};

struct NavPoint {
    double x, y, z;
};

struct Drone {
    int id;
    double x, y, z;        // Position in 3D space
    double vx, vy, vz;     // Velocity vectors
    double batteryLevel;   // Percentage (0.0 - 100.0)
    DroneStatus status;
    double signalStrength; // 0.0 - 1.0
    double radius;         // Bounding radius in meters
    
    NavigationMode navMode;
    std::vector<NavPoint> navQueue;

    Drone(int _id) : id(_id), x(0), y(0), z(0), vx(0), vy(0), vz(0), 
                     batteryLevel(100.0), status(DroneStatus::Flying), 
                     signalStrength(1.0), radius(0.3), navMode(NavigationMode::Manual) {}
};