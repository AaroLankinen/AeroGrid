#pragma once
#include <QColor>

/**
 * @brief Centralized configuration for the AeroGrid project.
 * 
 * This namespace contains all physical, environmental, and UI constants
 * to ensure consistency across the simulation engine and the graphical interface.
 */
namespace AeroGrid {
/**
 * @brief Physical simulation and drone behavior parameters.
 */
namespace Physics {
    constexpr double GRAVITY = 9.81;
    constexpr double DT = 0.05;
    constexpr int TICK_MS = 50;

    // Speeds
    constexpr double CRUISE_SPEED = 5.0;
    constexpr double ASCENT_SPEED = 2.0;
    constexpr double LANDING_FAST_SPEED = -5.0;
    constexpr double LANDING_SLOW_SPEED = -1.5;
    constexpr double MAX_SAFE_LANDING_SPEED = 2.0;
    constexpr double MIN_DRONE_DIST = 1.0;

    // Altitudes
    constexpr double MIN_SAFE_ALTITUDE_AGL = 5.0;
    constexpr double STANDARD_CRUISE_ALTITUDE = 80.0;

    // Battery
    constexpr double LANDED_CHARGE_RATE = 0.05;
    constexpr double HANGAR_CHARGE_RATE = 0.5;
    constexpr double HOVER_CONSUMPTION = 0.05;
    constexpr double VELOCITY_CONSUMPTION_FACTOR = 0.01;
    constexpr double LANDING_SAFETY_MARGIN = 5.0;
    constexpr double FAST_LANDING_BATTERY_COST = 2.0; // % per second
    constexpr double SLOW_LANDING_BATTERY_COST = 1.3; // % per second

    // Collision & Avoidance
    constexpr double DRONE_RADIUS = 0.3;
    constexpr double STATIC_OBS_SAFETY_MARGIN = 5.0;
    constexpr double DYNAMIC_OBS_SAFETY_MARGIN = 5.0;
    constexpr double DRONE_TO_DRONE_SAFETY_BUFFER = 4.0;
    constexpr double REPULSION_FORCE_STATIC = 0.2;
    constexpr double REPULSION_FORCE_DYNAMIC = 0.3;
    constexpr double REPULSION_FORCE_DRONE = 0.2;

    constexpr double CAMERA_PITCH_IDLE = -15.0;
}

/**
 * @brief Constants related to the Graphical User Interface.
 */
namespace UI {
    // Table Colors
    inline const QColor COLOR_TABLE_CRASHED = QColor(255, 200, 200);
    inline const QColor COLOR_TABLE_LOW_BATTERY = QColor(255, 230, 150);

    // Zoom Parameters
    constexpr double MIN_ZOOM = 0.5;
    constexpr double MAX_ZOOM = 5.0;
    constexpr double ZOOM_FACTOR = 1.2;
}

/**
 * @brief Global world and environmental parameters.
 */
namespace World {
    constexpr int DEFAULT_INVENTORY_COUNT = 12;
    constexpr double LAND_THRESHOLD = 1.0;
    constexpr double SIGNAL_MAX_RANGE = 300.0;
}
}