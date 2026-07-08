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

    // Camera and Underground
    constexpr double CAMERA_PITCH_IDLE = -15.0;
    constexpr double UNDERGROUND_THRESHOLD = 0.5;
}

/**
 * @brief Constants related to the Graphical User Interface.
 */
namespace UI {
    // Table Colors
    inline const QColor COLOR_TABLE_CRASHED = QColor(255, 200, 200);
    inline const QColor COLOR_TABLE_LOW_BATTERY = QColor(255, 230, 150);
    inline const QColor COLOR_TABLE_DISCONNECTED = QColor(220, 220, 220);

    // Zoom Parameters
    constexpr double MIN_ZOOM = 0.5;
    constexpr double MAX_ZOOM = 5.0;
    constexpr double ZOOM_FACTOR = 1.2;

    // Drone 3D/FPV Render Colors
    inline const QColor COLOR_SKY_TOP = QColor(60, 100, 200);
    inline const QColor COLOR_SKY_BOTTOM = QColor(100, 149, 237);
    inline const QColor COLOR_WATER = QColor(20, 60, 200);
    inline const QColor COLOR_LAND = QColor(34, 139, 34);
    inline const QColor COLOR_UNDERGROUND_DIRT = QColor(54, 38, 27);

    inline const QColor COLOR_OBSTACLE_OUTLINE = QColor(40, 5, 5);
    inline const QColor COLOR_DYNAMIC_OBSTACLE_OUTLINE = QColor(40, 20, 0);

    // Obstacle Base Colors
    inline const QColor COLOR_STATIC_OBSTACLE = QColor(200, 30, 30);
    inline const QColor COLOR_DYNAMIC_OBSTACLE = QColor(240, 120, 0);

    // 2D Map View Parameters
    namespace Map {
        constexpr double BASE_RADIUS = 15.0;
        constexpr double DRONE_RADIUS = 8.0;
        inline const QColor COLOR_STATIC_OBSTACLE = QColor(255, 0, 0, 180);
        inline const QColor COLOR_DYNAMIC_OBSTACLE = QColor(255, 140, 0, 200);
        inline const QColor COLOR_DRONE_NORMAL = Qt::cyan;
        inline const QColor COLOR_DRONE_CRASHED = Qt::red;
        inline const QColor COLOR_DRONE_SELECTED = Qt::yellow;
    }

    // Camera View Settings
    namespace Camera {
        constexpr int WIDGET_WIDTH = 240;
        constexpr int WIDGET_HEIGHT = 135;
        constexpr int NUM_COLS = 32;
        constexpr double TERRAIN_STEP = 3.0;
        constexpr double MAX_DRAW_DISTANCE = 150.0;
        constexpr double RENDERING_GAP_OFFSET = 0.5;
        constexpr double PROJECTION_K_X = 5.0;
        constexpr double PROJECTION_K_Y = 4.0;
        constexpr double REFERENCE_ROWS = 12.0;
    }
}

/**
 * @brief Global world and environmental parameters.
 */
namespace World {
    constexpr int DEFAULT_INVENTORY_COUNT = 12;
    constexpr double LAND_THRESHOLD = 1.0;
    constexpr double SIGNAL_MAX_RANGE = 300.0;

    // Generation and Safety Limits
    constexpr int SPAWN_SAFETY_LIMIT = 2000;
    constexpr int OBSTACLE_SPAWN_ATTEMPTS = 50;
    constexpr double OBSTACLE_SEPARATION_MARGIN = 4.0;
}
}