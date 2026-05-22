# AeroGrid - Drone Telemetry & Simulation Center

AeroGrid is a high-performance, real-time drone simulation environment built with C++ and Qt. It supports multi-drone operations with integrated collision avoidance, formation flying, and terrain-aware navigation.

## Key Features

*   **Physics Engine**: Multithreaded simulation running at 20Hz (50ms steps) handling 3D kinematics, gravity, and noise.
*   **Collision Avoidance**:
    *   **Symmetric Repulsion Fields**: Drones maintain a 4m safety buffer from each other.
    *   **Static Obstacles**: Automatic horizontal steering around buildings and towers.
    *   **Dynamic Hazards**: 3D avoidance logic to steer around moving entities (e.g., birds or unauthorized aircraft).
*   **Formation Flight**: Intelligent coordinate offsetting prevents drones from converging on a single point when commanded as a group, maintaining a grid formation.
*   **Telemetry UI**:
    *   Real-time data grid with stable multi-selection (Ctrl/Shift supported).
    *   **Visual Alerts**: Proximity warnings (⚠️) and color-coded battery/crash states.
*   **Interactive Map**: 
    *   Procedural terrain heatmap rendering.
    *   Rubber-band box selection for drones.
    *   Right-click to set waypoints based on current flight mode.

## Flight Modes

1.  **Manual**: Drones hover at their current location unless assigned a specific target.
2.  **Max Altitude**: Drones climb to a standard cruise altitude (80m) during transit.
3.  **Terrain Skimming**: Drones maintain a constant 5m Altitude Above Ground Level (AGL) using the terrain heightmap.

## Controls

| Action | Input |
| :--- | :--- |
| **Select Drone** | Left Click (Map or List) |
| **Multi-Select** | Ctrl + Click or Shift + Click |
| **Box Select** | Left Click + Drag on Map |
| **Set Waypoint** | Right Click on Map (Selected drones only) |
| **Remove Waypoint** | Click on a path dot (white circle) |

## Implementation Details

### Threading Model
The `SimulationEngine` runs in a dedicated `QThread`. Data exchange with the `TelemetryModel` is handled via a `std::mutex` and a thread-safe "Snap-shot" pattern to prevent UI stuttering during heavy physics calculations.

### Model/View Architecture
The project uses a custom `QAbstractTableModel` (`TelemetryModel`). It optimizes UI performance by using `dataChanged` signals instead of full model resets, ensuring that user selection and scroll position are preserved during high-frequency simulation updates.

## Build Requirements
*   C++17 Compatible Compiler
*   Qt 6.x Framework
*   CMake 3.10+