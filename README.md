# AeroGrid - Drone Telemetry & Simulation Center

AeroGrid is a high-performance, real-time drone simulation environment built with C++ and Qt. It supports multi-drone operations with integrated collision avoidance, formation flying, and terrain-aware navigation. The simulator is designed for testing drone fleet management software and autonomous flight algorithms.

## Key Features

### Physics Engine
*   **Real-time Simulation**: Multi-threaded physics running at 20Hz (50ms per frame) handling 3D kinematics, gravity, and sensor noise.
*   **3D Orientation**: Real-time yaw/pitch/roll calculation based on flight dynamics.
*   **FPV Camera View with Z-Buffer Occlusion**:
    *   A pixel-level software depth-buffering algorithm (`std::vector<std::vector<double>> zBuffer`) prevents drones from seeing through hills or inside terrain blocks.
    *   **3D Cylindrical Shading** on static cylinders and **Spherical Shading** on dynamic spheres create a premium 3D aesthetic.
    *   **Flat-Shaded Polygon Face Rendering** for Rectangles and Triangles, including perspective-correct depth interpolation and dark outlines.
    *   **Interactive Crosshairs**: Overlay elements showing target coordinates and center guidance.
*   **Accurate Battery Management**: 
    *   Hovering: 0.05% per frame + 0.01% per m/s of velocity.
    *   Hangar Charging: 0.5% per frame (10% per second) for accelerated charging.
    *   Landed Charging: 0.05% per frame (1.0% per second) via solar panels.
    *   Emergency Landing: Automatic descent when battery drops below safe landing threshold.
    *   **Crashed Battery Drainage**: Crashed drones continue to drain battery at the hover rate, maintaining active camera noise until battery reaches 0%.
*   **Realistic Kinematics**: 
    *   5 m/s maximum cruise velocity.
    *   Fixed ascent/descent rates (2.0 m/s ascent, 5.0 m/s aggressive descent, 1.5 m/s controlled descent).
    *   Air resistance and velocity damping.

### Collision & Crash Physics
*   **Symmetric Repulsion Fields**: Drones maintain a 4m safety buffer from each other.
*   **SDF-Based Static Obstacle Avoidance**:
    *   Drones navigate around structures using a proximity warning zone (5.0m safety margin + drone radius = 5.3m) based on 2D Signed Distance Fields (SDF).
    *   Low-speed bumps act as gentle nudges, sliding the drone along the obstacle boundaries.
    *   High-speed contacts (relative speed $\ge 2.0$ m/s) with terrain, obstacles, or other drones result in an immediate **Crash** status.
*   **Water Safety Limits**:
    *   Heights below 1.0 represent water. Drones cannot land on water; any contact with water immediately results in a crash.
*   **Safe Landing Limits**:
    *   Controlled landing is permitted only on land (height $\ge 1.0$) at safe speeds (speed $< 2.0$ m/s and vertical speed $\le 0.05$ m/s).
*   **FPV Camera Crash Behavior**:
    *   Crashed drones display active analog static noise on their camera widget while battery remains.
    *   Once the battery hits 0%, the feed transitions to a completely black screen displaying a dim gray warning overlay: `"NO SIGNAL (BATTERY DEPLETED)"`.
*   **Dynamic Hazard Avoidance**: 
    *   Moving obstacles (birds, unauthorized aircraft) bounce off boundaries.
    *   3D repulsion forces adjust drone velocity in real-time.

### Navigation & Flight Modes
*   **Three Flight Modes**:
    1. **Manual**: Drones hover at their current location unless assigned a specific target.
    2. **Max Altitude**: Drones climb to a standard cruise altitude (80m) during transit for safe clearance.
    3. **Terrain Skimming**: Drones maintain a constant 5m Altitude Above Ground Level (AGL) using the terrain heightmap.
*   **Formation Flying**: 
    *   Intelligent coordinate offsetting based on drone ID (3-column grid pattern) prevents drones from converging on a single point and maintains grid formation during navigation.
*   **Navigation Queue**: 
    *   Support for multi-waypoint missions with real-time editing (add, remove, clear) and return-to-base (RTB) landing sequences.

### Telemetry UI
*   **Real-time Data Grid**:
    *   7-column display: Drone ID, Position (X/Y/Z), Battery, Status, Proximity.
    *   Stable multi-selection (Ctrl/Shift supported) with persistent selection during updates.
*   **Visual Alerts**: 
    *   Color-coded rows for status (⚠️ Proximity warnings, Red for crashes, Orange for low battery).
*   **Dual-View System**:
    *   Deployed Drones view: Active aircraft with full telemetry.
    *   Hangar Inventory view: Drones waiting to be deployed, showing charge status.
*   **Dynamic Sorting**: Click column headers to sort by any metric.
*   **Interactive Dropdowns**: Combo box selections for map size presets and flight modes clear focus asynchronously, ensuring immediate menu closure upon left-click.

### Interactive Map
*   **Procedural Terrain Rendering**:
    *   Perlin noise-based heightmap generation with configurable land/water distribution (0-100%).
*   **Land-Only Static Buildings**:
    *   All buildings are placed strictly on dry land. The generator verifies that the center and all corner vertices of a building footprint are on land (height $\ge 1.0$).
*   **Procedural Geometric Shapes**:
    *   Supports Cylinders, oriented Rectangles, and Triangles.
    *   Supports compound architectural structures: L-shapes, Step Pyramids, and Fortress clusters.
    *   Renders exact rotated outline shapes on the top-down 2D map.

## Controls

| Action | Input |
| :--- | :--- |
| **Select Drone** | Left Click (Map or List) |
| **Multi-Select** | Ctrl + Click or Shift + Click |
| **Box Select** | Left Click + Drag on Map |
| **Set Waypoint** | Right Click on Map (Selected drones only) |
| **Remove Waypoint** | Click on a path dot (white circle) |
| **Deploy Drones** | Context menu or deployment panel |
| **Takeoff/Land** | Control buttons in UI |
| **Return to Base** | RTB command button |

## Technical Architecture

### Threading Model
The `SimulationEngine` runs in a dedicated `QThread` to prevent UI blocking during heavy physics calculations. Data exchange with the UI is handled via:
*   **Mutex-Protected Access**: All drone state queries use `std::shared_mutex` for thread-safe concurrent reads.
*   **Signal/Slot Communication**: Physics thread emits `simulationUpdated()` signal to trigger UI refreshes.
*   **Snapshot Pattern**: Deep copies of drone data prevent race conditions.

### Model/View Architecture
The project uses Qt's Model/View pattern with a custom `QAbstractTableModel` (`TelemetryModel`):
*   **Data Caching**: Local drone vector enables O(1) sorting and efficient filtering.
*   **Optimized Updates**: Uses `dataChanged()` signals instead of full model resets.

### Physics Loop (20Hz)
Each 50ms frame executes:
1. **Update Obstacles**: Move dynamic obstacles, handle boundary bouncing.
2. **Hangar Logic**: Process return-to-base transitions and hangar inventory.
3. **Per-Drone Updates**:
   *   Status transitions and battery consumption.
   *   Navigation queue processing and waypoint navigation.
   *   Altitude mode adjustment (MaxAltitude/TerrainSkimming).
4. **Collision Detection & Resolution**:
   *   SDF-based static obstacle culling, proximity warning, and boundary resolution.
   *   Dynamic obstacle avoidance (birds).
   *   Drone-to-drone repulsion fields.
5. **Physics Application**: Update positions via velocity vectors.
6. **Sensor Simulation**: GPS drift noise, signal strength decay.
7. **Signal Emission**: Notify UI of frame completion.

## Build Requirements
*   **C++17** Compatible Compiler (GCC 7+, Clang 5+, MSVC 2017+)
*   **Qt 6.x** Framework (6.10+ recommended)
*   **CMake 3.10+** for build configuration

## Building AeroGrid

### Using CMake
```bash
cmake --build build --target AeroGrid
./build/AeroGrid
```

### Testing
```bash
cmake --build build --target AeroGridTests
./build/AeroGridTests
```

The test suite provides comprehensive coverage of:
*   Drone state initialization and transitions.
*   Physics simulation accuracy (battery, altitude, velocity).
*   SDF-based collision detection and crash physics.
*   Navigation queue processing and formation flight offsets.
*   Terrain sampling, dry-land footprint checks, and custom map presets.
*   UI model updates.

## Simulation Parameters

### Drone Properties
*   **Radius**: 0.3m (collision detection).
*   **Max Speed**: 5 m/s.
*   **Initial Battery**: 100%.
*   **Signal Range**: 300m maximum.

### Physics Constants
*   **Gravity**: 9.81 m/s²
*   **Frame Time**: 50ms (20Hz)
*   **Battery Hover Cost**: 0.05% per frame (1.0% per second)
*   **Battery Speed Cost**: 0.01% per m/s of velocity
*   **Landed Charge Rate**: 0.05% per frame (1.0% per second)
*   **Hangar Charge Rate**: 0.5% per frame (10% per second)
*   **Max Safe Landing Speed**: 2.0 m/s
*   **Ascent Speed**: 2.0 m/s
*   **Aggressive Landing Speed**: 5.0 m/s (downward)
*   **Controlled Landing Speed**: 1.5 m/s (downward)

### World Constants
*   **Drone Safety Buffer**: 4m (symmetric repulsion)
*   **Static Obstacle Safety Margin**: 5m (added to SDF)
*   **Dynamic Obstacle Safety Margin**: 5m
*   **Base Placement Range**: ±25% of world dimensions
*   **Helipad Safety Zone**: ±20% of world dimensions

## File Structure

```
AeroGrid/
├── CMakeLists.txt           # Build configuration
├── README.md                # This file
├── src/
│   ├── main.cpp             # Qt application entry point
│   ├── core/
│   │   ├── Drone.h          # Drone state structure
│   │   ├── SimulationEngine.h/cpp  # Main physics loop & SDF collision math
│   │   ├── TerrainMap.h/cpp  # Procedural terrain generation & shapes
│   ├── gui/
│   │   ├── MainWindow.h/cpp  # Main UI window & 3D FPV camera widget
│   │   ├── TelemetryModel.h/cpp  # Table model for drone data
│   │   ├── TerrainView.h/cpp  # Map visualization
└── tests/
    └── TestAeroGrid.cpp      # Comprehensive test suite
```

---

**Last Updated**: May 2026  
**Current Version**: 1.1  
**Status**: Active Development