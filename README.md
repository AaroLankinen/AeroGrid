# AeroGrid - Drone Telemetry & Simulation Center

AeroGrid is a high-performance, real-time drone simulation environment built with C++ and Qt. It supports multi-drone operations with integrated collision avoidance, formation flying, and terrain-aware navigation. The simulator is designed for testing drone fleet management software and autonomous flight algorithms.

## Key Features

### Physics Engine
*   **Real-time Simulation**: Multi-threaded physics running at 20Hz (50ms per frame) handling 3D kinematics, gravity, and sensor noise
*   **3D Orientation**: Real-time yaw/pitch/roll calculation based on flight dynamics
*   **Remote Camera Support**: Internal support for drone-mounted POV cameras with configurable FOV
*   **Accurate Battery Management**: 
    *   Hovering: 0.05% per frame + 0.01% per m/s of velocity
    *   Hangar Charging: 0.5% per frame (5% per second) for accelerated charging
    *   Landed Charging: 0.05% per frame (0.5% per second) via solar panels
    *   Emergency Landing: Automatic descent when battery drops below safe landing threshold
*   **Realistic Kinematics**: 
    *   5 m/s maximum cruise velocity
    *   Configurable ascent/descent rates (2.0 m/s aggressive, 1.5 m/s controlled)
    *   Air resistance and velocity damping

### Collision Avoidance
*   **Symmetric Repulsion Fields**: Drones maintain a 4m safety buffer from each other
*   **Static Obstacle Avoidance**: 
    *   10 procedurally-generated buildings with terrain-aware placement
    *   5m safety margin around structures
    *   Hard collision detection prevents passing through obstacles
*   **Dynamic Hazard Avoidance**: 
    *   3-6 moving obstacles (birds, unauthorized aircraft)
    *   3D repulsion forces adjusting drone velocity in real-time
*   **Passive Avoidance**: Drones autonomously steer away without explicit commands

### Navigation & Flight Modes
*   **Three Flight Modes**:
    1. **Manual**: Drones hover at their current location unless assigned a specific target
    2. **Max Altitude**: Drones climb to a standard cruise altitude (80m) during transit for safe clearance
    3. **Terrain Skimming**: Drones maintain a constant 5m Altitude Above Ground Level (AGL) using the terrain heightmap
*   **Formation Flying**: 
    *   Intelligent coordinate offsetting based on drone ID (3-column grid pattern)
    *   Prevents drones from converging on a single point when commanded as a group
    *   Maintains grid formation automatically during waypoint navigation
*   **Navigation Queue**: 
    *   Support for multi-waypoint missions
    *   Real-time waypoint editing (add, remove, clear)
    *   Return-to-Base (RTB) sequences with automatic landing
    *   Signal strength simulation (300m max range)

### Telemetry UI
*   **Real-time Data Grid**:
    *   7-column display: Drone ID, Position (X/Y/Z), Battery, Status, Proximity
    *   Stable multi-selection (Ctrl/Shift supported)
    *   Persistent selection during high-frequency updates
*   **Visual Alerts**: 
    *   Color-coded rows for status (⚠️ Proximity warnings, Red for crashes, Orange for low battery)
    *   Bold font highlighting for selected drones
    *   Visual arrow (→) indicator in ID column
*   **Dual-View System**:
    *   Deployed Drones view: Active aircraft with full telemetry
    *   Hangar Inventory view: Drones waiting to be deployed, showing charge status
*   **Dynamic Sorting**: Click column headers to sort by any metric

### Interactive Map
*   **Procedural Terrain Rendering**:
    *   Perlin noise-based heightmap generation
    *   Configurable land/water distribution (0-100%)
    *   Smooth terrain variation with hills and valleys
*   **Visual Elements**:
    *   Real-time drone position tracking
    *   Building/obstacle visualization
    *   Terrain heatmap rendering
*   **User Controls**:
    *   Rubber-band box selection for grouping drones
    *   Right-click waypoint placement for selected drones
    *   Waypoint removal via point clicking (white circle markers)

## Flight Modes

| Mode | Purpose | Altitude Control |
| :--- | :--- | :--- |
| **Manual** | Stationary hovering | User-specified or terrain-relative |
| **Max Altitude** | Safe transit | Fixed 80m cruise altitude |
| **Terrain Skimming** | Low-level flight | 5m AGL (above ground level) |

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
*   **Mutex-Protected Access**: All drone state queries use `std::mutex` locks
*   **Signal/Slot Communication**: Physics thread emits `simulationUpdated()` signal to trigger UI refreshes
*   **Snapshot Pattern**: Deep copies of drone data prevent race conditions and ensure consistency
*   **60 FPS UI Updates**: Efficient dataChanged signals preserve selection and scroll position

### Model/View Architecture
The project uses Qt's Model/View pattern with a custom `QAbstractTableModel` (`TelemetryModel`):
*   **Data Caching**: Local drone vector enables O(1) sorting and efficient filtering
*   **Optimized Updates**: Uses `dataChanged()` signals instead of full model resets (preserves selection)
*   **Dual View Support**: Same engine, two synchronized table views (Deployed vs. Hangar)
*   **Color-Coded Styling**: Background and font roles provide visual feedback

### Physics Loop (20Hz)
Each 50ms frame executes:
1. **Update Obstacles**: Move dynamic obstacles, handle boundary bouncing
2. **Hangar Logic**: Process return-to-base transitions and inventory management
3. **Per-Drone Updates**:
   *   Status transitions (Landing → Landed detection)
   *   Battery consumption based on activity
   *   Navigation queue processing and waypoint navigation
   *   Altitude mode adjustment (MaxAltitude/TerrainSkimming)
4. **Collision Detection**:
   *   Static obstacle avoidance (buildings)
   *   Dynamic obstacle avoidance (birds)
   *   Drone-to-drone repulsion fields
5. **Physics Application**: Update positions via velocity vectors
6. **Sensor Simulation**: GPS drift noise, signal strength decay with distance
7. **Signal Emission**: Notify UI of frame completion

## Terrain System

### Procedural Generation
*   **Perlin Noise**: Three octaves of fractal noise for natural terrain variation
*   **Seeded Generation**: Same seed = identical terrain for reproducibility
*   **Efficient Sampling**: O(1) height queries via noise interpolation
*   **Land/Water Distribution**: User-configurable ratio (landProp parameter)

### Terrain Features
*   **Heights**: Values >= 1.0 are traversable land; < 1.0 is water
*   **Static Obstacles**: 10 buildings with randomized positions, sizes, and heights
*   **Safety Zone**: Helipad area kept clear of obstacles
*   **Obstacle Separation**: Buildings maintain minimum distance to prevent overlaps

## Build Requirements
*   **C++17** Compatible Compiler (GCC 7+, Clang 5+, MSVC 2017+)
*   **Qt 6.x** Framework (5.15+ technically supported)
*   **CMake 3.10+** for build configuration
*   **OpenGL Support** (for future rendering enhancements)

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
*   Drone state initialization and transitions
*   Physics simulation accuracy (battery, altitude, velocity)
*   Collision detection and avoidance
*   Navigation queue processing
*   Multi-drone coordination and formation flying
*   Terrain sampling and obstacle generation
*   UI model updates and data formatting

## Simulation Parameters

### Drone Properties
*   **Radius**: 0.3m (collision detection)
*   **Max Speed**: 5 m/s
*   **Initial Battery**: 100%
*   **Min Safe Hover Battery**: ~9.33% (5m AGL)
*   **Signal Range**: 300m maximum

### Physics Constants
*   **Gravity**: 9.81 m/s²
*   **Frame Time**: 50ms (20Hz)
*   **Battery Hover Cost**: 0.05% per frame
*   **Battery Speed Cost**: 0.01% per m/s of velocity
*   **Landed Charge Rate**: 0.05% per frame
*   **Hangar Charge Rate**: 0.5% per frame

### World Constants
*   **Drone Safety Buffer**: 4m (symmetric repulsion)
*   **Static Obstacle Safety Margin**: 5m
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
│   │   ├── SimulationEngine.h/cpp  # Main physics loop
│   │   ├── TerrainMap.h/cpp  # Procedural terrain generation
│   ├── gui/
│   │   ├── MainWindow.h/cpp  # Main UI window
│   │   ├── TelemetryModel.h/cpp  # Table model for drone data
│   │   ├── TerrainView.h/cpp  # Map visualization
└── tests/
    └── TestAeroGrid.cpp      # Comprehensive test suite
```

## Future Enhancements

Potential areas for expansion:
*   **3D Visualization**: OpenGL-based 3D view with camera control
*   **Advanced Pathfinding**: A* or RRT* algorithms for waypoint optimization
*   **Swarm Behaviors**: Flocking and coordinated maneuvers
*   **Sensor Simulation**: LiDAR, camera, and thermal imaging simulation
*   **Scripting Support**: Lua or Python for mission automation
*   **Network Multiplayer**: Multi-client simulation scenarios
*   **Export/Replay**: Save and replay simulation runs for analysis

## Performance

*   **20Hz Physics**: Consistent frame rate with 12 drones deployed
*   **Dynamic Obstacle Count**: 3-6 moving obstacles per simulation
*   **Static Obstacle Count**: 10 buildings per terrain
*   **Memory Usage**: ~5-10 MB for full simulation state
*   **CPU Usage**: ~10-15% of single core at 20Hz (depends on build optimization)

---

**Last Updated**: May 2026  
**Current Version**: 1.0  
**Status**: Active Development