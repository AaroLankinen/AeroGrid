#pragma once
#include <QObject>
#include <QThread>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <map>
#include "Drone.h"
#include "TerrainMap.h"

/**
 * @brief Background physics engine for the AeroGrid simulation.
 * 
 * The SimulationEngine is the core of AeroGrid, managing:
 * - Real-time physics simulation at 20Hz (50ms per frame)
 * - Multi-drone kinematics, collision detection, and avoidance
 * - Battery consumption and charging simulation driven by AeroGrid::Physics constants
 * - Terrain interaction and obstacle avoidance
 * - Navigation queue processing and formation flying
 * 
 * The engine runs in a dedicated QThread to prevent UI blocking. All drone
 * state is protected by a mutex to ensure thread-safe data exchange.
 * 
 * @section threading Threading Model
 * The simulation loop runs in a background worker thread. UI components access
 * drone data through thread-safe getter methods that use mutex locks. The
 * simulationUpdated() signal is emitted after each physics frame to notify
 * the UI of new data.
 * 
 * @section collision Collision System
 * - Passive Avoidance: Drones in proximity generate repulsion forces
 * - Static Obstacles: Buildings placed by TerrainMap; spacing uses Physics::STATIC_OBS_SAFETY_MARGIN
 * - Dynamic Obstacles: Moving entities like birds or unauthorized aircraft
 * - Drone-to-Drone: Symmetrical repulsion maintaining Physics::DRONE_TO_DRONE_SAFETY_BUFFER
 * 
 * @section battery Battery Management
 * - Base Charge Rate: Landed drones gain Physics::LANDED_CHARGE_RATE per tick
 * - Hangar Charge Rate: Accelerated Physics::HANGAR_CHARGE_RATE for fast cycling
 * - Flight Consumption: Physics::HOVER_CONSUMPTION plus a factor based on speed
 * - Emergency Landing: Triggered when battery drops below safe landing threshold
 * 
 * @section formation Formation Flying
 * When multiple drones navigate to the same waypoint, they apply unique X/Y offsets
 * based on their ID (a 3-column grid pattern) to prevent convergence and collisions.
 */
class SimulationEngine : public QObject {
    Q_OBJECT
    friend class TestAeroGrid;
public:
    /**
     * @brief Constructs a SimulationEngine instance.
     * 
     * @param parent QObject parent for memory management (optional).
     */
    explicit SimulationEngine(QObject* parent = nullptr);
    
    /**
     * @brief Destructor that ensures the simulation thread is properly stopped.
     */
    ~SimulationEngine() override;
    
    /**
     * @brief Initializes and starts the physics simulation.
     * 
     * Sets up the terrain, base location, hangar inventory (12 drones), and
     * dynamic obstacles. Spawns the background physics thread. This method
     * should be called before any other simulation methods.
     * 
     * @param seed Random seed for terrain and obstacle generation. Determines
     *             the entire world layout for reproducibility.
     * @param landProp Land proportion (0.0 - 1.0). Controls the ratio of
     *                 land vs. water in the procedural terrain map.
     *                 0.5 = 50% land, 50% water.
     * @param width Terrain grid width in cells (default 200).
     * @param height Terrain grid height in cells (default 200).
     * 
     * @note Calling this method while a simulation is running will stop the
     *       current one first.
     */
    void startSimulation(unsigned int seed, double landProp, int width = 200, int height = 200, int numStaticObstacles = 10, int numDynamicObstacles = -1, bool enableSignalLoss = false);
    
    /**
     * @brief Stops the physics simulation and cleans up the worker thread.
     * 
     * Waits for the physics thread to finish cleanly. Safe to call multiple times.
     */
    void stopSimulation();
    
    /**
     * @brief Returns a deep copy of all currently deployed drone states.
     * 
     * This method is thread-safe and uses a mutex to protect against concurrent
     * access from the physics thread. Use this to get current drone positions,
     * battery levels, and other real-time telemetry.
     * 
     * @return A vector of all Drone objects currently in flight or landed.
     */
    std::vector<Drone> getDroneData();

    /**
     * @brief Retrieves a pointer to a single drone by its ID.
     *
     * This method is thread-safe and uses a mutex to protect against concurrent
     * access from the physics thread.
     * @return A const pointer to the Drone object, or nullptr if not found.
     */
    const Drone* getDroneById(int id) const;
    
    /**
     * @brief Returns a deep copy of all drones currently in the hangar.
     * 
     * The hangar contains drones waiting to be deployed. Hangar drones charge
     * at an accelerated rate (0.5% per tick).
     * 
     * @return A vector of all Drone objects in inventory.
     */
    std::vector<Drone> getInventoryData();
    
    /**
     * @brief Returns the count of drones currently in the hangar.
     * 
     * @return The number of available drones in the hangar inventory.
     */
    int getInventoryCount();
    
    /**
     * @brief Returns the X coordinate of the base/helipad location.
     * 
     * The base location is randomly placed on land (height >= 1.0) during
     * simulation initialization. Drones return here when RTB is commanded.
     * 
     * @return The X world coordinate of the base in meters.
     */
    double getBaseX() const { return m_baseX; }
    
    /**
     * @brief Returns the Y coordinate of the base/helipad location.
     * 
     * @return The Y world coordinate of the base in meters.
     * @see getBaseX()
     */
    double getBaseY() const { return m_baseY; }

    /**
     * @brief Returns a deep copy of all dynamic obstacles (birds, other aircraft).
     * 
     * Dynamic obstacles move through the airspace and drones must avoid them.
     * They bounce off world boundaries and altitude limits.
     * 
     * @return A vector of all DynamicObstacle objects in the simulation.
     */
    std::vector<DynamicObstacle> getDynamicObstacleData();
    
    /**
     * @brief Returns a const reference to the terrain map.
     * 
     * The terrain defines altitude, static obstacles (buildings), and ground
     * features. Use this to query height at specific coordinates.
     * 
     * @return A const reference to the TerrainMap object.
     */
    const TerrainMap& getTerrain() const { return m_terrain; }
    
    /**
     * @brief Assigns a navigation target waypoint to a drone.
     * 
     * Adds a waypoint to the drone's navigation queue. The drone will navigate
     * toward this waypoint following the specified altitude mode. Multiple calls
     * build a queue of waypoints that are processed in order.
     * 
     * @param id The ID of the drone to command.
     * @param x Target X coordinate in meters.
     * @param y Target Y coordinate in meters.
     * @param mode How altitude should be managed (Manual, MaxAltitude, or TerrainSkimming).
     * 
     * @note The Z coordinate from the NavPoint is calculated based on mode
     *       during flight, not stored as-is.
     * @note Assigning a target clears the RTB flag, replacing RTB with manual navigation.
     */
    void assignTarget(int id, double x, double y, NavigationMode mode);
    
    /**
     * @brief Clears all navigation waypoints for a drone and stops movement.
     * 
     * Empties the drone's navigation queue and sets all velocity components to zero.
     * The drone will hover at its current position.
     * 
     * @param id The ID of the drone to command.
     */
    void clearNavQueue(int id);
    
    /**
     * @brief Removes a specific waypoint from a drone's navigation queue.
     * 
     * Useful for editing a multi-waypoint mission mid-flight.
     * 
     * @param id The ID of the drone.
     * @param index Zero-based index of the waypoint to remove. If out of range,
     *              this method has no effect.
     */
    void removeNavPoint(int id, int index);
    
    /**
     * @brief Commands a drone to perform a controlled landing.
     * 
     * Changes the drone's status to Landing and clears the navigation queue.
     * The physics loop will execute the landing sequence automatically.
     * 
     * @param id The ID of the drone to command. Must be in Flying state.
     * 
     * @note If battery drops below safe landing threshold, emergency landing
     *       is triggered automatically instead.
     */
    void landDrone(int id);
    
    /**
     * @brief Commands a drone to return to the base/helipad.
     * 
     * Sets the drone's navigation queue to the base location and marks it as
     * returning to base. The drone will navigate to base and then land automatically.
     * 
     * @param id The ID of the drone to command. Must be in Flying state.
     * 
     * @note This clears any existing navigation queue and replaces it with
     *       a single waypoint at the base.
     */
    void returnToBase(int id);
    
    /**
     * @brief Commands a drone to take off from the ground.
     * 
     * Changes the drone's status from Landed to Flying and initiates upward movement
     * (vz = 2.0 m/s). Battery level must be sufficient to safely hover and land.
     * 
     * @param id The ID of the drone to command. Must be in Landed state.
     * 
     * @note If battery is insufficient, the command is ignored.
     */
    void takeOff(int id);
    
    /**
     * @brief Deploys a single drone from the hangar to the flight area.
     * 
     * Removes the last drone from the inventory and places it at a landing pad
     * offset based on its ID (forming a 3-column grid to avoid collisions).
     * The drone starts in the Landed state.
     * 
     * @note If the hangar is empty, this method has no effect.
     */
    void launchDrone();
    
    /**
     * @brief Deploys multiple specific drones from the hangar by ID.
     * 
     * Removes each specified drone from the inventory (if present) and places
     * it at its individual landing pad. Non-existent IDs are silently ignored.
     * 
     * @param ids A vector of drone IDs to deploy.
     * 
     * @see launchDrone()
     */
    void launchDrones(const std::vector<int>& ids);

signals:
    /**
     * @brief Emitted when a new physics frame has been calculated and drone data is ready.
     * 
     * Connect UI update slots to this signal to refresh telemetry displays after
     * each simulation cycle (approximately every 50ms at 20Hz).
     */
    void simulationUpdated();

private:
    /**
     * @brief Main physics loop running in the background worker thread.
     * 
     * Executes at ~20Hz (50ms per frame) and handles:
     * - Kinematic updates for all drones and dynamic obstacles
     * - Collision detection (drone-to-drone, drone-to-static-obstacle, drone-to-dynamic-obstacle)
     * - Obstacle avoidance repulsion field calculations
     * - Battery consumption and landing automation
     * - Navigation queue processing
     * 
     * @note This method is private and called only by the worker thread.
     */
    void run();
    
    /**
     * @brief Handles transitions between deployed drones and hangar inventory.
     * 
     * Checks if any drone has completed Return-to-Base and landed. If so,
     * moves it back to the inventory (hangar) in sorted ID order.
     * 
     * @note Called once per physics frame from run().
     */
    void processHangarLogic();
    
    /**
     * @brief Updates positions of dynamic obstacles.
     * @param dt Time step in seconds.
     */
    void updateDynamicObstacles(double dt);

    /**
     * @brief Handles charging logic for drones currently in the hangar.
     */
    void updateHangarDrones();

    /**
     * @brief Evaluates drone-to-drone collisions and applies repulsion forces.
     */
    void handleDroneToDroneCollisions();

    /**
     * @brief Main state machine and physics update for a single drone.
     * @param drone Reference to the drone to update.
     * @param dt Time step in seconds.
     */
    void updateDroneState(Drone& drone, double dt);

    /**
     * @brief Calculates intended velocity based on flight status and mission queue.
     * @param drone Reference to the drone.
     * @param groundHeight Terrain altitude at drone's current position.
     * @param dt Time step in seconds.
     */
    void calculateDroneMovement(Drone& drone, double groundHeight, double dt);

    /**
     * @brief Calculates repulsion from static and dynamic obstacles.
     * @param drone Reference to the drone.
     * @param groundHeight Terrain altitude at drone's current position.
     */
    void applyObstacleAvoidance(Drone& drone, double groundHeight);

    /**
     * @brief Checks if the drone has touched the ground and determines if it landed or crashed.
     * @param drone Reference to the drone.
     * @param groundHeight Terrain altitude at drone's current position.
     */
    void checkGroundContact(Drone& drone, double groundHeight);

    /**
     * @brief Updates signal strength based on distance to base station.
     * @param drone Reference to the drone.
     */
    void updateDroneSignalStrength(Drone& drone);

    /**
     * @brief Gathers physical world data to compile a sensory package for the drone.
     */
    DroneSensors populateSensors(const Drone& physicalDrone) const;

    // Simulation State
    std::map<int, Drone> m_drones;              ///< Perceived telemetry drones database.
    std::map<int, Drone> m_physicalDrones;      ///< True physical states of active drones.
    std::map<int, DroneController> m_controllers; ///< Onboard autopilot controllers.
    std::vector<Drone> m_baseInventory;         ///< Drones in the hangar.
    std::vector<DynamicObstacle> m_dynamicObstacles;  ///< Moving obstacles (birds, etc.)
    mutable std::shared_mutex m_mutex;          ///< Protects m_drones with Read-Write access.
    std::atomic<bool> m_running;                ///< Flag to signal the physics thread to stop.
    bool m_enableSignalLoss = false;            ///< True to simulate signal path loss and blockage.
    
    // Base Station & Terrain
    double m_baseX;                             ///< X coordinate of helipad (randomized).
    double m_baseY;                             ///< Y coordinate of helipad (randomized).
    TerrainMap m_terrain;                       ///< Procedurally generated world.
    
    // Threading
    QThread* m_workerThread;                    ///< Background physics thread.
};