#pragma once
#include <QObject>
#include <QThread>
#include <vector>
#include <mutex>
#include <atomic>
#include "Drone.h"
#include "TerrainMap.h"

/**
 * @brief Background physics engine for the AeroGrid simulation.
 * 
 * Processes kinematics, collision avoidance, and navigation logic in a dedicated thread.
 */
class SimulationEngine : public QObject {
    Q_OBJECT
public:
    explicit SimulationEngine(QObject* parent = nullptr);
    void startSimulation(unsigned int seed, double landProp);
    void stopSimulation();
    
    // Thread-safe access to drone data for the UI
    /**
     * @brief Returns a deep copy of all current drone states.
     * 
     * Uses a internal mutex to ensure consistency while the worker thread
     * is actively calculating physics.
     */
    std::vector<Drone> getDroneData();
    std::vector<Drone> getInventoryData();
    int getInventoryCount();
    double getBaseX() const { return m_baseX; }
    double getBaseY() const { return m_baseY; }

    std::vector<DynamicObstacle> getDynamicObstacleData();
    const TerrainMap& getTerrain() const { return m_terrain; }
    
    void assignTarget(int id, double x, double y, NavigationMode mode);
    void clearNavQueue(int id);
    void removeNavPoint(int id, int index);
    void landDrone(int id);
    void returnToBase(int id);
    void takeOff(int id);
    void launchDrone();
    void launchDrones(const std::vector<int>& ids);

signals:
    void simulationUpdated(); // Signal triggered when a frame is calculated

private:
    void run(); // Main loop for the simulation thread
    void processHangarLogic();
    
    std::vector<Drone> m_drones;
    std::vector<Drone> m_baseInventory;
    std::vector<DynamicObstacle> m_dynamicObstacles;
    std::mutex m_mutex;       // Protects m_drones during thread access
    std::atomic<bool> m_running;
    double m_baseX;           // Randomized base X coordinate
    double m_baseY;           // Randomized base Y coordinate
    TerrainMap m_terrain;
    QThread* m_workerThread = nullptr;
};