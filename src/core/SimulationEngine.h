#pragma once
#include <QObject>
#include <QThread>
#include <vector>
#include <mutex>
#include <atomic>
#include "Drone.h"
#include "TerrainMap.h"

class SimulationEngine : public QObject {
    Q_OBJECT
public:
    explicit SimulationEngine(QObject* parent = nullptr);
    void startSimulation();
    void stopSimulation();
    
    // Thread-safe access to drone data for the UI
    std::vector<Drone> getDroneData();
    std::vector<DynamicObstacle> getDynamicObstacleData();
    const TerrainMap& getTerrain() const { return m_terrain; }
    
    void assignTarget(int id, double x, double y, NavigationMode mode);
    void clearNavQueue(int id);
    void removeNavPoint(int id, int index);
    void landDrone(int id);
    void returnToBase(int id);
    void takeOff(int id);

signals:
    void simulationUpdated(); // Signal triggered when a frame is calculated

private:
    void run(); // Main loop for the simulation thread
    
    std::vector<Drone> m_drones;
    std::vector<DynamicObstacle> m_dynamicObstacles;
    std::mutex m_mutex;       // Protects m_drones during thread access
    std::atomic<bool> m_running;
    TerrainMap m_terrain;
    QThread* m_workerThread = nullptr;
};