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

signals:
    void simulationUpdated(); // Signal triggered when a frame is calculated

private:
    void run(); // Main loop for the simulation thread
    
    std::vector<Drone> m_drones;
    std::mutex m_mutex;       // Protects m_drones during thread access
    std::atomic<bool> m_running;
    TerrainMap m_terrain;
    QThread* m_workerThread = nullptr;
};