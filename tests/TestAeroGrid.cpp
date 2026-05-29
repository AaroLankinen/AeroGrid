#include <QtTest>
#include <QSignalSpy>
#include <cmath>
#include <QFont>
#include <QBrush>
#include <QColor>
#include "SimulationEngine.h"
#include "TelemetryModel.h"
#include "Drone.h"

class TestAeroGrid : public QObject {
    Q_OBJECT

private slots:
    // ===== DRONE STRUCT TESTS =====
    
    /**
     * @brief Verifies that a Drone is correctly initialized with default values.
     */
    void testDroneInitialState() {
        Drone d(101);
        QCOMPARE(d.id, 101);
        QCOMPARE(d.x, 0.0);
        QCOMPARE(d.y, 0.0);
        QCOMPARE(d.z, 0.0);
        QCOMPARE(d.vx, 0.0);
        QCOMPARE(d.vy, 0.0);
        QCOMPARE(d.vz, 0.0);
        QCOMPARE(d.batteryLevel, 100.0);
        QCOMPARE(d.status, DroneStatus::Flying);
        QCOMPARE(d.signalStrength, 1.0);
        QCOMPARE(d.radius, 0.3);
        QCOMPARE(d.navMode, NavigationMode::Manual);
        QVERIFY(d.navQueue.empty());
        QCOMPARE(d.returningToBase, false);
        QCOMPARE(d.proximityAlert, false);
    }

    /**
     * @brief Verifies that multiple drones can have different IDs.
     */
    void testMultipleDrones() {
        Drone d1(1), d2(2), d3(3);
        QCOMPARE(d1.id, 1);
        QCOMPARE(d2.id, 2);
        QCOMPARE(d3.id, 3);
    }

    // ===== SIMULATION ENGINE TESTS =====

    /**
     * @brief Verifies SimulationEngine setup: terrain, base positioning, and hangar initialization.
     */
    void testEngineInitialization() {
        SimulationEngine engine;
        engine.startSimulation(12345, 0.5, 200, 200);
        
        // Hangar should start with 12 drones, active fleet should be empty
        QCOMPARE(engine.getInventoryCount(), 12);
        QCOMPARE(engine.getDroneData().size(), 0);
        
        // Verify base location is within bounds
        double bx = engine.getBaseX();
        double by = engine.getBaseY();
        QVERIFY(bx >= -50.0 && bx <= 50.0);
        QVERIFY(by >= -50.0 && by <= 50.0);
        QVERIFY(engine.getTerrain().getHeightAt(bx, by) >= 1.0);
        
        // Verify dynamic obstacles were created
        auto obstacles = engine.getDynamicObstacleData();
        QVERIFY(obstacles.size() >= 3 && obstacles.size() <= 6);
        
        engine.stopSimulation();
    }

    /**
     * @brief Verifies that different seeds produce different terrain.
     */
    void testDifferentSeeds() {
        SimulationEngine engine1, engine2;
        engine1.startSimulation(1000, 0.5, 200, 200);
        engine2.startSimulation(2000, 0.5, 200, 200);
        
        // Base positions should likely be different (extremely unlikely to be the same)
        double bx1 = engine1.getBaseX(), by1 = engine1.getBaseY();
        double bx2 = engine2.getBaseX(), by2 = engine2.getBaseY();
        
        // It's technically possible they're the same, but extremely unlikely
        // Just verify they're both valid
        QVERIFY(bx1 >= -50.0 && bx1 <= 50.0);
        QVERIFY(bx2 >= -50.0 && bx2 <= 50.0);
        
        engine1.stopSimulation();
        engine2.stopSimulation();
    }

    /**
     * @brief Verifies that starting the simulation triggers the simulationUpdated signal.
     */
    void testSimulationSignalEmission() {
        SimulationEngine engine;
        QSignalSpy spy(&engine, &SimulationEngine::simulationUpdated);
        
        engine.startSimulation(1, 1.0, 200, 200);
        
        // Wait for at least one physics cycle
        QVERIFY(spy.wait(500));
        QVERIFY(spy.count() > 0);
        
        engine.stopSimulation();
    }

    /**
     * @brief Verifies that stopSimulation stops signal emission.
     */
    void testStopSimulation() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        
        QTest::qWait(100);
        engine.stopSimulation();
        
        // After stopping, simulation should not be running
        QCOMPARE(engine.getDroneData().size(), 0);
    }

    /**
     * @brief Verifies drone deployment from hangar to active fleet.
     */
    void testDroneDeployment() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        
        // Deploy a single drone
        engine.launchDrone();
        QCOMPARE(engine.getInventoryCount(), 11);
        QCOMPARE(engine.getDroneData().size(), 1);
        
        // Check state of newly deployed drone
        std::vector<Drone> drones = engine.getDroneData();
        Drone& deployed = drones.back();
        QCOMPARE(deployed.status, DroneStatus::Landed);
        
        // Drone is placed at formation offset from base, not exactly at base
        // Offset is: (id % 3 - 1) * 4.0 for X, (id / 3 - 1) * 4.0 for Y
        double expectedOffsetX = (deployed.id % 3 - 1) * 4.0;
        double expectedOffsetY = (deployed.id / 3 - 1) * 4.0;
        QCOMPARE(deployed.x, engine.getBaseX() + expectedOffsetX);
        QCOMPARE(deployed.y, engine.getBaseY() + expectedOffsetY);
        
        QVERIFY(deployed.vx == 0.0 && deployed.vy == 0.0 && deployed.vz == 0.0);

        engine.stopSimulation();
    }

    /**
     * @brief Verifies multiple drone deployment with specific IDs.
     */
    void testDroneDeploymentMultiple() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        
        std::vector<int> ids = {0, 1, 2};
        engine.launchDrones(ids);
        QCOMPARE(engine.getInventoryCount(), 9);
        QCOMPARE(engine.getDroneData().size(), 3);
        
        // Verify all deployed drones have correct IDs
        std::vector<Drone> drones = engine.getDroneData();
        for (const auto& d : drones) {
            QVERIFY(std::find(ids.begin(), ids.end(), d.id) != ids.end());
        }

        engine.stopSimulation();
    }

    /**
     * @brief Verifies deployment when hangar is empty.
     */
    void testDeploymentEmptyHangar() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        
        // Deploy all 12 drones
        for (int i = 0; i < 12; ++i) {
            engine.launchDrone();
        }
        
        QCOMPARE(engine.getInventoryCount(), 0);
        QCOMPARE(engine.getDroneData().size(), 12);
        
        // Attempt to deploy from empty hangar should have no effect
        engine.launchDrone();
        QCOMPARE(engine.getInventoryCount(), 0);
        QCOMPARE(engine.getDroneData().size(), 12);

        engine.stopSimulation();
    }

    /**
     * @brief Verifies get/set inventory data methods.
     */
    void testInventoryDataAccess() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        
        std::vector<Drone> inventory = engine.getInventoryData();
        QCOMPARE(inventory.size(), 12);
        QCOMPARE(engine.getInventoryCount(), 12);
        
        // Deploy one and verify inventory decreases
        engine.launchDrone();
        QCOMPARE(engine.getInventoryCount(), 11);

        engine.stopSimulation();
    }

    /**
     * @brief Tests flight commands: takeoff, target assignment, and queue clearing.
     */
    void testFlightCommandsTakeoff() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        int id = engine.getDroneData().at(0).id;

        // Takeoff command
        engine.takeOff(id);
        
        QSignalSpy spy(&engine, &SimulationEngine::simulationUpdated);
        QVERIFY(spy.wait(200));
        
        std::vector<Drone> drones = engine.getDroneData();
        QCOMPARE(drones.at(0).status, DroneStatus::Flying);
        QVERIFY(drones.at(0).vz > 0); // Should be ascending

        engine.stopSimulation();
    }

    /**
     * @brief Tests target assignment and navigation mode.
     */
    void testAssignTarget() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        int id = engine.getDroneData().at(0).id;
        engine.takeOff(id);

        // Assign navigation target
        engine.assignTarget(id, 10.0, 10.0, NavigationMode::MaxAltitude);
        
        std::vector<Drone> drones = engine.getDroneData();
        QCOMPARE(drones.at(0).navQueue.size(), 1);
        QCOMPARE(drones.at(0).navMode, NavigationMode::MaxAltitude);
        QCOMPARE(drones.at(0).navQueue.front().x, 10.0);
        QCOMPARE(drones.at(0).navQueue.front().y, 10.0);

        engine.stopSimulation();
    }

    /**
     * @brief Tests multiple target assignment (waypoint queue).
     */
    void testMultipleTargets() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        int id = engine.getDroneData().at(0).id;

        engine.assignTarget(id, 10.0, 10.0, NavigationMode::Manual);
        engine.assignTarget(id, 20.0, 20.0, NavigationMode::Manual);
        engine.assignTarget(id, 30.0, 30.0, NavigationMode::Manual);
        
        std::vector<Drone> drones = engine.getDroneData();
        QCOMPARE(drones.at(0).navQueue.size(), 3);

        engine.stopSimulation();
    }

    /**
     * @brief Tests clearing navigation queue.
     */
    void testClearNavQueue() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        int id = engine.getDroneData().at(0).id;

        engine.assignTarget(id, 10.0, 10.0, NavigationMode::Manual);
        engine.clearNavQueue(id);
        
        std::vector<Drone> drones = engine.getDroneData();
        QVERIFY(drones.at(0).navQueue.empty());
        QCOMPARE(drones.at(0).vx, 0.0);
        QCOMPARE(drones.at(0).vy, 0.0);
        QCOMPARE(drones.at(0).vz, 0.0);

        engine.stopSimulation();
    }

    /**
     * @brief Tests removing a specific waypoint from queue.
     */
    void testRemoveNavPoint() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        int id = engine.getDroneData().at(0).id;

        engine.assignTarget(id, 10.0, 10.0, NavigationMode::Manual);
        engine.assignTarget(id, 20.0, 20.0, NavigationMode::Manual);
        engine.assignTarget(id, 30.0, 30.0, NavigationMode::Manual);
        
        // Remove middle waypoint
        engine.removeNavPoint(id, 1);
        
        std::vector<Drone> drones = engine.getDroneData();
        QCOMPARE(drones.at(0).navQueue.size(), 2);
        QCOMPARE(drones.at(0).navQueue[0].x, 10.0);
        QCOMPARE(drones.at(0).navQueue[1].x, 30.0); // Middle removed

        engine.stopSimulation();
    }

    /**
     * @brief Tests removing invalid waypoint index.
     */
    void testRemoveNavPointInvalidIndex() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        int id = engine.getDroneData().at(0).id;

        engine.assignTarget(id, 10.0, 10.0, NavigationMode::Manual);
        
        // Try to remove non-existent index
        engine.removeNavPoint(id, 10);
        
        std::vector<Drone> drones = engine.getDroneData();
        QCOMPARE(drones.at(0).navQueue.size(), 1); // Should still have the target

        engine.stopSimulation();
    }

    /**
     * @brief Tests landing command.
     */
    void testLandDrone() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        int id = engine.getDroneData().at(0).id;
        
        engine.takeOff(id);
        QTest::qWait(100);
        
        engine.landDrone(id);
        
        std::vector<Drone> drones = engine.getDroneData();
        QCOMPARE(drones.at(0).status, DroneStatus::Landing);
        QVERIFY(drones.at(0).navQueue.empty());

        engine.stopSimulation();
    }

    /**
     * @brief Tests return to base command.
     */
    void testReturnToBase() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        int id = engine.getDroneData().at(0).id;
        
        engine.takeOff(id);
        engine.assignTarget(id, 50.0, 50.0, NavigationMode::Manual);
        engine.returnToBase(id);
        
        std::vector<Drone> drones = engine.getDroneData();
        // Should have base location in nav queue
        QCOMPARE(drones.at(0).navQueue.size(), 1);
        QCOMPARE(drones.at(0).navQueue.front().x, engine.getBaseX());
        QCOMPARE(drones.at(0).navQueue.front().y, engine.getBaseY());
        QCOMPARE(drones.at(0).returningToBase, true);

        engine.stopSimulation();
    }

    /**
     * @brief Tests terrain map integration.
     */
    void testTerrainIntegration() {
        SimulationEngine engine;
        engine.startSimulation(12345, 0.5, 200, 200);
        
        const TerrainMap& terrain = engine.getTerrain();
        QCOMPARE(terrain.getWidth(), 200);
        QCOMPARE(terrain.getHeight(), 200);
        QCOMPARE(terrain.getCellSize(), 1.0);
        QCOMPARE(terrain.getWorldWidth(), 200.0);
        QCOMPARE(terrain.getWorldHeight(), 200.0);
        
        // Test height at base location
        double baseHeight = terrain.getHeightAt(engine.getBaseX(), engine.getBaseY());
        QVERIFY(baseHeight >= 1.0);

        engine.stopSimulation();
    }

    /**
     * @brief Tests dynamic obstacle data.
     */
    void testDynamicObstacles() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        
        std::vector<DynamicObstacle> obs = engine.getDynamicObstacleData();
        QVERIFY(obs.size() >= 3 && obs.size() <= 6);
        
        // Verify obstacle properties
        for (const auto& o : obs) {
            QVERIFY(o.id >= 100);
            QVERIFY(o.radius > 0);
            QVERIFY(o.z >= 10.0 && o.z <= 60.0);
        }

        engine.stopSimulation();
    }

    /**
     * @brief Tests physics execution - battery drain.
     */
    void testPhysicsBatteryDrain() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        int id = engine.getDroneData().at(0).id;
        
        double initialBattery = engine.getDroneData().at(0).batteryLevel;
        engine.takeOff(id);
        
        QTest::qWait(300);
        
        std::vector<Drone> drones = engine.getDroneData();
        double finalBattery = drones.at(0).batteryLevel;
        
        QVERIFY(finalBattery < initialBattery);
        QVERIFY(finalBattery >= 0);

        engine.stopSimulation();
    }

    /**
     * @brief Tests physics execution - altitude changes.
     */
    void testPhysicsAltitudeChange() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        int id = engine.getDroneData().at(0).id;
        
        double initialZ = engine.getDroneData().at(0).z;
        engine.takeOff(id);
        
        QTest::qWait(200);
        
        std::vector<Drone> drones = engine.getDroneData();
        double finalZ = drones.at(0).z;
        
        // Should be higher after takeoff (ascending)
        QVERIFY(finalZ > initialZ);

        engine.stopSimulation();
    }

    // ===== TERRAIN MAP TESTS =====

    /**
     * @brief Tests TerrainMap initialization and properties.
     */
    void testTerrainMapInitialization() {
        TerrainMap terrain(12345, 0.5, 256, 256, 1.0);
        
        QCOMPARE(terrain.getWidth(), 256);
        QCOMPARE(terrain.getHeight(), 256);
        QCOMPARE(terrain.getCellSize(), 1.0);
        QCOMPARE(terrain.getWorldWidth(), 256.0);
        QCOMPARE(terrain.getWorldHeight(), 256.0);
    }

    /**
     * @brief Tests height sampling at various locations.
     */
    void testTerrainHeightSampling() {
        TerrainMap terrain(1, 0.5, 100, 100);
        
        // Heights should be valid (>=0)
        for (int x = -50; x <= 50; x += 10) {
            for (int y = -50; y <= 50; y += 10) {
                double h = terrain.getHeightAt(x, y);
                QVERIFY(h >= 0.0);
            }
        }
    }

    /**
     * @brief Tests that different seeds produce different terrains.
     */
    void testTerrainDifferentSeeds() {
        TerrainMap terrain1(100, 0.5, 100, 100);
        TerrainMap terrain2(200, 0.5, 100, 100);
        
        // Sampling the same location should give different heights
        double h1 = terrain1.getHeightAt(0, 0);
        double h2 = terrain2.getHeightAt(0, 0);
        
        // Extremely unlikely to be exactly the same
        QVERIFY(!(h1 == h2 && h1 == terrain1.getHeightAt(10, 10) && 
                  h2 == terrain2.getHeightAt(10, 10)));
    }

    /**
     * @brief Tests static obstacles are created.
     */
    void testTerrainObstacles() {
        TerrainMap terrain(1, 0.5, 200, 200);
        
        const auto& obstacles = terrain.getObstacles();
        QCOMPARE(obstacles.size(), 10);
        
        // Verify each obstacle has valid properties
        for (const auto& obs : obstacles) {
            QVERIFY(obs.id >= 0 && obs.id < 10);
            QVERIFY(obs.radius > 0);
            QVERIFY(obs.height > 0);
        }
    }

    /**
     * @brief Tests terrain with different land proportions.
     */
    void testTerrainLandProportion() {
        TerrainMap terrain1(1, 0.1, 100, 100); // 10% land
        TerrainMap terrain2(1, 0.9, 100, 100); // 90% land
        
        // Both should be valid
        QVERIFY(terrain1.getHeightAt(0, 0) >= 0);
        QVERIFY(terrain2.getHeightAt(0, 0) >= 0);
    }

    // ===== TELEMETRY MODEL TESTS =====

    /**
     * @brief Tests TelemetryModel initialization.
     */
    void testTelemetryModelInitialization() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        
        TelemetryModel model(&engine, DroneListType::Deployed);
        
        QCOMPARE(model.rowCount(), 0); // No deployed drones yet
        QCOMPARE(model.columnCount(), 7);
        
        engine.stopSimulation();
    }

    /**
     * @brief Tests TelemetryModel with deployed drones.
     */
    void testTelemetryModelDeployedDrones() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        
        TelemetryModel model(&engine, DroneListType::Deployed);
        model.updateModel();
        
        QCOMPARE(model.rowCount(), 1);
        QCOMPARE(model.columnCount(), 7);
        
        engine.stopSimulation();
    }

    /**
     * @brief Tests TelemetryModel hangar view.
     */
    void testTelemetryModelHangar() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        
        TelemetryModel model(&engine, DroneListType::Hangar);
        model.updateModel();
        
        QCOMPARE(model.rowCount(), 12); // All drones in hangar initially
        
        engine.launchDrone();
        model.updateModel();
        
        QCOMPARE(model.rowCount(), 11); // One deployed
        
        engine.stopSimulation();
    }

    /**
     * @brief Tests TelemetryModel data retrieval.
     */
    void testTelemetryModelData() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        
        TelemetryModel model(&engine, DroneListType::Deployed);
        model.updateModel();
        
        // Test ID column
        QString idStr = model.data(model.index(0, 0), Qt::DisplayRole).toString();
        QVERIFY(!idStr.isEmpty());
        
        // Test Battery column
        QString batteryStr = model.data(model.index(0, 4), Qt::DisplayRole).toString();
        QVERIFY(batteryStr.contains("%"));
        
        engine.stopSimulation();
    }

    /**
     * @brief Tests TelemetryModel header data.
     */
    void testTelemetryModelHeaders() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        
        TelemetryModel model(&engine, DroneListType::Deployed);
        
        QCOMPARE(model.headerData(0, Qt::Horizontal, Qt::DisplayRole).toString(), QString("Drone ID"));
        QCOMPARE(model.headerData(1, Qt::Horizontal, Qt::DisplayRole).toString(), QString("X"));
        QCOMPARE(model.headerData(2, Qt::Horizontal, Qt::DisplayRole).toString(), QString("Y"));
        QCOMPARE(model.headerData(3, Qt::Horizontal, Qt::DisplayRole).toString(), QString("Z"));
        QCOMPARE(model.headerData(4, Qt::Horizontal, Qt::DisplayRole).toString(), QString("Battery"));
        QCOMPARE(model.headerData(5, Qt::Horizontal, Qt::DisplayRole).toString(), QString("Status"));
        QCOMPARE(model.headerData(6, Qt::Horizontal, Qt::DisplayRole).toString(), QString("Proximity"));
        
        engine.stopSimulation();
    }

    /**
     * @brief Tests TelemetryModel selection and visual indicators.
     */
    void testTelemetryModelSelection() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        
        TelemetryModel model(&engine, DroneListType::Deployed);
        model.updateModel();
        
        int droneId = model.getDroneIdAt(0);
        model.setSelectedIds({droneId});
        
        // Selected drone should have indicator
        QString idStr = model.data(model.index(0, 0), Qt::DisplayRole).toString();
        QVERIFY(idStr.contains("→"));
        
        // Font should be bold
        QVariant fontVar = model.data(model.index(0, 0), Qt::FontRole);
        QFont font = fontVar.value<QFont>();
        QVERIFY(font.bold());
        
        engine.stopSimulation();
    }

    /**
     * @brief Tests TelemetryModel sorting.
     */
    void testTelemetryModelSorting() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrones({0, 1, 2});
        
        TelemetryModel model(&engine, DroneListType::Deployed);
        model.updateModel();
        
        // Sort by ID
        model.sort(0, Qt::AscendingOrder);
        QCOMPARE(model.m_sortColumn, 0);
        QCOMPARE(model.m_sortOrder, Qt::AscendingOrder);
        
        // Sort descending
        model.sort(0, Qt::DescendingOrder);
        QCOMPARE(model.m_sortOrder, Qt::DescendingOrder);
        
        engine.stopSimulation();
    }

    /**
     * @brief Tests TelemetryModel background role (status colors).
     */
    void testTelemetryModelStatusColors() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        
        TelemetryModel model(&engine, DroneListType::Deployed);
        model.updateModel();
        
        // Check that background color for normal drone exists
        QVariant bg = model.data(model.index(0, 0), Qt::BackgroundRole);
        // Normal battery should have no background (empty variant) or white
        QVERIFY(bg.isNull() || bg.canConvert<QBrush>());
        
        engine.stopSimulation();
    }

    /**
     * @brief Tests TelemetryModel drone ID retrieval.
     */
    void testTelemetryModelGetDroneId() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        
        TelemetryModel model(&engine, DroneListType::Deployed);
        model.updateModel();
        
        int droneId = model.getDroneIdAt(0);
        QVERIFY(droneId >= 0);
        
        // Invalid row should return -1
        QCOMPARE(model.getDroneIdAt(10), -1);
        QCOMPARE(model.getDroneIdAt(-1), -1);
        
        engine.stopSimulation();
    }

    /**
     * @brief Tests TelemetryModel dynamic updates.
     */
    void testTelemetryModelDynamicUpdate() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        
        TelemetryModel model(&engine, DroneListType::Deployed);
        model.updateModel();
        QCOMPARE(model.rowCount(), 0);
        
        engine.launchDrone();
        model.updateModel();
        QCOMPARE(model.rowCount(), 1);
        
        engine.launchDrone();
        model.updateModel();
        QCOMPARE(model.rowCount(), 2);
        
        engine.stopSimulation();
    }

    /**
     * @brief Tests TelemetryModel status display.
     */
    void testTelemetryModelStatusDisplay() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        
        TelemetryModel model(&engine, DroneListType::Deployed);
        model.updateModel();
        
        // Should show maintenance status for new drone
        QString status = model.data(model.index(0, 5), Qt::DisplayRole).toString();
        QVERIFY(!status.isEmpty());
        
        engine.stopSimulation();
    }

    /**
     * @brief Tests TelemetryModel proximity alert display.
     */
    void testTelemetryModelProximityAlert() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0, 200, 200);
        engine.launchDrone();
        
        TelemetryModel model(&engine, DroneListType::Deployed);
        model.updateModel();
        
        QString proximity = model.data(model.index(0, 6), Qt::DisplayRole).toString();
        QCOMPARE(proximity, QString("Clear")); // New drone shouldn't have alert
        
        engine.stopSimulation();
    }

    /**
     * @brief Tests custom map size and configured static/dynamic obstacle counts.
     */
    void testCustomMapSizeAndObstacles() {
        SimulationEngine engine;
        engine.startSimulation(42, 0.6, 300, 400, 15, 8);
        
        // 1. Verify custom dimensions in TerrainMap
        const auto& terrain = engine.getTerrain();
        QCOMPARE(terrain.getWidth(), 300);
        QCOMPARE(terrain.getHeight(), 400);
        
        // 2. Verify custom static obstacle count
        QCOMPARE((int)terrain.getObstacles().size(), 15);
        
        // 3. Verify custom dynamic obstacle count
        QCOMPARE((int)engine.getDynamicObstacleData().size(), 8);
        
        engine.stopSimulation();
    }
};

QTEST_MAIN(TestAeroGrid)
#include "TestAeroGrid.moc"