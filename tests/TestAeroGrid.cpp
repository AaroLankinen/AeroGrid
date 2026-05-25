#include <QtTest>
#include <QSignalSpy>
#include <cmath>
#include <QFont>
#include "SimulationEngine.h"
#include "TelemetryModel.h"
#include "Drone.h"

class TestAeroGrid : public QObject {
    Q_OBJECT

private slots:
    /**
     * @brief Verifies that a Drone is correctly initialized with default values.
     */
    void testDroneInitialState() {
        Drone d(101);
        QCOMPARE(d.id, 101);
        QCOMPARE(d.batteryLevel, 100.0);
        QCOMPARE(d.status, DroneStatus::Flying); // Default in constructor
        QCOMPARE(d.radius, 0.3);
        QVERIFY(d.navQueue.empty());
    }

    /**
     * @brief Verifies SimulationEngine setup: terrain, base positioning, and hangar initialization.
     */
    void testEngineInitialization() {
        SimulationEngine engine;
        // Test with 50% land proportion
        engine.startSimulation(12345, 0.5);
        
        // Hangar should start with 12 drones, active fleet should be empty
        QCOMPARE(engine.getInventoryCount(), 12);
        QCOMPARE(engine.getDroneData().size(), 0);
        
        // Verify base location is within the defined grid and on "land" (height >= 1.0)
        double bx = engine.getBaseX();
        double by = engine.getBaseY();
        QVERIFY(bx >= -20.0 && bx <= 20.0);
        QVERIFY(by >= -20.0 && by <= 20.0);
        QVERIFY(engine.getTerrain().getHeightAt(bx, by) >= 1.0);
        
        // Verify dynamic obstacles (birds/other drones) were created
        auto obstacles = engine.getDynamicObstacleData();
        QVERIFY(obstacles.size() >= 3 && obstacles.size() <= 6);
        
        engine.stopSimulation();
    }

    /**
     * @brief Verifies that starting the simulation triggers the simulationUpdated signal.
     */
    void testSimulationSignalEmission() {
        SimulationEngine engine;
        QSignalSpy spy(&engine, &SimulationEngine::simulationUpdated);
        
        engine.startSimulation(1, 1.0);
        
        // Wait for at least one physics cycle (approx 50ms)
        QVERIFY(spy.wait(500));
        QVERIFY(spy.count() > 0);
        
        engine.stopSimulation();
    }

    /**
     * @brief Verifies drone deployment logic from hangar to active fleet.
     */
    void testDroneDeployment() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0);
        
        // Deploy a single drone
        engine.launchDrone();
        QCOMPARE(engine.getInventoryCount(), 11);
        QCOMPARE(engine.getDroneData().size(), 1);
        
        // Check state of the newly deployed drone (should be on helipad)
        Drone d = engine.getDroneData().back();
        QCOMPARE(d.status, DroneStatus::Landed);
        QCOMPARE(d.x, engine.getBaseX());
        QCOMPARE(d.y, engine.getBaseY());

        // Deploy multiple specific IDs
        std::vector<int> ids = {0, 1, 2};
        engine.launchDrones(ids);
        QCOMPARE(engine.getInventoryCount(), 8);
        QCOMPARE(engine.getDroneData().size(), 4);
        
        engine.stopSimulation();
    }

    /**
     * @brief Tests flight commands: takeoff, target assignment, and clearing queue.
     */
    void testFlightCommands() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0);
        engine.launchDrone();
        int id = engine.getDroneData().at(0).id;

        // Takeoff command
        engine.takeOff(id);
        
        QSignalSpy spy(&engine, &SimulationEngine::simulationUpdated);
        QVERIFY(spy.wait(200)); // Process a few cycles
        
        auto drones = engine.getDroneData();
        QCOMPARE(drones.at(0).status, DroneStatus::Flying);
        QVERIFY(drones.at(0).vz > 0); // Should be ascending

        // Assign Navigation Target
        engine.assignTarget(id, 10.0, 10.0, NavigationMode::MaxAltitude);
        drones = engine.getDroneData();
        QCOMPARE(drones.at(0).navQueue.size(), 1);
        QCOMPARE(drones.at(0).navMode, NavigationMode::MaxAltitude);
        
        // Clear Navigation Queue
        engine.clearNavQueue(id);
        drones = engine.getDroneData();
        QVERIFY(drones.at(0).navQueue.empty());
        QCOMPARE(drones.at(0).vx, 0.0); // Should stop horizontal movement

        engine.stopSimulation();
    }

    /**
     * @brief Verifies physics logic execution: battery drain and height changes.
     */
    void testPhysicsAndTelemetry() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0);
        engine.launchDrone();
        int id = engine.getDroneData().at(0).id;
        
        engine.takeOff(id);
        
        // Let physics run for 300ms
        QTest::qWait(300);
        
        auto drones = engine.getDroneData();
        QVERIFY(drones.at(0).batteryLevel < 100.0); // Battery should drain
        QVERIFY(drones.at(0).z > engine.getTerrain().getHeightAt(drones.at(0).x, drones.at(0).y)); // Should be in the air

        engine.stopSimulation();
    }

    /**
     * @brief Verifies TelemetryModel correctly formats drone data for the UI.
     */
    void testTelemetryModelFormatting() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0);
        engine.launchDrone(); // Deploys drone (likely ID 11 from hangar end)

        TelemetryModel model(&engine, DroneListType::Deployed);
        model.updateModel();

        // Check row and column counts
        QCOMPARE(model.rowCount(), 1);
        QCOMPARE(model.columnCount(), 7);

        // Verify ID column (string representation)
        QCOMPARE(model.data(model.index(0, 0), Qt::DisplayRole).toString(), QString("11"));
        
        // Verify Battery formatting (1 decimal + %)
        QCOMPARE(model.data(model.index(0, 4), Qt::DisplayRole).toString(), QString("100.0%"));
        
        // Verify Header text
        QCOMPARE(model.headerData(0, Qt::Orientation::Horizontal, Qt::DisplayRole).toString(), QString("Drone ID"));
        
        engine.stopSimulation();
    }

    /**
     * @brief Verifies selection-based UI changes (bold font and indicator).
     */
    void testModelSelectionVisuals() {
        SimulationEngine engine;
        engine.startSimulation(1, 1.0);
        engine.launchDrone(); 
        
        TelemetryModel model(&engine, DroneListType::Deployed);
        model.updateModel();

        model.setSelectedIds({11}); // Select the drone
        
        // Selected ID should now have a visual indicator arrow
        QString idStr = model.data(model.index(0, 0), Qt::DisplayRole).toString();
        QVERIFY(idStr.contains("→"));
        
        // Font should be bold when selected
        QVariant font = model.data(model.index(0, 0), Qt::FontRole);
        QVERIFY(font.value<QFont>().bold());
        
        engine.stopSimulation();
    }
};

QTEST_MAIN(TestAeroGrid)
#include "TestAeroGrid.moc"