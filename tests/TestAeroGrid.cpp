#include <QtTest>
#include <QSignalSpy>
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
        QCOMPARE(d.x, 0.0);
        QCOMPARE(d.vx, 0.0);
    }

    /**
     * @brief Verifies that the SimulationEngine populates the default drone fleet.
     */
    void testEngineSetup() {
        SimulationEngine engine;
        QCOMPARE(engine.getDroneData().size(), 5);
    }

    /**
     * @brief Verifies that starting the simulation triggers the simulationUpdated signal.
     */
    void testSimulationSignalEmission() {
        SimulationEngine engine;
        QSignalSpy spy(&engine, &SimulationEngine::simulationUpdated);
        
        engine.startSimulation();
        
        // The engine loop has a 50ms sleep; wait up to 500ms for at least one emission
        QVERIFY(spy.wait(500));
        QVERIFY(spy.count() > 0);
        
        engine.stopSimulation();
    }

    /**
     * @brief Verifies that the internal simulation logic (like battery drain) is executing.
     */
    void testBatteryConsumption() {
        SimulationEngine engine;
        auto initialData = engine.getDroneData();
        double startBattery = initialData.at(0).batteryLevel;
        
        engine.startSimulation();
        QTest::qWait(200); // Wait for approximately 4 cycles of 50ms
        engine.stopSimulation();
        
        auto finalData = engine.getDroneData();
        QVERIFY(finalData.at(0).batteryLevel < startBattery);
    }

    /**
     * @brief Verifies that TelemetryModel correctly formats drone data for the UI.
     */
    void testTelemetryModelFormatting() {
        SimulationEngine engine;
        TelemetryModel model(&engine);

        // Fetch initial data from engine into model
        model.updateModel();

        // Verify formatting for the first drone (ID 0)
        // Column 0: ID (returned as int/QVariant)
        QCOMPARE(model.data(model.index(0, 0), Qt::DisplayRole).toInt(), 0);

        // Column 1: X (String, formatted to 2 decimals: "0.00")
        QCOMPARE(model.data(model.index(0, 1), Qt::DisplayRole).toString(), QString("0.00"));

        // Column 4: Battery (String, 1 decimal + %: "100.0%")
        QCOMPARE(model.data(model.index(0, 4), Qt::DisplayRole).toString(), QString("100.0%"));
    }
};

QTEST_MAIN(TestAeroGrid)
#include "TestAeroGrid.moc"