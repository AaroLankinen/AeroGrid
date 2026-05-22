#include "MainWindow.h"
#include "TerrainView.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QTableView>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    auto* centralWidget = new QWidget(this);
    auto* layout = new QHBoxLayout(centralWidget);
    auto* leftLayout = new QVBoxLayout();

    m_tableView = new QTableView(this);
    m_model = new TelemetryModel(&m_engine, this);
    m_tableView->setModel(m_model);

    // CRITICAL: Connect the Engine's signal to the Model's update slot
    connect(&m_engine, &SimulationEngine::simulationUpdated, 
            m_model, &TelemetryModel::updateModel);

    // Control buttons
    auto* startBtn = new QPushButton("Start Simulation", this);
    connect(startBtn, &QPushButton::clicked, &m_engine, &SimulationEngine::startSimulation);

    auto* modeSelector = new QComboBox(this);
    modeSelector->addItem("Max Altitude", static_cast<int>(NavigationMode::MaxAltitude));
    modeSelector->addItem("Terrain Skimming", static_cast<int>(NavigationMode::TerrainSkimming));

    auto* selectionLabel = new QLabel("Select a drone on the map", this);

    leftLayout->addWidget(m_tableView);
    leftLayout->addWidget(selectionLabel);
    leftLayout->addWidget(new QLabel("Flight Mode:"));
    leftLayout->addWidget(modeSelector);
    leftLayout->addWidget(startBtn);

    auto* terrainView = new TerrainView(&m_engine, this);
    // Trigger a repaint of the map whenever simulation data changes
    connect(&m_engine, &SimulationEngine::simulationUpdated, 
            terrainView, QOverload<>::of(&TerrainView::update));
            
    static int currentSelectedId = -1;
    connect(terrainView, &TerrainView::droneSelected, [=](int id) {
        currentSelectedId = id;
        selectionLabel->setText(QString("Drone %1 Selected").arg(id));
    });

    connect(terrainView, &TerrainView::mapTargetSet, [=](double x, double y) {
        if (currentSelectedId != -1) {
            NavigationMode mode = static_cast<NavigationMode>(modeSelector->currentData().toInt());
            m_engine.assignTarget(currentSelectedId, x, y, mode);
        }
    });

    layout->addLayout(leftLayout);
    layout->addWidget(terrainView);

    setCentralWidget(centralWidget);
}