#include "MainWindow.h"
#include "TerrainView.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
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

    leftLayout->addWidget(m_tableView);
    leftLayout->addWidget(startBtn);

    auto* terrainView = new TerrainView(&m_engine, this);
    // Trigger a repaint of the map whenever simulation data changes
    connect(&m_engine, &SimulationEngine::simulationUpdated, 
            terrainView, QOverload<>::of(&TerrainView::update));

    layout->addLayout(leftLayout);
    layout->addWidget(terrainView);

    setCentralWidget(centralWidget);
}