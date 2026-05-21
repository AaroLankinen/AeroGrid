#include "MainWindow.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    auto* centralWidget = new QWidget(this);
    auto* layout = new QVBoxLayout(centralWidget);

    m_tableView = new QTableView(this);
    m_model = new TelemetryModel(&m_engine, this);
    m_tableView->setModel(m_model);

    // CRITICAL: Connect the Engine's signal to the Model's update slot
    connect(&m_engine, &SimulationEngine::simulationUpdated, 
            m_model, &TelemetryModel::updateModel);

    // Control buttons
    auto* startBtn = new QPushButton("Start Simulation", this);
    connect(startBtn, &QPushButton::clicked, &m_engine, &SimulationEngine::startSimulation);

    layout->addWidget(m_tableView);
    layout->addWidget(startBtn);
    setCentralWidget(centralWidget);
}