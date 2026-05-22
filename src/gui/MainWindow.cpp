#include "MainWindow.h"
#include "TerrainView.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QTableView>
#include <QApplication>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    auto* centralWidget = new QWidget(this);
    auto* layout = new QHBoxLayout(centralWidget);
    auto* leftLayout = new QVBoxLayout();

    m_tableView = new QTableView(this);
    m_model = new TelemetryModel(&m_engine, this);
    m_tableView->setModel(m_model);
    m_tableView->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_tableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_tableView->setEditTriggers(QAbstractItemView::NoEditTriggers);

    // CRITICAL: Connect the Engine's signal to the Model's update slot
    connect(&m_engine, &SimulationEngine::simulationUpdated, 
            m_model, &TelemetryModel::updateModel);

    // Control buttons
    auto* startBtn = new QPushButton("Start Simulation", this);
    connect(startBtn, &QPushButton::clicked, &m_engine, &SimulationEngine::startSimulation);

    m_modeSelector = new QComboBox(this);
    m_modeSelector->addItem("Max Altitude", static_cast<int>(NavigationMode::MaxAltitude));
    m_modeSelector->addItem("Terrain Skimming", static_cast<int>(NavigationMode::TerrainSkimming));
    // Ensure the dropdown closes and loses focus after a selection
    connect(m_modeSelector, QOverload<int>::of(&QComboBox::activated), [=](int) { 
        m_modeSelector->clearFocus(); 
    });

    m_selectionLabel = new QLabel("No drones selected", this);

    m_clearQueueBtn = new QPushButton("Clear Selected Queue", this);
    m_landBtn = new QPushButton("Land Drone", this);
    m_takeOffBtn = new QPushButton("Take Off", this);
    m_rtbBtn = new QPushButton("Return to Base", this);

    leftLayout->addWidget(m_tableView);
    leftLayout->addWidget(m_selectionLabel);
    leftLayout->addWidget(new QLabel("Flight Mode:"));
    leftLayout->addWidget(m_modeSelector);
    leftLayout->addWidget(m_clearQueueBtn);
    leftLayout->addWidget(m_landBtn);
    leftLayout->addWidget(m_takeOffBtn);
    leftLayout->addWidget(m_rtbBtn);
    leftLayout->addWidget(startBtn);

    updateButtonStates();

    auto* terrainView = new TerrainView(&m_engine, this);
    // Trigger a repaint of the map whenever simulation data changes
    connect(&m_engine, &SimulationEngine::simulationUpdated, 
            terrainView, QOverload<>::of(&TerrainView::update));
            
    auto updateUI = [this, terrainView]() {
        m_model->setSelectedIds(m_selectedIds);
        terrainView->setSelectedIds(m_selectedIds);
        if (m_selectedIds.empty()) {
            m_selectionLabel->setText("No drones selected");
        } else {
            m_selectionLabel->setText(QString("%1 Drones Selected").arg(m_selectedIds.size()));
        }
        updateButtonStates();
    };

    connect(terrainView, &TerrainView::dronesSelected, [=](const std::set<int>& ids) {
        m_selectedIds = ids;
        updateUI();
    });

    connect(m_tableView, &QTableView::clicked, [=](const QModelIndex &index) {
        int id = m_model->getDroneIdAt(index.row());
        if (m_selectedIds.count(id) && !(QApplication::keyboardModifiers() & Qt::ControlModifier)) {
            m_selectedIds.erase(id); // Deselect if already selected
        } else if (QApplication::keyboardModifiers() & Qt::ControlModifier) {
            m_selectedIds.insert(id);
        } else {
            m_selectedIds = {id};
        }
        updateUI();
    });

    connect(m_rtbBtn, &QPushButton::clicked, [=]() {
        for (int id : m_selectedIds) m_engine.returnToBase(id);
    });

    connect(m_landBtn, &QPushButton::clicked, [=]() {
        for (int id : m_selectedIds) m_engine.landDrone(id);
    });

    connect(terrainView, &TerrainView::mapTargetSet, [=](double x, double y) {
        NavigationMode mode = static_cast<NavigationMode>(m_modeSelector->currentData().toInt());
        for (int id : m_selectedIds) {
            m_engine.assignTarget(id, x, y, mode);
        }
    });

    connect(m_takeOffBtn, &QPushButton::clicked, [=]() {
        for (int id : m_selectedIds) m_engine.takeOff(id);
    });

    connect(m_clearQueueBtn, &QPushButton::clicked, [=]() {
        for (int id : m_selectedIds) m_engine.clearNavQueue(id);
    });

    connect(terrainView, &TerrainView::navPointClicked, [=](int droneId, int index) {
        m_engine.removeNavPoint(droneId, index);
    });

    layout->addLayout(leftLayout);
    layout->addWidget(terrainView);

    setCentralWidget(centralWidget);
}

void MainWindow::updateButtonStates() {
    bool hasSelection = !m_selectedIds.empty();
    m_clearQueueBtn->setEnabled(hasSelection);
    m_landBtn->setEnabled(hasSelection);
    m_takeOffBtn->setEnabled(hasSelection);
    m_rtbBtn->setEnabled(hasSelection);
}