#include "MainWindow.h"
#include "TerrainView.h"
#include <QRandomGenerator>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QTableView>
#include <QApplication>
#include <QItemSelectionModel>
#include <QSignalBlocker>

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

    // Seed Control
    auto* seedLayout = new QHBoxLayout();
    auto* seedInput = new QLineEdit(this);
    seedInput->setPlaceholderText("Map Seed (Numeric)");
    seedInput->setText(QString::number(12345));
    auto* randSeedBtn = new QPushButton("🎲", this);
    seedLayout->addWidget(new QLabel("Seed:"));
    seedLayout->addWidget(seedInput);
    seedLayout->addWidget(randSeedBtn);

    // Control buttons
    auto* startBtn = new QPushButton("Start Simulation", this);
    
    auto* terrainView = new TerrainView(&m_engine, this);

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
    leftLayout->addLayout(seedLayout);
    leftLayout->addWidget(m_selectionLabel);
    leftLayout->addWidget(new QLabel("Flight Mode:"));
    leftLayout->addWidget(m_modeSelector);
    leftLayout->addWidget(m_clearQueueBtn);
    leftLayout->addWidget(m_landBtn);
    leftLayout->addWidget(m_takeOffBtn);
    leftLayout->addWidget(m_rtbBtn);
    leftLayout->addWidget(startBtn);

    // Connections for Seed and Start
    connect(randSeedBtn, &QPushButton::clicked, [=]() {
        seedInput->setText(QString::number(QRandomGenerator::global()->generate() % 999999));
    });

    connect(startBtn, &QPushButton::clicked, [this, seedInput, terrainView]() {
        unsigned int seed = seedInput->text().toUInt();
        if (seed == 0 && seedInput->text() != "0") seed = 12345;
        
        m_engine.startSimulation(seed);
        
        // Force the terrain view to re-draw its procedural cache for the new seed
        terrainView->renderTerrainCache();
        terrainView->update();
    });

    updateButtonStates();

    // Trigger a repaint of the map whenever simulation data changes
    connect(&m_engine, &SimulationEngine::simulationUpdated, 
            terrainView, QOverload<>::of(&TerrainView::update));
            
    auto updateUI = [this, terrainView]() {
        // Sync set -> table view selection visually
        if (m_tableView->selectionModel()) {
            QSignalBlocker blocker(m_tableView->selectionModel());
            m_tableView->selectionModel()->clearSelection();
            for (int i = 0; i < m_model->rowCount(); ++i) {
                if (m_selectedIds.count(m_model->getDroneIdAt(i))) {
                    m_tableView->selectionModel()->select(m_model->index(i, 0), 
                        QItemSelectionModel::Select | QItemSelectionModel::Rows);
                }
            }
        }
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

    // Synchronize Table Selection -> internal set
    // This handles Shift+Click (ranges) and Ctrl+Click (multi) automatically
    connect(m_tableView->selectionModel(), &QItemSelectionModel::selectionChanged, 
            [this, terrainView](const QItemSelection &selected, const QItemSelection &deselected) {
        m_selectedIds.clear();
        for (const auto& index : m_tableView->selectionModel()->selectedRows()) {
            int id = m_model->getDroneIdAt(index.row());
            if (id != -1) m_selectedIds.insert(id);
        }
        
        // Update Map and Model (without re-syncing the table selection)
        m_model->setSelectedIds(m_selectedIds);
        terrainView->setSelectedIds(m_selectedIds);
        
        if (m_selectedIds.empty()) {
            m_selectionLabel->setText("No drones selected");
        } else {
            m_selectionLabel->setText(QString("%1 Drones Selected").arg(m_selectedIds.size()));
        }
        updateButtonStates();
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