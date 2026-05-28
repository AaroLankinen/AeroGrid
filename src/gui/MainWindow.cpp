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
#include <QHeaderView>
#include <QApplication>
#include <QItemSelectionModel>
#include <QGroupBox>
#include <QSignalBlocker>
#include <QScrollArea>
#include <QPainter>
#include <QVector2D>
#include <QtMath> // For qCos, qSin, qSqrt
#include <QColor>
#include <algorithm>
#include <cmath>

/**
 * @brief Specialized widget that renders a first-person perspective from a drone's camera.
 * 
 * Projects the heightmap data from the SimulationEngine's TerrainMap using the 
 * drone's 3D position and orientation (yaw/pitch).
 */
class DroneCameraWidget : public QWidget {
public:
    DroneCameraWidget(int droneId, SimulationEngine* engine, QWidget* parent = nullptr)
        : QWidget(parent), m_droneId(droneId), m_engine(engine) {
        setFixedSize(240, 135); // 16:9 aspect ratio
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.fillRect(rect(), Qt::black);

        const Drone* drone = m_engine->getDroneById(m_droneId);
        if (!drone) return; // Drone not found or no longer active

        const TerrainMap& terrain = m_engine->getTerrain();

        // Render simplified horizon/ground projection
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(0, 0, width(), height() / 2, QColor(100, 149, 237)); // Sky

        double yawRad = drone->yaw * (M_PI / 180.0);
        QVector2D forward(qCos(yawRad), qSin(yawRad));
        QVector2D right(-forward.y(), forward.x());

        const int numCols = 32;
        const int numRows = 12;
        const double step = 3.0;
        
        for (int z = numRows; z > 0; --z) {
            double dist = z * step;
            for (int x = -numCols / 2; x < numCols / 2; ++x) { // Corrected loop condition
                double xOff = x * step * (dist / 15.0);
                double worldX = drone->x + forward.x() * dist + right.x() * xOff;
                double worldY = drone->y + forward.y() * dist + right.y() * xOff;
                double h = terrain.getHeightAt(worldX, worldY);
                
                double screenX = (x + numCols / 2) * (width() / (double)numCols); // Corrected division to double
                double screenY = height() / 2.0 + (drone->z - h) * (4.0 / (dist + 1.0)) * (height() / 12.0); // Corrected division to double
                
                QColor color = (h < 1.0) ? QColor(20, 60, 200) : QColor(34, 139, 34);
                painter.setBrush(color.darker(100 + (z * 5)));
                painter.setPen(Qt::NoPen);
                painter.drawRect(QRectF(screenX, screenY, (width() / (double)numCols) + 1, height() / 4.0));
            }
        }

        painter.setPen(Qt::white);
        painter.drawText(5, 15, QString("D%1 | ALT: %2m").arg(m_droneId).arg(drone->z, 0, 'f', 1));
        painter.setPen(QColor(255, 255, 255, 100));
        painter.drawLine(width()/2 - 10, height()/2, width()/2 + 10, height()/2);
        painter.drawLine(width()/2, height()/2 - 10, width()/2, height()/2 + 10);
    }

private:
    int m_droneId;
    SimulationEngine* m_engine;
};

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    auto* centralWidget = new QWidget(this);
    auto* mainLayout = new QVBoxLayout(centralWidget);

    // --- 0. Remote Camera Views (Top) ---
    auto* cameraScroll = new QScrollArea(this);
    cameraScroll->setWidgetResizable(true);
    cameraScroll->setFixedHeight(160);
    m_cameraScrollContent = new QWidget(this);
    m_cameraLayout = new QHBoxLayout(m_cameraScrollContent);
    m_cameraLayout->setContentsMargins(5, 5, 5, 5);
    m_cameraLayout->setAlignment(Qt::AlignLeft);
    cameraScroll->setWidget(m_cameraScrollContent);
    cameraScroll->hide();
    mainLayout->addWidget(cameraScroll);

    // --- 1. Map Generation Control Group (Top) ---
    auto* configGroup = new QGroupBox("Map Generation Settings", this);
    auto* configLayout = new QVBoxLayout(configGroup);

    // Seed UI
    auto* seedLayout = new QHBoxLayout();
    auto* seedInput = new QLineEdit(this);
    seedInput->setPlaceholderText("Map Seed (Numeric)");
    seedInput->setText(QString::number(12345));
    auto* randSeedBtn = new QPushButton("🎲", this);
    seedLayout->addWidget(new QLabel("Seed:", this));
    seedLayout->addWidget(seedInput);
    seedLayout->addWidget(randSeedBtn);
    configLayout->addLayout(seedLayout);

    // Map Presets UI
    auto* dimLayout = new QHBoxLayout();
    m_mapPresetCombo = new QComboBox(this);
    m_mapPresetCombo->addItem("Small (250x250)", 250);
    m_mapPresetCombo->addItem("Medium (500x500)", 500);
    m_mapPresetCombo->addItem("Large (1000x1000)", 1000);
    m_mapPresetCombo->setCurrentIndex(1); // Default to Medium

    // Ensure the dropdown closes and loses focus after a selection
    connect(m_mapPresetCombo, QOverload<int>::of(&QComboBox::activated), [this](int) {
        m_mapPresetCombo->clearFocus();
    });

    dimLayout->addWidget(new QLabel("Map Size:", this));
    dimLayout->addWidget(m_mapPresetCombo);
    configLayout->addLayout(dimLayout);

    // Slider UI
    m_landWaterSlider = new QSlider(Qt::Horizontal, this);
    m_landWaterSlider->setRange(0, 100);
    m_landWaterSlider->setValue(50);
    m_landWaterLabel = new QLabel("Land: 50% | Water: 50%", this);
    configLayout->addWidget(new QLabel("Land vs Water Proportion:", this));
    configLayout->addWidget(m_landWaterSlider);
    configLayout->addWidget(m_landWaterLabel);

    auto* startBtn = new QPushButton("Start Simulation", this);
    configLayout->addWidget(startBtn);
    mainLayout->addWidget(configGroup);

    // --- 2. Middle Section: Hangar (Left) | Map (Center) | Deployed (Right) ---
    auto* middleLayout = new QHBoxLayout();

    // Left: Hangar Inventory
    auto* hangarContainer = new QWidget(this);
    auto* hangarLayout = new QVBoxLayout(hangarContainer);
    hangarLayout->setContentsMargins(0, 0, 0, 0);
    hangarLayout->addWidget(new QLabel("<b>Hangar Inventory</b>", this));
    m_hangarTableView = new QTableView(this);
    m_hangarModel = new TelemetryModel(&m_engine, DroneListType::Hangar, this);
    m_hangarTableView->setModel(m_hangarModel);
    m_hangarTableView->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_hangarTableView->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_hangarTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_hangarTableView->horizontalHeader()->setStretchLastSection(true);
    hangarLayout->addWidget(m_hangarTableView);

    // Center: Terrain View
    m_terrainView = new TerrainView(&m_engine, this);
    m_terrainView->hide();

    // Right: Deployed Drones
    auto* deployedContainer = new QWidget(this);
    auto* deployedLayout = new QVBoxLayout(deployedContainer);
    deployedLayout->setContentsMargins(0, 0, 0, 0);
    deployedLayout->addWidget(new QLabel("<b>Deployed Drones</b>", this));
    m_tableView = new QTableView(this);
    m_model = new TelemetryModel(&m_engine, DroneListType::Deployed, this);
    m_tableView->setModel(m_model);
    m_tableView->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_tableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_tableView->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_tableView->horizontalHeader()->setStretchLastSection(true);
    deployedLayout->addWidget(m_tableView);

    // Enable clickable sorting for hangar table
    connect(m_hangarTableView->horizontalHeader(), &QHeaderView::sectionClicked, [this](int column) {
        Qt::SortOrder order = Qt::AscendingOrder;
        if (m_hangarModel->m_sortColumn == column && m_hangarModel->m_sortOrder == Qt::AscendingOrder) {
            order = Qt::DescendingOrder;
        }
        m_hangarModel->sort(column, order);
    });

    // Enable clickable sorting for deployed table
    connect(m_tableView->horizontalHeader(), &QHeaderView::sectionClicked, [this](int column) {
        Qt::SortOrder order = Qt::AscendingOrder;
        if (m_model->m_sortColumn == column && m_model->m_sortOrder == Qt::AscendingOrder) {
            order = Qt::DescendingOrder;
        }
        m_model->sort(column, order);
    });

    // Constrain map to a central square and let tables fill extra width
    // Stretch factors 1, 0, 1 make the side panels greedy.
    middleLayout->addWidget(hangarContainer, 1);
    middleLayout->addWidget(m_terrainView, 0); 
    middleLayout->addWidget(deployedContainer, 1);
    mainLayout->addLayout(middleLayout, 1);  // Give middle section most vertical space

    // --- 3. Bottom Section: Drone Controls ---
    m_simUIContainer = new QWidget(this);
    auto* controlsLayout = new QHBoxLayout(m_simUIContainer);
    controlsLayout->setContentsMargins(0, 0, 0, 0);

    m_selectionLabel = new QLabel("No drones selected", this);
    m_modeSelector = new QComboBox(this);
    m_modeSelector->addItem("Max Altitude", static_cast<int>(NavigationMode::MaxAltitude));
    m_modeSelector->addItem("Terrain Skimming", static_cast<int>(NavigationMode::TerrainSkimming));
    // Ensure the dropdown closes and loses focus after a selection
    connect(m_modeSelector, QOverload<int>::of(&QComboBox::activated), [this](int) { 
        m_modeSelector->clearFocus(); 
    });

    m_launchBtn = new QPushButton("Launch Drone (12 Available)", this);
    m_clearQueueBtn = new QPushButton("Clear Selected Queue", this);
    m_landBtn = new QPushButton("Land Drone", this);
    m_takeOffBtn = new QPushButton("Take Off", this);
    m_rtbBtn = new QPushButton("Return to Base", this);

    controlsLayout->addWidget(m_selectionLabel);
    controlsLayout->addWidget(new QLabel("Flight Mode:", this));
    controlsLayout->addWidget(m_modeSelector);
    controlsLayout->addWidget(m_launchBtn);
    controlsLayout->addWidget(m_clearQueueBtn);
    controlsLayout->addWidget(m_takeOffBtn);
    controlsLayout->addWidget(m_landBtn);
    controlsLayout->addWidget(m_rtbBtn);
    mainLayout->addWidget(m_simUIContainer);
    m_simUIContainer->hide();


    auto updateSelectionLabel = [this]() {
        if (m_selectedIds.empty()) {
            m_selectionLabel->setText("No drones selected");
        } else {
            m_selectionLabel->setText(QString("%1 Drones Selected").arg(m_selectedIds.size()));
        }
    };

    auto syncSelectionState = [this, updateSelectionLabel]() {
        m_model->setSelectedIds(m_selectedIds);
        m_terrainView->setSelectedIds(m_selectedIds);
        updateSelectionLabel();
        updateButtonStates();
        updateCameraViews();
    };

    auto selectedIdsFromView = [this](QTableView* view, TelemetryModel* model) {
        std::set<int> ids;
        if (!view->selectionModel()) return ids;
        for (const auto& index : view->selectionModel()->selectedRows()) {
            int id = model->getDroneIdAt(index.row());
            if (id != -1) ids.insert(id);
        }
        return ids;
    };

    auto applyToSelected = [this](auto action) {
        for (int id : m_selectedIds) action(id);
    };

    connect(randSeedBtn, &QPushButton::clicked, [seedInput]() {
        seedInput->setText(QString::number(QRandomGenerator::global()->generate() % 999999));
    });

    connect(m_landWaterSlider, &QSlider::valueChanged, [this](int value) {
        m_landWaterLabel->setText(QString("Land: %1% | Water: %2%").arg(value).arg(100 - value));
    });

    connect(startBtn, &QPushButton::clicked, [this, seedInput, startBtn]() {
        unsigned int seed = seedInput->text().toUInt();
        if (seed == 0 && seedInput->text() != "0") seed = 12345;
        
        double landProp = m_landWaterSlider->value() / 100.0;
        int size = m_mapPresetCombo->currentData().toInt();
        m_engine.startSimulation(seed, landProp, size, size);
        
        m_terrainView->resetView();
        m_terrainView->renderTerrainCache();
        m_terrainView->show();
        m_simUIContainer->show();
        startBtn->setText("Regenerate Map");
    });

    connect(&m_engine, &SimulationEngine::simulationUpdated, [this]() {
        m_terrainView->update();
        for (int i = 0; i < m_cameraLayout->count(); ++i) {
            if (auto* w = m_cameraLayout->itemAt(i)->widget()) w->update();
        }
        m_model->updateModel();
        m_hangarModel->updateModel();
        int count = m_engine.getInventoryCount();
        m_launchBtn->setText(QString("Launch Drone (%1 Available)").arg(count));
        m_launchBtn->setEnabled(count > 0);
    });

    connect(m_launchBtn, &QPushButton::clicked, [this]() {
        if (m_selectedHangarIds.empty()) {
            m_engine.launchDrone();
        } else {
            std::vector<int> ids(m_selectedHangarIds.begin(), m_selectedHangarIds.end());
            m_engine.launchDrones(ids);
            m_selectedHangarIds.clear();
        }
    });
            
    auto updateUI = [this]() {
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
        m_terrainView->setSelectedIds(m_selectedIds);
        if (m_selectedIds.empty()) {
            m_selectionLabel->setText("No drones selected");
        } else {
            m_selectionLabel->setText(QString("%1 Drones Selected").arg(m_selectedIds.size()));
        }
        updateButtonStates();
    };

    connect(m_terrainView, &TerrainView::dronesSelected, [this, syncSelectionState](const std::set<int>& ids) {
        m_selectedIds = ids;
        syncSelectionState();
    });

    connect(m_tableView->selectionModel(), &QItemSelectionModel::selectionChanged, 
            [this, syncSelectionState, selectedIdsFromView](const QItemSelection&, const QItemSelection&) {
        m_selectedIds = selectedIdsFromView(m_tableView, m_model);
        syncSelectionState();
    });

    connect(m_hangarTableView->selectionModel(), &QItemSelectionModel::selectionChanged, [this, selectedIdsFromView]() {
        m_selectedHangarIds = selectedIdsFromView(m_hangarTableView, m_hangarModel);
        m_hangarModel->setSelectedIds(m_selectedHangarIds);
    });

    connect(m_rtbBtn, &QPushButton::clicked, [this, applyToSelected]() {
        applyToSelected([this](int id) { m_engine.returnToBase(id); });
    });

    connect(m_landBtn, &QPushButton::clicked, [this, applyToSelected]() {
        applyToSelected([this](int id) { m_engine.landDrone(id); });
    });

    connect(m_terrainView, &TerrainView::mapTargetSet, [=](double x, double y) {
        NavigationMode mode = static_cast<NavigationMode>(m_modeSelector->currentData().toInt());
        for (int id : m_selectedIds) {
            m_engine.assignTarget(id, x, y, mode);
        }
    });

    connect(m_takeOffBtn, &QPushButton::clicked, [this, applyToSelected]() {
        applyToSelected([this](int id) { m_engine.takeOff(id); });
    });

    connect(m_clearQueueBtn, &QPushButton::clicked, [this, applyToSelected]() {
        applyToSelected([this](int id) { m_engine.clearNavQueue(id); });
    });

    connect(m_terrainView, &TerrainView::navPointClicked, [=](int droneId, int index) {
        m_engine.removeNavPoint(droneId, index);
    });

    updateButtonStates();
    setCentralWidget(centralWidget);
}

void MainWindow::updateCameraViews() {
    QLayoutItem* child;
    while ((child = m_cameraLayout->takeAt(0)) != nullptr) {
        if (child->widget()) child->widget()->deleteLater();
        delete child;
    }

    for (int id : m_selectedIds) {
        auto* cam = new DroneCameraWidget(id, &m_engine, this);
        m_cameraLayout->addWidget(cam);
    }
    
    // Show scroll area only if cameras exist
    m_cameraScrollContent->parentWidget()->parentWidget()->setVisible(!m_selectedIds.empty());
}

void MainWindow::updateButtonStates() {
    bool hasSelection = !m_selectedIds.empty();
    m_clearQueueBtn->setEnabled(hasSelection);
    m_landBtn->setEnabled(hasSelection);
    m_takeOffBtn->setEnabled(hasSelection);
    m_rtbBtn->setEnabled(hasSelection);
}