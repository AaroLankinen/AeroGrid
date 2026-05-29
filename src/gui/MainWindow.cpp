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
#include <QImage>
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

        int w = width();
        int h = height();

        // Check if the drone is underground or crashed
        double groundHeight = terrain.getHeightAt(drone->x, drone->y);
        bool isUnderground = (drone->z < groundHeight - 0.5);
        bool isCrashed = (drone->status == DroneStatus::Crashed);

        // Render everything onto a QImage to allow pixel-level depth buffering
        QImage image(w, h, QImage::Format_ARGB32);
        
        // Render sky gradient in the top half, default water/ground in the bottom half
        for (int py = 0; py < h; ++py) {
            QColor color;
            if (py < h / 2) {
                // Sky gradient: transition from blue at top to sky blue at horizon
                double t = py / (h / 2.0);
                color = QColor(
                    qBound(0, qRound(60 + t * 40), 255),
                    qBound(0, qRound(100 + t * 49), 255),
                    qBound(0, qRound(200 + t * 37), 255)
                );
            } else {
                // Ground backdrop: default dark blue water
                color = QColor(20, 60, 200);
            }
            for (int px = 0; px < w; ++px) {
                image.setPixelColor(px, py, color);
            }
        }

        // Initialize Z-buffer (depth buffer) with infinity
        std::vector<std::vector<double>> zBuffer(w, std::vector<double>(h, 1e9));

        double yawRad = drone->yaw * (M_PI / 180.0);
        QVector2D forward(qCos(yawRad), qSin(yawRad));
        QVector2D right(-forward.y(), forward.x());

        const int numCols = 32;
        const int numRows = 12;
        const double step = 3.0;

        // 1. Render Terrain (from back to front)
        for (int z = numRows; z > 0; --z) {
            double dist = z * step;
            for (int x = -numCols / 2; x < numCols / 2; ++x) {
                double xOff = x * step * (dist / 15.0);
                double worldX = drone->x + forward.x() * dist + right.x() * xOff;
                double worldY = drone->y + forward.y() * dist + right.y() * xOff;
                double terrainH = terrain.getHeightAt(worldX, worldY);

                double screenX, screenY, dummyDist;
                // Use helper to project point on terrain
                if (!projectPoint(worldX, worldY, terrainH, drone, forward, right, screenX, screenY, dummyDist, w, h)) {
                    continue;
                }

                double colWidth = (double)w / numCols;
                int xStart = qMax(0, qRound((x + numCols / 2.0) * colWidth));
                int xEnd = qMin(w - 1, qRound((x + numCols / 2.0 + 1.0) * colWidth));
                int yStart = qMax(0, qRound(screenY));
                int yEnd = h - 1;

                QColor color = (terrainH < 1.0) ? QColor(20, 60, 200) : QColor(34, 139, 34);
                QColor darkenedColor = color.darker(100 + (z * 5));

                for (int px = xStart; px <= xEnd; ++px) {
                    for (int py = yStart; py <= yEnd; ++py) {
                        if (dist <= zBuffer[px][py]) {
                            zBuffer[px][py] = dist;
                            image.setPixelColor(px, py, darkenedColor);
                        }
                    }
                }
            }
        }

        // 2. Render Static Obstacles
        const auto& staticObstacles = terrain.getObstacles();
        for (const auto& obs : staticObstacles) {
            double obsH = terrain.getHeightAt(obs.x, obs.y);
            double zBottom = obsH - 0.5; // Slightly lower than ground to prevent gaps
            double zTop = obsH + obs.height;

            double projX, projY_bottom, dist;
            if (!projectPoint(obs.x, obs.y, zBottom, drone, forward, right, projX, projY_bottom, dist, w, h)) {
                continue;
            }
            if (dist > 150.0) continue; // Out of view distance

            double projY_top;
            double dummyDist;
            projectPoint(obs.x, obs.y, zTop, drone, forward, right, projX, projY_top, dummyDist, w, h);

            double K_x = 5.0 * (w / 32.0);
            double halfWidth = (obs.radius / dist) * K_x;
            if (halfWidth < 0.5) halfWidth = 0.5;

            int xStart = qRound(projX - halfWidth);
            int xEnd = qRound(projX + halfWidth);
            int yStart = qRound(projY_top);
            int yEnd = qRound(projY_bottom);

            for (int px = xStart; px <= xEnd; ++px) {
                if (px < 0 || px >= w) continue;

                // Cylinder horizontal shading (darker at edges)
                double t = 0.0;
                if (halfWidth > 0.1) {
                    t = qAbs(px - projX) / halfWidth;
                }
                t = qBound(0.0, t, 1.0);

                // Base crimson red color, shaded by distance
                int red = qBound(50, 200 - qRound(dist * 0.8), 255);
                int green = qBound(10, 30 - qRound(dist * 0.1), 255);
                int blue = qBound(10, 30 - qRound(dist * 0.1), 255);

                double shade = 1.0 - 0.4 * t;
                red = qBound(0, qRound(red * shade), 255);
                green = qBound(0, qRound(green * shade), 255);
                blue = qBound(0, qRound(blue * shade), 255);

                for (int py = yStart; py <= yEnd; ++py) {
                    if (py < 0 || py >= h) continue;

                    if (dist <= zBuffer[px][py]) {
                        zBuffer[px][py] = dist;

                        bool isOutline = (px == xStart || px == xEnd || py == yStart || py == yEnd);
                        if (isOutline) {
                            image.setPixelColor(px, py, QColor(40, 5, 5));
                        } else {
                            image.setPixelColor(px, py, QColor(red, green, blue));
                        }
                    }
                }
            }
        }

        // 3. Render Dynamic Obstacles
        auto dynamicObstacles = m_engine->getDynamicObstacleData();
        for (const auto& obs : dynamicObstacles) {
            double projX, projY, dist;
            if (!projectPoint(obs.x, obs.y, obs.z, drone, forward, right, projX, projY, dist, w, h)) {
                continue;
            }
            if (dist > 150.0) continue;

            double K_x = 5.0 * (w / 32.0);
            double radiusPx = (obs.radius / dist) * K_x;
            if (radiusPx < 0.5) radiusPx = 0.5;

            int xStart = qRound(projX - radiusPx);
            int xEnd = qRound(projX + radiusPx);
            int yStart = qRound(projY - radiusPx);
            int yEnd = qRound(projY + radiusPx);

            for (int px = xStart; px <= xEnd; ++px) {
                if (px < 0 || px >= w) continue;
                for (int py = yStart; py <= yEnd; ++py) {
                    if (py < 0 || py >= h) continue;

                    double dx = px - projX;
                    double dy = py - projY;
                    double distSq = dx*dx + dy*dy;
                    if (distSq <= radiusPx * radiusPx) {
                        if (dist <= zBuffer[px][py]) {
                            zBuffer[px][py] = dist;

                            double t = qSqrt(distSq) / radiusPx;
                            t = qBound(0.0, t, 1.0);

                            // Base orange color, shaded by distance
                            int red = qBound(50, 240 - qRound(dist * 0.8), 255);
                            int green = qBound(20, 120 - qRound(dist * 0.4), 255);
                            int blue = 0;

                            double shade = 1.0 - 0.4 * t;
                            red = qBound(0, qRound(red * shade), 255);
                            green = qBound(0, qRound(green * shade), 255);
                            blue = qBound(0, qRound(blue * shade), 255);

                            bool isOutline = (distSq > (radiusPx - 1.0) * (radiusPx - 1.0));
                            if (isOutline) {
                                image.setPixelColor(px, py, QColor(40, 20, 0));
                            } else {
                                image.setPixelColor(px, py, QColor(red, green, blue));
                            }
                        }
                    }
                }
            }
        }

        // Apply crash/underground screen overrides
        if (isCrashed) {
            // Analog static noise effect
            for (int py = 0; py < h; ++py) {
                for (int px = 0; px < w; ++px) {
                    int val = QRandomGenerator::global()->bounded(256);
                    image.setPixelColor(px, py, qRgb(val, val, val));
                }
            }
        } else if (isUnderground) {
            // Dark brown dirt texture with static noise
            image.fill(QColor(54, 38, 27));
            for (int i = 0; i < 500; ++i) {
                int px = QRandomGenerator::global()->bounded(w);
                int py = QRandomGenerator::global()->bounded(h);
                int noise = QRandomGenerator::global()->bounded(30) - 15;
                QColor c = image.pixelColor(px, py);
                image.setPixelColor(px, py, QColor(
                    qBound(0, c.red() + noise, 255),
                    qBound(0, c.green() + noise, 255),
                    qBound(0, c.blue() + noise, 255)
                ));
            }
        }

        // Draw the main image on the widget
        painter.drawImage(0, 0, image);

        // Draw HUD overlay elements
        painter.setRenderHint(QPainter::Antialiasing);

        if (isCrashed) {
            painter.setPen(Qt::red);
            QFont f = painter.font();
            f.setBold(true);
            f.setPointSize(10);
            painter.setFont(f);
            painter.drawText(rect(), Qt::AlignCenter, "⚠️ CONNECTION LOST\n(CRASHED)");
        } else if (isUnderground) {
            painter.setPen(QColor(230, 126, 34)); // Warning orange
            QFont f = painter.font();
            f.setBold(true);
            f.setPointSize(10);
            painter.setFont(f);
            painter.drawText(rect(), Qt::AlignCenter, "⚠️ CAMERA OBSTRUCTED\n(UNDERGROUND)");
        } else {
            // Draw crosshairs
            painter.setPen(QColor(255, 255, 255, 100));
            painter.drawLine(w/2 - 10, h/2, w/2 + 10, h/2);
            painter.drawLine(w/2, h/2 - 10, w/2, h/2 + 10);
        }

        // Draw overlay text: ID and altitude
        painter.setPen(Qt::white);
        QFont f = painter.font();
        f.setBold(false);
        f.setPointSize(8);
        painter.setFont(f);
        painter.drawText(5, 15, QString("D%1 | ALT: %2m").arg(m_droneId).arg(drone->z, 0, 'f', 1));
    }

private:
    bool projectPoint(double wx, double wy, double wz, const Drone* drone,
                      const QVector2D& forward, const QVector2D& right,
                      double& outX, double& outY, double& outDist, int w, int h) {
        double dx = wx - drone->x;
        double dy = wy - drone->y;
        outDist = dx * forward.x() + dy * forward.y();
        if (outDist <= 0.1) return false;

        double xOff = dx * right.x() + dy * right.y();

        double K_x = 5.0 * (w / 32.0);
        outX = w / 2.0 + (xOff / outDist) * K_x;

        double K_y = (4.0 / (outDist + 1.0)) * (h / 12.0);
        outY = h / 2.0 + (drone->z - wz) * K_y;
        return true;
    }

    int m_droneId;
    SimulationEngine* m_engine;
};

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    auto* centralWidget = new QWidget(this);
    auto* mainLayout = new QVBoxLayout(centralWidget);

    // --- 0. Remote Camera Views (Top) ---
    m_cameraScroll = new QScrollArea(this);
    m_cameraScroll->setWidgetResizable(true);
    m_cameraScroll->setFixedHeight(160);
    m_cameraScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    m_cameraScroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_cameraScrollContent = new QWidget(this);
    m_cameraLayout = new QHBoxLayout(m_cameraScrollContent);
    m_cameraLayout->setContentsMargins(5, 5, 5, 5);
    m_cameraLayout->setAlignment(Qt::AlignLeft);
    m_cameraScroll->setWidget(m_cameraScrollContent);
    mainLayout->addWidget(m_cameraScroll);

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

    // Map Presets and Custom Dimensions UI
    auto* dimLayout = new QHBoxLayout();
    m_mapPresetCombo = new QComboBox(this);
    m_mapPresetCombo->addItem("Small (250x250)", 250);
    m_mapPresetCombo->addItem("Medium (500x500)", 500);
    m_mapPresetCombo->addItem("Large (1000x1000)", 1000);
    m_mapPresetCombo->addItem("Custom", -1);
    m_mapPresetCombo->setCurrentIndex(1); // Default to Medium

    // Ensure the dropdown closes and loses focus after a selection
    connect(m_mapPresetCombo, QOverload<int>::of(&QComboBox::activated), [this](int) {
        m_mapPresetCombo->clearFocus();
    });

    m_mapWidthSpin = new QSpinBox(this);
    m_mapWidthSpin->setRange(50, 2000);
    m_mapWidthSpin->setValue(500);

    m_mapHeightSpin = new QSpinBox(this);
    m_mapHeightSpin->setRange(50, 2000);
    m_mapHeightSpin->setValue(500);

    dimLayout->addWidget(new QLabel("Preset:", this));
    dimLayout->addWidget(m_mapPresetCombo);
    dimLayout->addWidget(new QLabel("Width:", this));
    dimLayout->addWidget(m_mapWidthSpin);
    dimLayout->addWidget(new QLabel("Height:", this));
    dimLayout->addWidget(m_mapHeightSpin);
    configLayout->addLayout(dimLayout);

    // Obstacle Counts UI
    auto* obsLayout = new QHBoxLayout();
    m_staticObsSpin = new QSpinBox(this);
    m_staticObsSpin->setRange(0, 200);
    m_staticObsSpin->setValue(10);

    m_dynamicObsSpin = new QSpinBox(this);
    m_dynamicObsSpin->setRange(0, 100);
    m_dynamicObsSpin->setValue(5);

    obsLayout->addWidget(new QLabel("Static Obstacles:", this));
    obsLayout->addWidget(m_staticObsSpin);
    obsLayout->addWidget(new QLabel("Dynamic Obstacles:", this));
    obsLayout->addWidget(m_dynamicObsSpin);
    configLayout->addLayout(obsLayout);

    // Sync presets with width/height spinboxes
    connect(m_mapPresetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        int size = m_mapPresetCombo->itemData(index).toInt();
        if (size > 0) {
            QSignalBlocker blockerW(m_mapWidthSpin);
            QSignalBlocker blockerH(m_mapHeightSpin);
            m_mapWidthSpin->setValue(size);
            m_mapHeightSpin->setValue(size);
        }
    });

    auto setToCustom = [this]() {
        int customIdx = m_mapPresetCombo->findData(-1);
        if (customIdx != -1 && m_mapPresetCombo->currentIndex() != customIdx) {
            QSignalBlocker blocker(m_mapPresetCombo);
            m_mapPresetCombo->setCurrentIndex(customIdx);
        }
    };

    connect(m_mapWidthSpin, QOverload<int>::of(&QSpinBox::valueChanged), setToCustom);
    connect(m_mapHeightSpin, QOverload<int>::of(&QSpinBox::valueChanged), setToCustom);

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
        int width = m_mapWidthSpin->value();
        int height = m_mapHeightSpin->value();
        int numStatic = m_staticObsSpin->value();
        int numDynamic = m_dynamicObsSpin->value();
        m_engine.startSimulation(seed, landProp, width, height, numStatic, numDynamic);
        
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
    updateCameraViews();
    setCentralWidget(centralWidget);
}

void MainWindow::updateCameraViews() {
    QLayoutItem* child;
    while ((child = m_cameraLayout->takeAt(0)) != nullptr) {
        if (child->widget()) child->widget()->deleteLater();
        delete child;
    }

    if (m_selectedIds.empty()) {
        m_cameraLayout->setAlignment(Qt::AlignCenter);
        auto* placeholder = new QLabel("Select one or more deployed drones to view live camera feeds", m_cameraScrollContent);
        placeholder->setAlignment(Qt::AlignCenter);
        placeholder->setStyleSheet("color: #7f8c8d; font-size: 13px; font-weight: bold;");
        m_cameraLayout->addWidget(placeholder);
    } else {
        m_cameraLayout->setAlignment(Qt::AlignLeft);
        for (int id : m_selectedIds) {
            auto* cam = new DroneCameraWidget(id, &m_engine, this);
            m_cameraLayout->addWidget(cam);
        }
    }
}

void MainWindow::updateButtonStates() {
    bool hasSelection = !m_selectedIds.empty();
    m_clearQueueBtn->setEnabled(hasSelection);
    m_landBtn->setEnabled(hasSelection);
    m_takeOffBtn->setEnabled(hasSelection);
    m_rtbBtn->setEnabled(hasSelection);
}