#pragma once
#include <QMainWindow>
#include <QTableView>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QComboBox>
#include <QSpinBox>
#include <set>
#include <QHBoxLayout>
#include "TelemetryModel.h"

class TerrainView;
class QScrollArea;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private:
    void updateButtonStates();
    void updateCameraViews();

    SimulationEngine m_engine;
    TelemetryModel* m_model;
    TelemetryModel* m_hangarModel;
    QTableView* m_tableView;
    QTableView* m_hangarTableView;
    QWidget* m_simUIContainer;
    QScrollArea* m_cameraScroll;
    QWidget* m_cameraScrollContent;
    QHBoxLayout* m_cameraLayout;
    TerrainView* m_terrainView;

    std::set<int> m_selectedIds;
    std::set<int> m_selectedHangarIds;
    QPushButton* m_launchBtn;
    QPushButton* m_clearQueueBtn;
    QPushButton* m_landBtn;
    QPushButton* m_takeOffBtn;
    QPushButton* m_rtbBtn;
    QComboBox* m_mapPresetCombo;                ///< Combo box for map dimension presets.
    QSpinBox* m_mapWidthSpin;                   ///< Spin box for custom map width.
    QSpinBox* m_mapHeightSpin;                  ///< Spin box for custom map height.
    QSpinBox* m_staticObsSpin;                  ///< Spin box for custom number of static obstacles.
    QSpinBox* m_dynamicObsSpin;                 ///< Spin box for custom number of dynamic obstacles.
    QComboBox* m_modeSelector;                  ///< Dropdown selector for drone flight modes.
    QLabel* m_selectionLabel;                   ///< Label displaying information about the selected drone(s).
    QSlider* m_landWaterSlider;                 ///< Slider to control the land proportion.
    QLabel* m_landWaterLabel;                   ///< Label displaying current land proportion value.
};