#pragma once
#include <QMainWindow>
#include <QTableView>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QSlider>
#include <QComboBox>
#include <set>
#include <QHBoxLayout>
#include "TelemetryModel.h"

class TerrainView;

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
    QComboBox* m_mapPresetCombo;
    QComboBox* m_modeSelector;
    QLabel* m_selectionLabel;
    QSlider* m_landWaterSlider;
    QLabel* m_landWaterLabel;
};