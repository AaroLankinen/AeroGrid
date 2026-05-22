#pragma once
#include <QMainWindow>
#include <QTableView>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <set>
#include "TelemetryModel.h"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private:
    void updateButtonStates();

    SimulationEngine m_engine;
    TelemetryModel* m_model;
    QTableView* m_tableView;

    std::set<int> m_selectedIds;
    QPushButton* m_clearQueueBtn;
    QPushButton* m_landBtn;
    QPushButton* m_takeOffBtn;
    QPushButton* m_rtbBtn;
    QComboBox* m_modeSelector;
    QLabel* m_selectionLabel;
};