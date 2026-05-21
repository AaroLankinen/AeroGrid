#pragma once
#include <QMainWindow>
#include <QTableView>
#include <QVBoxLayout>
#include <QPushButton>
#include "TelemetryModel.h"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private:
    SimulationEngine m_engine;
    TelemetryModel* m_model;
    QTableView* m_tableView;
};