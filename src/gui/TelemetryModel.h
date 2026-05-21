#pragma once
#include <QAbstractTableModel>
#include "../core/SimulationEngine.h"

class TelemetryModel : public QAbstractTableModel {
    Q_OBJECT
public:
    explicit TelemetryModel(SimulationEngine* engine, QObject* parent = nullptr);

    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

public slots:
    void updateModel(); // Call this when engine emits simulationUpdated

private:
    SimulationEngine* m_engine;
    std::vector<Drone> m_cachedDrones;
};