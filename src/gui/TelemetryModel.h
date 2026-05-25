#pragma once
#include <QAbstractTableModel>
#include "SimulationEngine.h"
#include <set>

enum class DroneListType { Deployed, Hangar };

/**
 * @brief Custom TableModel for displaying real-time drone telemetry.
 * 
 * Features dynamic row styling for warnings and stable selection tracking.
 */
class TelemetryModel : public QAbstractTableModel {
    Q_OBJECT
public:
    explicit TelemetryModel(SimulationEngine* engine, DroneListType type, QObject* parent = nullptr);

    void sort(int column, Qt::SortOrder order = Qt::AscendingOrder) override;

    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

    // Public members for sorting state (accessed by header click connections)
    int m_sortColumn = -1;
    Qt::SortOrder m_sortOrder = Qt::AscendingOrder;

public slots:
    /**
     * @brief Refreshes the local data cache from the engine.
     * 
     * Emits dataChanged() to refresh the view. If the drone count changes,
     * it performs a model reset instead.
     */
    void updateModel(); // Call this when engine emits simulationUpdated
    void setSelectedIds(const std::set<int>& ids);
    int getDroneIdAt(int row) const;

private:
    SimulationEngine* m_engine;
    std::vector<Drone> m_cachedDrones;
    std::set<int> m_selectedIds;
    DroneListType m_type;
};