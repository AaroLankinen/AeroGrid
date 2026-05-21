#include "TelemetryModel.h"

TelemetryModel::TelemetryModel(SimulationEngine* engine, QObject* parent) 
    : QAbstractTableModel(parent), m_engine(engine) {}

int TelemetryModel::rowCount(const QModelIndex&) const { return m_cachedDrones.size(); }
int TelemetryModel::columnCount(const QModelIndex&) const { return 5; }

QVariant TelemetryModel::data(const QModelIndex& index, int role) const {
    if (role != Qt::DisplayRole || !index.isValid()) return QVariant();
    
    const auto& drone = m_cachedDrones[index.row()];
    switch (index.column()) {
        case 0: return drone.id;
        case 1: return QString::number(drone.x, 'f', 2);
        case 2: return QString::number(drone.y, 'f', 2);
        case 3: return QString::number(drone.z, 'f', 2);
        case 4: return QString::number(drone.batteryLevel, 'f', 1) + "%";
        default: return QVariant();
    }
}

QVariant TelemetryModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (role != Qt::DisplayRole || orientation != Qt::Horizontal) return QVariant();
    switch (section) {
        case 0: return "Drone ID";
        case 1: return "X"; case 2: return "Y"; case 3: return "Z";
        case 4: return "Battery";
        default: return QVariant();
    }
}

void TelemetryModel::updateModel() {
    // 1. Fetch thread-safe copy from engine
    m_cachedDrones = m_engine->getDroneData();
    // 2. Notify the View that the layout has changed
    beginResetModel();
    endResetModel();
}