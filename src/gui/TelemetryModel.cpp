#include "TelemetryModel.h"
#include <QBrush>

TelemetryModel::TelemetryModel(SimulationEngine* engine, QObject* parent) 
    : QAbstractTableModel(parent), m_engine(engine) {}

int TelemetryModel::rowCount(const QModelIndex&) const { 
    return m_cachedDrones.size(); 
}
int TelemetryModel::columnCount(const QModelIndex&) const { 
    return 6; // Increased column count to include Status 
}

QVariant TelemetryModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid()) return QVariant();

    const auto& drone = m_cachedDrones[index.row()];

    if (role == Qt::BackgroundRole) {
        if (drone.status == DroneStatus::Crashed) return QBrush(QColor(255, 200, 200)); // Light Red
        if (drone.batteryLevel < 20.0) return QBrush(QColor(255, 230, 150)); // Light Orange/Yellow
        return QVariant();
    }

    if (role != Qt::DisplayRole) return QVariant();
    
    switch (index.column()) {
        case 0: return drone.id; // Drone ID
        case 1: return QString::number(drone.x, 'f', 2); // X position
        case 2: return QString::number(drone.y, 'f', 2); // Y position
        case 3: return QString::number(drone.z, 'f', 2); // Z position (altitude)
        case 4: return QString::number(drone.batteryLevel, 'f', 1) + "%"; // Battery Level
        case 5: // Drone Status
            // Convert enum to human-readable string
            switch (drone.status) {
                case DroneStatus::Flying: 
                    if (!drone.navQueue.empty()) {
                        const auto& next = drone.navQueue.front();
                        return QString("Moving to (%1, %2)")
                            .arg(next.x, 0, 'f', 1).arg(next.y, 0, 'f', 1);
                    }
                    return "Maintaining position";
                case DroneStatus::Crashed: return "Crashed";
                case DroneStatus::Disconnected: return "Disconnected";
                case DroneStatus::Landing: return "Landing...";
                case DroneStatus::Landed: return "Landed (Charging)";
                default: return "Unknown";
            }
        default: return QVariant();
    }
}

QVariant TelemetryModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (role != Qt::DisplayRole || orientation != Qt::Horizontal) return QVariant();
    switch (section) {
        case 0: return "Drone ID";
        case 1: return "X"; case 2: return "Y"; case 3: return "Z";
        case 4: return "Battery"; // Battery Level
        case 5: return "Status"; // New header for Status
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