#include "TelemetryModel.h"
#include <QBrush>
#include <QFont>

TelemetryModel::TelemetryModel(SimulationEngine* engine, DroneListType type, QObject* parent) 
    : QAbstractTableModel(parent), m_engine(engine), m_type(type) {
    if (m_type == DroneListType::Deployed) m_cachedDrones = m_engine->getDroneData();
    else m_cachedDrones = m_engine->getInventoryData();
}

int TelemetryModel::rowCount(const QModelIndex&) const { 
    return m_cachedDrones.size(); 
}
int TelemetryModel::columnCount(const QModelIndex&) const { 
    return 7; // Increased column count to include Proximity Alert
}

QVariant TelemetryModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid()) return QVariant();

    const auto& drone = m_cachedDrones[index.row()];

    if (role == Qt::BackgroundRole) {
        if (drone.status == DroneStatus::Crashed) return QBrush(QColor(255, 200, 200)); // Light Red

        double groundHeight = m_engine->getTerrain().getHeightAt(drone.x, drone.y);
        double altitudeAGL = std::max(0.0, drone.z - groundHeight);
        double t_fast = std::max(0.0, (altitudeAGL - 5.0) / 5.0);
        double t_slow = std::min(altitudeAGL, 5.0) / 1.5;
        double batteryRequiredToLand = (t_fast * 2.0) + (t_slow * 1.3) + 5.0;

        if (drone.batteryLevel < batteryRequiredToLand) return QBrush(QColor(255, 230, 150)); // Light Orange/Yellow
        return QVariant();
    }

    if (role == Qt::FontRole && m_selectedIds.count(drone.id)) {
        QFont boldFont;
        boldFont.setBold(true);
        return boldFont;
    }

    if (role != Qt::DisplayRole) return QVariant();
    
    switch (index.column()) {
        case 0: return (m_selectedIds.count(drone.id) ? "→ " : "") + QString::number(drone.id);
        case 1: return QString::number(drone.x, 'f', 2); // X position
        case 2: return QString::number(drone.y, 'f', 2); // Y position
        case 3: return QString::number(drone.z, 'f', 2); // Z position (altitude)
        case 4: return QString::number(drone.batteryLevel, 'f', 1) + "%"; // Battery Level
        case 5: // Drone Status
            if (m_type == DroneListType::Hangar) return "In hangar, charging";

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
                case DroneStatus::Landing: return "Landing (User)";
                case DroneStatus::EmergencyLanding: return "Landing (Emergency)";
                case DroneStatus::Landed: return "Landed (Charging)";
                default: return "Unknown";
            }
        case 6: return drone.proximityAlert ? "⚠️ NEAR OBJECT" : "Clear";
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
        case 6: return "Proximity";
        default: return QVariant();
    }
}

void TelemetryModel::updateModel() {
    std::vector<Drone> newDrones;
    if (m_type == DroneListType::Deployed) newDrones = m_engine->getDroneData();
    else newDrones = m_engine->getInventoryData();

    // If the number of drones changed, we must reset the model to update the view's row count.
    // Otherwise, use dataChanged to preserve selection state and scroll position.
    if (newDrones.size() != m_cachedDrones.size()) {
        beginResetModel();
        m_cachedDrones = newDrones;
        endResetModel();
    } else if (!m_cachedDrones.empty()) {
        m_cachedDrones = newDrones;
        emit dataChanged(index(0, 0), index(rowCount() - 1, columnCount() - 1));
    }
}

void TelemetryModel::setSelectedIds(const std::set<int>& ids) {
    m_selectedIds = ids;
    emit dataChanged(index(0, 0), index(rowCount() - 1, columnCount() - 1));
}

int TelemetryModel::getDroneIdAt(int row) const {
    if (row < 0 || row >= (int)m_cachedDrones.size()) return -1;
    return m_cachedDrones[row].id;
}