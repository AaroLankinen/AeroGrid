#include "TelemetryModel.h"
#include <QBrush>
#include <QFont>
#include "Constants.h"

TelemetryModel::TelemetryModel(SimulationEngine* engine, DroneListType type, QObject* parent) 
    : QAbstractTableModel(parent), m_engine(engine), m_type(type) {
    m_sortColumn = -1;
    m_sortOrder = Qt::AscendingOrder;
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
        if (drone.status == DroneStatus::Crashed) return QBrush(AeroGrid::UI::COLOR_TABLE_CRASHED); 
        if (drone.status == DroneStatus::Disconnected) return QBrush(AeroGrid::UI::COLOR_TABLE_DISCONNECTED); 

        double groundHeight = m_engine->getTerrain().getHeightAt(drone.x, drone.y);
        double batteryRequiredToLand = drone.calculateBatteryRequiredToLand(groundHeight);

        if (drone.batteryLevel < batteryRequiredToLand) return QBrush(AeroGrid::UI::COLOR_TABLE_LOW_BATTERY); 
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

    // Re-apply current sort parameters to the new data set
    if (m_sortColumn != -1) {
        std::sort(newDrones.begin(), newDrones.end(), [this](const Drone& a, const Drone& b) {
            bool less = false;
            switch (m_sortColumn) {
                case 0: less = a.id < b.id; break;
                case 1: less = a.x < b.x; break;
                case 2: less = a.y < b.y; break;
                case 3: less = a.z < b.z; break;
                case 4: less = a.batteryLevel < b.batteryLevel; break;
                case 5: less = static_cast<int>(a.status) < static_cast<int>(b.status); break;
                case 6: less = a.proximityAlert < b.proximityAlert; break;
                default: return false;
            }
            return m_sortOrder == Qt::AscendingOrder ? less : !less;
        });
    }

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

void TelemetryModel::sort(int column, Qt::SortOrder order) {
    m_sortColumn = column;
    m_sortOrder = order;
    updateModel();
}

void TelemetryModel::setSelectedIds(const std::set<int>& ids) {
    m_selectedIds = ids;
    if (rowCount() > 0) {
        emit dataChanged(index(0, 0), index(rowCount() - 1, columnCount() - 1));
    }
}

int TelemetryModel::getDroneIdAt(int row) const {
    if (row < 0 || row >= (int)m_cachedDrones.size()) return -1;
    return m_cachedDrones[row].id;
}