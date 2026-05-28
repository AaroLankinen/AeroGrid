#pragma once
#include <QAbstractTableModel>
#include "SimulationEngine.h"
#include <set>

/**
 * @brief Enumerates the two types of drone lists that can be displayed.
 */
enum class DroneListType { 
    Deployed,   ///< Currently active drones in the field.
    Hangar      ///< Drones waiting in the base hangar/inventory.
};

/**
 * @brief Custom Qt table model for displaying real-time drone telemetry.
 * 
 * TelemetryModel provides a bridge between the SimulationEngine (data source)
 * and Qt's view system (UI display). It:
 * - Caches drone data for efficient sorting and filtering
 * - Provides row/column layout for 7 columns of telemetry
 * - Implements color-coded warnings for battery and crash states
 * - Tracks user selection with visual indicators and bold fonts
 * - Supports dynamic sorting by any column
 * 
 * The model displays one of two views:
 * - Deployed: Currently flying drones
 * - Hangar: Drones in inventory, showing "In hangar, charging" status
 * 
 * @section columns Displayed Columns
 * 0. Drone ID (with → arrow if selected)
 * 1. X Position (meters, 2 decimal places)
 * 2. Y Position (meters, 2 decimal places)
 * 3. Z Position/Altitude (meters, 2 decimal places)
 * 4. Battery Level (percentage, 1 decimal place with %)
 * 5. Status (human-readable state or destination)
 * 6. Proximity Alert (⚠️ NEAR OBJECT or Clear)
 * 
 * @section colors Color Coding
 * - Light Red (UI::COLOR_TABLE_CRASHED): Crashed drones
 * - Light Orange (UI::COLOR_TABLE_LOW_BATTERY): Battery level below safe landing 
 *   threshold as calculated by the drone's flight physics.
 * - White/Default: Normal operation
 * 
 * @section selection Selection Tracking
 * Selected drones are displayed with:
 * - A bold font for better visibility
 * - A "→ " prefix in the ID column
 * 
 * @section performance Performance Notes
 * Uses a cached vector of drone data (m_cachedDrones) and only performs model
 * resets when the count changes. Otherwise, uses dataChanged() signals to preserve
 * user scroll position and selection state during high-frequency updates.
 */
class TelemetryModel : public QAbstractTableModel {
    Q_OBJECT
public:
    /**
     * @brief Constructs a TelemetryModel connected to a SimulationEngine.
     * 
     * @param engine Pointer to the SimulationEngine providing drone data.
     *               The model does not own the engine; caller is responsible
     *               for engine lifetime.
     * @param type Which drone list to display (Deployed or Hangar).
     * @param parent Qt parent object for memory management (optional).
     */
    explicit TelemetryModel(SimulationEngine* engine, DroneListType type, QObject* parent = nullptr);

    /**
     * @brief Sorts the drone list by a specific column.
     * 
     * Allows sorting by ID, position (X/Y/Z), battery level, status, or proximity alert.
     * Re-applies sorting automatically when updateModel() is called.
     * 
     * @param column Zero-based column index (0-6).
     * @param order Qt::AscendingOrder or Qt::DescendingOrder.
     */
    void sort(int column, Qt::SortOrder order = Qt::AscendingOrder) override;

    /**
     * @brief Returns the number of drones currently cached (row count).
     * 
     * @param parent Unused parameter (required by Qt interface).
     * @return The number of rows in the table (0 to 12 for deployed, 0 to 12 for hangar).
     */
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    
    /**
     * @brief Returns the number of columns (always 7).
     * 
     * Columns are: ID, X, Y, Z, Battery, Status, Proximity.
     * 
     * @param parent Unused parameter (required by Qt interface).
     * @return Always returns 7.
     */
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    
    /**
     * @brief Retrieves data for display at a specific cell.
     * 
     * Supports multiple roles:
     * - Qt::DisplayRole: Text content for the cell
     * - Qt::FontRole: Font styling (bold for selected drones)
     * - Qt::BackgroundRole: Cell background color (red for crashed, orange for low battery)
     * 
     * @param index The model index (row, column) of the cell.
     * @param role The data role being requested.
     * @return A QVariant containing the appropriate data, or empty QVariant if invalid.
     */
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    
    /**
     * @brief Retrieves header data for columns or rows.
     * 
     * Currently only supports horizontal (column) headers with DisplayRole.
     * 
     * @param section The column index (0-6).
     * @param orientation Qt::Horizontal for columns, Qt::Vertical for rows (unsupported).
     * @param role The data role (typically Qt::DisplayRole).
     * @return The header string for the column, or empty QVariant if invalid.
     */
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

    // Public members for sorting state (accessed by header click connections)
    int m_sortColumn;           ///< Currently sorted column index (-1 = unsorted).
    Qt::SortOrder m_sortOrder;  ///< Current sort order (Ascending or Descending).

public slots:
    /**
     * @brief Refreshes the cached drone data from the engine.
     * 
     * Pulls fresh drone data from the SimulationEngine, re-applies sorting,
     * and emits appropriate change signals.
     * 
     * If the row count changed (drone deployed/returned), performs a full
     * model reset. Otherwise uses dataChanged() to preserve selection and scroll position.
     * 
     * Call this method when the engine emits simulationUpdated() to refresh the view.
     */
    void updateModel();
    
    /**
     * @brief Sets which drones are currently selected by the user.
     * 
     * Selected drones are displayed with bold font and a "→ " prefix in the ID column.
     * Triggers a dataChanged() signal to refresh the view.
     * 
     * @param ids A set of drone IDs to mark as selected. Empty set = no selection.
     */
    void setSelectedIds(const std::set<int>& ids);
    
    /**
     * @brief Retrieves the drone ID at a specific row.
     * 
     * Useful for converting row indices to drone IDs after user selection.
     * 
     * @param row Zero-based row index.
     * @return The drone ID at that row, or -1 if the row is out of bounds.
     */
    int getDroneIdAt(int row) const;

private:
    SimulationEngine* m_engine;                 ///< Pointer to the data source (not owned).
    std::vector<Drone> m_cachedDrones;          ///< Local cache of drone data (for sorting/filtering).
    std::set<int> m_selectedIds;                ///< IDs of currently selected drones.
    DroneListType m_type;                       ///< Whether displaying Deployed or Hangar drones.
};