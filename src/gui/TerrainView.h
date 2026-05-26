#pragma once
#include <QWidget>
#include <QImage>
#include <QPaintEvent>
#include <QRubberBand>
#include <set>
#include "SimulationEngine.h"

class TerrainView : public QWidget {
    Q_OBJECT
public:
    explicit TerrainView(SimulationEngine* engine, QWidget* parent = nullptr);
    void setSelectedIds(const std::set<int>& ids);
    void renderTerrainCache();
    void resetView();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

signals:
    void dronesSelected(const std::set<int>& ids);
    void mapTargetSet(double worldX, double worldY);
    void navPointClicked(int droneId, int pointIndex);

private:
    // Helper to convert screen coordinates to world coordinates with zoom/pan applied
    void screenToWorld(double screenX, double screenY, double& outWorldX, double& outWorldY) const;
    // Helper to convert world coordinates to screen coordinates with zoom/pan applied
    void worldToScreen(double worldX, double worldY, double& outScreenX, double& outScreenY) const;
    // Helper to update cached view parameters based on current zoom and pan
    void updateViewMetrics() const;
    void getClampedWorldCenter(double& outCenterX, double& outCenterY) const;
    double getEffectivePixelsPerMeter() const;

    QImage m_terrainCache;
    SimulationEngine* m_engine;
    std::set<int> m_selectedIds;
    QRubberBand* m_rubberBand = nullptr;
    QPoint m_origin;
    
    // Zoom and pan state
    double m_zoomLevel = 1.0;      // 1.0 = no zoom, >1.0 = zoomed in
    double m_panX = 0.0;           // World coordinate offset
    double m_panY = 0.0;
    bool m_panning = false;
    QPoint m_panLastPos;
    static constexpr double MIN_ZOOM = 0.5;
    static constexpr double MAX_ZOOM = 5.0;
    static constexpr double ZOOM_FACTOR = 1.2;  // Multiplier per scroll tick
    
    // Cached view parameters (updated each paint)
    mutable double m_cachedSrcX = 0.0;
    mutable double m_cachedSrcY = 0.0;
    mutable double m_cachedViewPixelsX = 0.0;
    mutable double m_cachedViewPixelsY = 0.0;
    mutable double m_cachedBaseScale = 1.0;
};