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

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

signals:
    void dronesSelected(const std::set<int>& ids);
    void mapTargetSet(double worldX, double worldY);
    void navPointClicked(int droneId, int pointIndex);

private:
    QImage m_terrainCache;
    SimulationEngine* m_engine;
    std::set<int> m_selectedIds;
    QRubberBand* m_rubberBand = nullptr;
    QPoint m_origin;
};