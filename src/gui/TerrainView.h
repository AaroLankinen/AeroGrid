#pragma once
#include <QWidget>
#include <QImage>
#include <QPaintEvent>
#include "SimulationEngine.h"

class TerrainView : public QWidget {
    Q_OBJECT
public:
    explicit TerrainView(SimulationEngine* engine, QWidget* parent = nullptr);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;

signals:
    void droneSelected(int id);
    void mapTargetSet(double worldX, double worldY);

private:
    void renderTerrainCache();
    QImage m_terrainCache;
    SimulationEngine* m_engine;
    int m_selectedDroneId = -1;
};