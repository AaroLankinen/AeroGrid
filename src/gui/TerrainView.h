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

private:
    void renderTerrainCache();
    QImage m_terrainCache;
    SimulationEngine* m_engine;
};