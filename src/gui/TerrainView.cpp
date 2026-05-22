#include "TerrainView.h"
#include <QPainter>
#include <QMouseEvent>
#include <QtGlobal>

TerrainView::TerrainView(SimulationEngine* engine, QWidget* parent)
    : QWidget(parent), m_engine(engine) {
    setMinimumSize(400, 400);
    renderTerrainCache();
}

void TerrainView::renderTerrainCache() {
    const auto& terrain = m_engine->getTerrain();
    int w = terrain.getWidth();
    int h = terrain.getHeight();
    
    m_terrainCache = QImage(w, h, QImage::Format_RGB32);
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            // Use world-to-grid mapping
            double worldX = (x - w/2.0) * terrain.getCellSize();
            double worldY = (y - h/2.0) * terrain.getCellSize();
            double height = terrain.getHeightAt(worldX, worldY);
            
            // Heatmap color: Green for low altitude, Brown/White for hills
            int val = qBound(0, static_cast<int>(height * 10), 255);
            m_terrainCache.setPixel(x, y, qRgb(val, 150 + val/2, 50));
        }
    }
}

void TerrainView::mousePressEvent(QMouseEvent* event) {
    const auto& terrain = m_engine->getTerrain();
    double worldX = (static_cast<double>(event->x()) / width() * terrain.getWidth() - terrain.getWidth() / 2.0) * terrain.getCellSize();
    double worldY = (static_cast<double>(event->y()) / height() * terrain.getHeight() - terrain.getHeight() / 2.0) * terrain.getCellSize();

    if (event->button() == Qt::LeftButton) {
        // Selection logic
        auto drones = m_engine->getDroneData();
        m_selectedDroneId = -1;
        for (const auto& drone : drones) {
            double dx = drone.x - worldX;
            double dy = drone.y - worldY;
            if (std::sqrt(dx*dx + dy*dy) < 5.0) { // 5m click radius
                m_selectedDroneId = drone.id;
                emit droneSelected(m_selectedDroneId);
                break;
            }
        }
    } else if (event->button() == Qt::RightButton) {
        emit mapTargetSet(worldX, worldY);
    }
    update();
}

void TerrainView::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Draw the pre-rendered terrain heatmap
    painter.drawImage(rect(), m_terrainCache);

    // Draw drones at their relative positions
    auto drones = m_engine->getDroneData();
    const auto& terrain = m_engine->getTerrain();
    
    for (const auto& drone : drones) {
        float px = (drone.x / terrain.getCellSize() + terrain.getWidth() / 2.0f) * width() / terrain.getWidth();
        float py = (drone.y / terrain.getCellSize() + terrain.getHeight() / 2.0f) * height() / terrain.getHeight();

        QColor color = (drone.status == DroneStatus::Crashed) ? Qt::red : Qt::cyan;
        if (drone.id == m_selectedDroneId) color = Qt::yellow;
        
        painter.setBrush(color);
        painter.setPen(Qt::black);
        painter.drawEllipse(QPointF(px, py), 8, 8);
        
        painter.setPen(Qt::white);
        painter.drawText(px + 8, py + 5, QString("D%1").arg(drone.id));
    }
}