#include "TerrainView.h"
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QtGlobal>
#include <cmath>

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
            
            // Procedural Interpretation: Low heights represent water features
            if (height < 1.0) {
                // Water (Lakes / Rivers)
                int blueVal = qBound(150, 200 + static_cast<int>(height * 20), 255);
                m_terrainCache.setPixel(x, y, qRgb(20, 60, blueVal));
            } else {
                // Land (Plains to Peaks)
                int val = qBound(0, static_cast<int>((height - 1.0) * 12), 255);
                m_terrainCache.setPixel(x, y, qRgb(val, 150 + val / 2, 50));
            }
        }
    }
}

void TerrainView::screenToWorld(double screenX, double screenY, double& outWorldX, double& outWorldY) const {
    const auto& terrain = m_engine->getTerrain();
    
    // Convert screen pixels to cache pixels using cached view parameters
    double cachePx = m_cachedSrcX + (screenX / width()) * m_cachedViewPixelsX;
    double cachePy = m_cachedSrcY + (screenY / height()) * m_cachedViewPixelsY;
    
    // Convert cache pixels to world coordinates
    outWorldX = (cachePx - terrain.getWidth() / 2.0) * terrain.getCellSize();
    outWorldY = (cachePy - terrain.getHeight() / 2.0) * terrain.getCellSize();
}

void TerrainView::worldToScreen(double worldX, double worldY, double& outScreenX, double& outScreenY) const {
    const auto& terrain = m_engine->getTerrain();
    
    // Convert world coordinates to cache pixels
    double cachePx = worldX / terrain.getCellSize() + terrain.getWidth() / 2.0;
    double cachePy = worldY / terrain.getCellSize() + terrain.getHeight() / 2.0;
    
    // Convert cache pixels to screen pixels using cached view parameters
    outScreenX = (cachePx - m_cachedSrcX) / m_cachedViewPixelsX * width();
    outScreenY = (cachePy - m_cachedSrcY) / m_cachedViewPixelsY * height();
}

void TerrainView::setSelectedIds(const std::set<int>& ids) {
    m_selectedIds = ids;
    update();
}

void TerrainView::mousePressEvent(QMouseEvent* event) {
    double worldX, worldY;
    screenToWorld(event->position().x(), event->position().y(), worldX, worldY);
    bool ctrlPressed = event->modifiers() & Qt::ControlModifier;

    if (event->button() == Qt::LeftButton) {
        auto drones = m_engine->getDroneData();
        
        // First check if user clicked on a navigation point of a drone
        for (const auto& drone : drones) {
            for (int i = 0; i < (int)drone.navQueue.size(); ++i) {
                const auto& pt = drone.navQueue[i];
                double dx = pt.x - worldX;
                double dy = pt.y - worldY;
                if (std::sqrt(dx*dx + dy*dy) < 3.0) { // 3m hit box for path points
                    emit navPointClicked(drone.id, i);
                    return;
                }
            }
        }

        bool droneHit = false;
        for (const auto& drone : drones) {
            double dx = drone.x - worldX;
            double dy = drone.y - worldY;
            if (std::sqrt(dx*dx + dy*dy) < 5.0) { // 5m click radius
                if (ctrlPressed) {
                    if (m_selectedIds.count(drone.id)) m_selectedIds.erase(drone.id);
                    else m_selectedIds.insert(drone.id);
                } else {
                    m_selectedIds = {drone.id};
                }
                emit dronesSelected(m_selectedIds);
                droneHit = true;
                break;
            }
        }

        if (!droneHit) {
            if (!ctrlPressed) {
                m_selectedIds.clear();
                emit dronesSelected(m_selectedIds);
            }
            m_origin = event->pos();
            if (!m_rubberBand) m_rubberBand = new QRubberBand(QRubberBand::Rectangle, this);
            m_rubberBand->setGeometry(QRect(m_origin, QSize()));
            m_rubberBand->show();
        }
    } else if (event->button() == Qt::RightButton) {
        emit mapTargetSet(worldX, worldY);
    }
    update();
}

void TerrainView::mouseMoveEvent(QMouseEvent* event) {
    if (m_rubberBand) m_rubberBand->setGeometry(QRect(m_origin, event->pos()).normalized());
}

void TerrainView::mouseReleaseEvent(QMouseEvent* event) {
    if (m_rubberBand) {
        QRect rect = m_rubberBand->geometry();
        m_rubberBand->hide();
        
        auto drones = m_engine->getDroneData();
        const auto& terrain = m_engine->getTerrain();
        
        auto worldToScreenLambda = [this](double wx, double wy) {
            double sx, sy;
            worldToScreen(wx, wy, sx, sy);
            return QPoint(static_cast<int>(sx), static_cast<int>(sy));
        };

        if (!(event->modifiers() & Qt::ControlModifier)) m_selectedIds.clear();

        for (const auto& drone : drones) {
            if (rect.contains(worldToScreenLambda(drone.x, drone.y))) {
                m_selectedIds.insert(drone.id);
            }
        }
        emit dronesSelected(m_selectedIds);
        delete m_rubberBand;
        m_rubberBand = nullptr;
    }
}

void TerrainView::wheelEvent(QWheelEvent* event) {
    if (!m_engine) return;
    
    const auto& terrain = m_engine->getTerrain();
    
    // Calculate the base scale that makes the terrain fill the widget at zoom 1.0
    double scaleX = static_cast<double>(width()) / terrain.getWidth();
    double scaleY = static_cast<double>(height()) / terrain.getHeight();
    double baseScale = std::min(scaleX, scaleY);
    
    // Dynamically calculate max zoom based on screen resolution and terrain cell size
    // At maximum zoom, a cell should be roughly 4-8 pixels across (readable without being huge)
    double cellPixelsAtBaseScale = terrain.getCellSize() * baseScale;
    double maxZoomDynamic = std::max(4.0, cellPixelsAtBaseScale / 4.0);  // 4 pixels per cell minimum
    
    // Get cursor position in world coordinates before zoom
    double cursorWorldX, cursorWorldY;
    screenToWorld(event->position().x(), event->position().y(), cursorWorldX, cursorWorldY);
    
    // Update zoom level with dynamic limits
    double oldZoom = m_zoomLevel;
    if (event->angleDelta().y() > 0) {
        m_zoomLevel = std::min(maxZoomDynamic, m_zoomLevel * ZOOM_FACTOR);
    } else {
        m_zoomLevel = std::max(MIN_ZOOM, m_zoomLevel / ZOOM_FACTOR);
    }
    
    // Only adjust pan if zoom actually changed
    if (m_zoomLevel != oldZoom) {
        // Adjust pan so the world point under the cursor stays at the cursor position
        double newCursorWorldX, newCursorWorldY;
        screenToWorld(event->position().x(), event->position().y(), newCursorWorldX, newCursorWorldY);
        
        m_panX += cursorWorldX - newCursorWorldX;
        m_panY += cursorWorldY - newCursorWorldY;
        
        update();
    }
    
    event->accept();
}

void TerrainView::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    const auto& terrain = m_engine->getTerrain();
    
    // Calculate the base scale that makes the terrain fill the widget at zoom 1.0
    double scaleX = static_cast<double>(width()) / terrain.getWidth();
    double scaleY = static_cast<double>(height()) / terrain.getHeight();
    double baseScale = std::min(scaleX, scaleY);  // Maintain aspect ratio
    
    // Effective scale with zoom applied
    double effectiveScale = baseScale * m_zoomLevel;
    
    // Calculate which part of the terrain cache to display
    // At zoom 1.0, we see the full cache; at zoom 2.0, we see 1/2 the cache, etc.
    double viewCachePixelsX = width() / effectiveScale;
    double viewCachePixelsY = height() / effectiveScale;
    
    // Clamp to terrain bounds
    viewCachePixelsX = std::min(viewCachePixelsX, static_cast<double>(terrain.getWidth()));
    viewCachePixelsY = std::min(viewCachePixelsY, static_cast<double>(terrain.getHeight()));
    
    // Calculate center of view in terrain cache pixels, adjusted by pan
    double centerCacheX = terrain.getWidth() / 2.0 + m_panX / (terrain.getCellSize() * baseScale);
    double centerCacheY = terrain.getHeight() / 2.0 + m_panY / (terrain.getCellSize() * baseScale);
    
    // Calculate source rectangle in terrain cache
    m_cachedSrcX = centerCacheX - viewCachePixelsX / 2.0;
    m_cachedSrcY = centerCacheY - viewCachePixelsY / 2.0;
    
    // Clamp to terrain bounds
    if (m_cachedSrcX < 0) m_cachedSrcX = 0;
    if (m_cachedSrcY < 0) m_cachedSrcY = 0;
    if (m_cachedSrcX + viewCachePixelsX > terrain.getWidth()) 
        m_cachedSrcX = terrain.getWidth() - viewCachePixelsX;
    if (m_cachedSrcY + viewCachePixelsY > terrain.getHeight()) 
        m_cachedSrcY = terrain.getHeight() - viewCachePixelsY;
    
    // Cache these for coordinate conversion
    m_cachedViewPixelsX = viewCachePixelsX;
    m_cachedViewPixelsY = viewCachePixelsY;
    m_cachedBaseScale = baseScale;
    
    QRectF sourceRect(m_cachedSrcX, m_cachedSrcY, viewCachePixelsX, viewCachePixelsY);
    QRectF targetRect(0, 0, width(), height());
    
    painter.drawImage(targetRect, m_terrainCache, sourceRect);

    // Helper lambda to convert world coordinates to screen coordinates
    auto worldToScreenOnScreen = [&](double wx, double wy) {
        // Convert world to terrain cache pixels
        double cachePx = wx / terrain.getCellSize() + terrain.getWidth() / 2.0;
        double cachePy = wy / terrain.getCellSize() + terrain.getHeight() / 2.0;
        
        // Convert cache pixels to screen pixels
        double screenX = (cachePx - m_cachedSrcX) / viewCachePixelsX * width();
        double screenY = (cachePy - m_cachedSrcY) / viewCachePixelsY * height();
        
        return QPointF(screenX, screenY);
    };

    // Draw Base (Helipad)
    QPointF basePos = worldToScreenOnScreen(m_engine->getBaseX(), m_engine->getBaseY());
    painter.setPen(QPen(Qt::white, 2));
    painter.setBrush(Qt::darkGray);
    painter.drawEllipse(basePos, 15, 15);
    painter.drawText(QRectF(basePos.x()-10, basePos.y()-10, 20, 20), Qt::AlignCenter, "H");

    // Draw Static Obstacles (High Contrast Red)
    painter.setBrush(QColor(255, 0, 0, 180)); 
    painter.setPen(QPen(Qt::black, 1));
    for (const auto& obs : terrain.getObstacles()) {
        QPointF screenPos = worldToScreenOnScreen(obs.x, obs.y);
        float screenRadius = (obs.radius / terrain.getCellSize()) / viewCachePixelsX * width();
        painter.drawEllipse(screenPos, screenRadius, screenRadius);
    }

    // Draw Dynamic Obstacles (High Contrast Orange)
    painter.setBrush(QColor(255, 140, 0, 200)); 
    painter.setPen(QPen(Qt::black, 1));
    for (const auto& obs : m_engine->getDynamicObstacleData()) {
        QPointF screenPos = worldToScreenOnScreen(obs.x, obs.y);
        float screenRadius = (obs.radius / terrain.getCellSize()) / viewCachePixelsX * width();
        painter.drawEllipse(screenPos, screenRadius, screenRadius);
    }

    // Draw drones at their relative positions
    auto drones = m_engine->getDroneData();
    for (const auto& drone : drones) {
        QPointF dronePos = worldToScreenOnScreen(drone.x, drone.y);

        // Draw Navigation Path
        if (!drone.navQueue.empty()) {
            QPen pathPen(Qt::white, 1, Qt::DashLine);
            painter.setPen(pathPen);
            
            QPointF lastPt = dronePos;
            for (const auto& pt : drone.navQueue) {
                QPointF currentPt = worldToScreenOnScreen(pt.x, pt.y);
                painter.drawLine(lastPt, currentPt);
                
                painter.setBrush(Qt::NoBrush);
                painter.drawEllipse(currentPt, 3, 3); // Target dot
                lastPt = currentPt;
            }
        }

        float px = dronePos.x();
        float py = dronePos.y();

        QColor color = (drone.status == DroneStatus::Crashed) ? Qt::red : Qt::cyan;
        if (m_selectedIds.count(drone.id)) color = Qt::yellow;
        
        painter.setBrush(color);
        painter.setPen(Qt::black);
        painter.drawEllipse(QPointF(px, py), 8, 8);
        
        painter.setPen(Qt::white);
        painter.drawText(px + 8, py + 5, QString("D%1").arg(drone.id));
    }
}