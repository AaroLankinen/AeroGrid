#include "TerrainView.h"
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QResizeEvent>
#include <QtGlobal>
#include <cmath>
#include "Constants.h"

TerrainView::TerrainView(SimulationEngine* engine, QWidget* parent)
    : QWidget(parent), m_engine(engine) {
    setMinimumSize(200, 200);
    // Maintain square aspect ratio policy
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    renderTerrainCache();
}

void TerrainView::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    // Force the widget width to match its height to maintain a square viewport.
    // This ensures that extra horizontal space in the layout is given to side panels.
    if (height() > 0 && width() != height()) {
        QMetaObject::invokeMethod(this, [this](){
            setFixedWidth(height());
        }, Qt::QueuedConnection);
    }
}

void TerrainView::resetView() {
    m_zoomLevel = 1.0;
    m_panX = 0.0;
    m_panY = 0.0;
    m_panning = false;
    update();
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

void TerrainView::getClampedWorldCenter(double& outCenterX, double& outCenterY) const {
    if (!m_engine) {
        outCenterX = m_panX;
        outCenterY = m_panY;
        return;
    }

    const auto& terrain = m_engine->getTerrain();
    double scaleX = static_cast<double>(width()) / terrain.getWidth();
    double scaleY = static_cast<double>(height()) / terrain.getHeight();
    double baseScale = std::min(scaleX, scaleY);
    double effectivePixelsPerCell = baseScale * m_zoomLevel;

    double viewPixelsX = width() / effectivePixelsPerCell;
    double viewPixelsY = height() / effectivePixelsPerCell;
    double viewWorldWidth = viewPixelsX * terrain.getCellSize();
    double viewWorldHeight = viewPixelsY * terrain.getCellSize();

    const double halfWorldWidth = terrain.getWorldWidth() * 0.5;
    const double halfWorldHeight = terrain.getWorldHeight() * 0.5;

    if (viewWorldWidth >= terrain.getWorldWidth()) {
        outCenterX = 0.0;
    } else {
        outCenterX = qBound(-halfWorldWidth + viewWorldWidth * 0.5,
                            m_panX,
                            halfWorldWidth - viewWorldWidth * 0.5);
    }

    if (viewWorldHeight >= terrain.getWorldHeight()) {
        outCenterY = 0.0;
    } else {
        outCenterY = qBound(-halfWorldHeight + viewWorldHeight * 0.5,
                            m_panY,
                            halfWorldHeight - viewWorldHeight * 0.5);
    }
}

void TerrainView::updateViewMetrics() const {
    if (!m_engine) return;
    const auto& terrain = m_engine->getTerrain();

    // Calculate the base scale that makes the terrain fill the widget at zoom 1.0
    double scaleX = static_cast<double>(width()) / terrain.getWidth();
    double scaleY = static_cast<double>(height()) / terrain.getHeight();
    m_cachedBaseScale = std::min(scaleX, scaleY);  // Maintain aspect ratio

    // Effective scale with zoom applied
    double effectiveScale = m_cachedBaseScale * m_zoomLevel;

    // Recalculate view dimensions in world-space pixels (cells) without clamping the ratio.
    // This ensures that the screen-to-world mapping remains consistent during resizes.
    m_cachedViewPixelsX = width() / effectiveScale;
    m_cachedViewPixelsY = height() / effectiveScale;
}

double TerrainView::getEffectivePixelsPerMeter() const {
    if (!m_engine) return 1.0;
    return (m_cachedBaseScale * m_zoomLevel) / m_engine->getTerrain().getCellSize();
}

void TerrainView::screenToWorld(double screenX, double screenY, double& outWorldX, double& outWorldY) const {
    updateViewMetrics();
    if (!m_engine) return;
    const auto& terrain = m_engine->getTerrain();

    double effectivePixelsPerMeter = getEffectivePixelsPerMeter();
    double clampedCenterX, clampedCenterY;
    getClampedWorldCenter(clampedCenterX, clampedCenterY);

    outWorldX = (screenX - width() / 2.0) / effectivePixelsPerMeter + clampedCenterX;
    outWorldY = (screenY - height() / 2.0) / effectivePixelsPerMeter + clampedCenterY;
}

void TerrainView::worldToScreen(double worldX, double worldY, double& outScreenX, double& outScreenY) const {
    updateViewMetrics();
    if (!m_engine) return;
    double effectivePixelsPerMeter = getEffectivePixelsPerMeter();
    double clampedCenterX, clampedCenterY;
    getClampedWorldCenter(clampedCenterX, clampedCenterY);

    outScreenX = (worldX - clampedCenterX) * effectivePixelsPerMeter + width() / 2.0;
    outScreenY = (worldY - clampedCenterY) * effectivePixelsPerMeter + height() / 2.0;
}

void TerrainView::setSelectedIds(const std::set<int>& ids) {
    m_selectedIds = ids;
    update();
}

bool TerrainView::checkNavPointHit(const QPointF& mousePos, double worldX, double worldY) {
    auto drones = m_engine->getDroneData();
    for (const auto& drone : drones) {
        for (int i = 0; i < (int)drone.navQueue.size(); ++i) {
            const auto& pt = drone.navQueue[i];
            double dx = pt.x - worldX;
            double dy = pt.y - worldY;
            if (std::sqrt(dx*dx + dy*dy) < 3.0) { // 3m hit box
                emit navPointClicked(drone.id, i);
                return true;
            }
        }
    }
    return false;
}

bool TerrainView::checkDroneHit(double worldX, double worldY, bool ctrlPressed) {
    auto drones = m_engine->getDroneData();
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
            return true;
        }
    }
    return false;
}

void TerrainView::mousePressEvent(QMouseEvent* event) {
    double worldX, worldY;
    screenToWorld(event->position().x(), event->position().y(), worldX, worldY);
    bool ctrlPressed = event->modifiers() & Qt::ControlModifier;

    switch (event->button()) {
    case Qt::MiddleButton:
        m_panning = true;
        m_panLastPos = event->pos();
        setCursor(Qt::ClosedHandCursor);
        break;

    case Qt::LeftButton:
        if (checkNavPointHit(event->position(), worldX, worldY)) return;
        if (checkDroneHit(worldX, worldY, ctrlPressed)) break;
        else {
            if (!ctrlPressed) {
                m_selectedIds.clear();
                emit dronesSelected(m_selectedIds);
            }
            m_origin = event->pos();
            if (!m_rubberBand)
                m_rubberBand = new QRubberBand(QRubberBand::Rectangle, this);
            m_rubberBand->setGeometry(QRect(m_origin, QSize()));
            m_rubberBand->show();
        }
        break;

    case Qt::RightButton:
        emit mapTargetSet(worldX, worldY);
        break;

    default:
        break;
    }
    update();
}

void TerrainView::mouseMoveEvent(QMouseEvent* event) {
    if (m_panning) {
        updateViewMetrics();
        double pixelsPerMeter = getEffectivePixelsPerMeter();
        QPoint delta = event->pos() - m_panLastPos;
        m_panLastPos = event->pos();
        if (pixelsPerMeter > 0) {
            m_panX -= delta.x() / pixelsPerMeter;
            m_panY -= delta.y() / pixelsPerMeter;
        }
        update();
        return;
    }

    if (m_rubberBand) m_rubberBand->setGeometry(QRect(m_origin, event->pos()).normalized());
}

void TerrainView::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::MiddleButton) {
        m_panning = false;
        unsetCursor();
        return;
    }
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
    
    // Get cursor position in world coordinates before zoom
    double cursorWorldX, cursorWorldY;
    screenToWorld(event->position().x(), event->position().y(), cursorWorldX, cursorWorldY);

    const auto& terrain = m_engine->getTerrain();
    // Dynamically calculate max zoom based on screen resolution and terrain cell size
    // At maximum zoom, a cell should be roughly 4-8 pixels across (readable without being huge)
    double cellPixelsAtBaseScale = terrain.getCellSize() * m_cachedBaseScale;
    double maxZoomDynamic = std::max(4.0, cellPixelsAtBaseScale / 4.0);  // 4 pixels per cell minimum
    
    // Update zoom level with dynamic limits
    double oldZoom = m_zoomLevel;
    using namespace AeroGrid::UI;
    if (event->angleDelta().y() > 0) {
        m_zoomLevel = std::min(maxZoomDynamic, m_zoomLevel * ZOOM_FACTOR);
    } else {
        m_zoomLevel = std::max(AeroGrid::UI::MIN_ZOOM, m_zoomLevel / ZOOM_FACTOR);
    }
    
    // Only adjust pan if zoom actually changed
    if (m_zoomLevel != oldZoom) {
        // Adjust pan so the world point under the cursor stays at the cursor position
        double newCursorWorldX, newCursorWorldY;
        screenToWorld(event->position().x(), event->position().y(), newCursorWorldX, newCursorWorldY);
        
        m_panX += cursorWorldX - newCursorWorldX;
        m_panY += cursorWorldY - newCursorWorldY;

        // Clamp the pan to ensure view stays within world bounds
        // Don't replace m_panX/Y entirely - that discards the cursor adjustment
        // Instead, we'll let updateViewMetrics() handle clamping dynamically
        // This preserves the cursor-tracking zoom
        
        update();
    }
    
    event->accept();
}

void TerrainView::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    updateViewMetrics();
    const auto& terrain = m_engine->getTerrain();
    const double ppm = getEffectivePixelsPerMeter();
    
    // 1. Draw Terrain Backdrop
    // Instead of stretching a source rect to the widget size, we determine exactly where 
    // the world bounds of the terrain map fall on the current screen.
    double tx0, ty0, tx1, ty1;
    worldToScreen(-terrain.getWorldWidth() / 2.0, -terrain.getWorldHeight() / 2.0, tx0, ty0);
    worldToScreen(terrain.getWorldWidth() / 2.0, terrain.getWorldHeight() / 2.0, tx1, ty1);
    painter.drawImage(QRectF(QPointF(tx0, ty0), QPointF(tx1, ty1)), m_terrainCache);

    // Helper lambda to convert world coordinates to screen coordinates
    auto worldToScreenOnScreen = [&](double wx, double wy) {
        double sx, sy;
        worldToScreen(wx, wy, sx, sy);
        return QPointF(sx, sy);
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
        float screenRadius = static_cast<float>(obs.radius * ppm);
        painter.drawEllipse(screenPos, screenRadius, screenRadius);
    }

    // Draw Dynamic Obstacles (High Contrast Orange)
    painter.setBrush(QColor(255, 140, 0, 200)); 
    painter.setPen(QPen(Qt::black, 1));
    for (const auto& obs : m_engine->getDynamicObstacleData()) {
        QPointF screenPos = worldToScreenOnScreen(obs.x, obs.y);
        float screenRadius = static_cast<float>(obs.radius * ppm);
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