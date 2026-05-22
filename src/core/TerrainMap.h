#pragma once
#include <vector>
#include <cmath>
#include <algorithm>

struct StaticObstacle {
    double x, y, radius, height;
};

struct DynamicObstacle {
    int id;
    double x, y, z, vx, vy, vz, radius;
};

class TerrainMap {
public:
    // Creates a simple map centered at (0,0)
    TerrainMap(int width = 200, int height = 200, double cellSize = 1.0) 
        : m_width(width), m_height(height), m_cellSize(cellSize), m_heights(width * height, 0.0) {
        
        // Generate a central hill for testing
        int cx = width / 2;
        int cy = height / 2;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                double dist = std::sqrt(std::pow(x - cx, 2) + std::pow(y - cy, 2));
                m_heights[y * width + x] = std::max(0.0, 15.0 - dist * 0.4);
            }
        }

        // Add some static obstacles (Buildings/Trees)
        m_obstacles.push_back({-20.0, -20.0, 5.0, 30.0}); // "Building"
        m_obstacles.push_back({30.0, 40.0, 3.0, 40.0});   // "Tower"
        m_obstacles.push_back({10.0, -30.0, 2.0, 10.0});  // "Tree"
        m_obstacles.push_back({-40.0, 10.0, 2.0, 10.0});  // "Tree"
    }

    double getHeightAt(double x, double y) const {
        int ix = static_cast<int>((x / m_cellSize) + m_width / 2.0);
        int iy = static_cast<int>((y / m_cellSize) + m_height / 2.0);

        if (ix < 0 || ix >= m_width || iy < 0 || iy >= m_height) return 0.0;
        return m_heights[iy * m_width + ix];
    }

    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    double getCellSize() const { return m_cellSize; }
    
    const std::vector<StaticObstacle>& getObstacles() const { return m_obstacles; }

private:
    int m_width, m_height;
    double m_cellSize;
    std::vector<double> m_heights;
    std::vector<StaticObstacle> m_obstacles;
};