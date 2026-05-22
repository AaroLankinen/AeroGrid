#pragma once
#include <vector>
#include <cmath>
#include <algorithm>

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
    }

    double getHeightAt(double x, double y) const {
        int ix = static_cast<int>((x / m_cellSize) + m_width / 2.0);
        int iy = static_cast<int>((y / m_cellSize) + m_height / 2.0);

        if (ix < 0 || ix >= m_width || iy < 0 || iy >= m_height) return 0.0;
        return m_heights[iy * m_width + ix];
    }

private:
    int m_width, m_height;
    double m_cellSize;
    std::vector<double> m_heights;
};