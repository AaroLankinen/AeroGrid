#pragma once
#include <vector>
#include <cmath>
#include <algorithm>

struct StaticObstacle {
    int id;
    double x, y, radius, height;
};

struct DynamicObstacle {
    int id;
    double x, y, z, vx, vy, vz, radius;
};

class TerrainMap {
public:
    TerrainMap(unsigned int seed = 12345, double landProp = 0.5, int width = 200, int height = 200, double cellSize = 1.0);

    double getHeightAt(double x, double y) const;

    int getWidth() const;
    int getHeight() const;
    double getCellSize() const;
    double getWorldWidth() const;
    double getWorldHeight() const;
    
    const std::vector<StaticObstacle>& getObstacles() const;

private:
    double calculateRawNoise(double x, double y) const;
    double getNoiseValue(double nx, double ny, double nz) const;

    int m_width, m_height;
    double m_cellSize;
    double m_waterThreshold;
    double m_landProp;
    std::vector<int> m_permutation;
    std::vector<StaticObstacle> m_staticObstacles;
};