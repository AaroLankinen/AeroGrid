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
    TerrainMap(unsigned int seed = 12345);

    double getHeightAt(double x, double y) const;

    int getWidth() const;
    int getHeight() const;
    double getCellSize() const;
    
    const std::vector<StaticObstacle>& getObstacles() const;

private:
    int m_width, m_height;
    double m_cellSize;
    std::vector<int> m_permutation;
    std::vector<StaticObstacle> m_staticObstacles;
};