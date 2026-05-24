#include "TerrainMap.h"
#include <cmath>
#include <numeric>
#include <random>
#include <algorithm>

namespace {
    // Perlin Noise helper functions
    double fade(double t) { return t * t * t * (t * (t * 6 - 15) + 10); }
    double lerp(double t, double a, double b) { return a + t * (b - a); }
    double grad(int hash, double x, double y, double z) {
        int h = hash & 15;
        double u = h < 8 ? x : y;
        double v = h < 4 ? y : h == 12 || h == 14 ? x : z;
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }
}

TerrainMap::TerrainMap(unsigned int seed) : m_width(200), m_height(200), m_cellSize(1.0), m_permutation(512), m_staticObstacles() {
    // Initialize permutation table for Perlin noise
    m_permutation.resize(256);
    std::iota(m_permutation.begin(), m_permutation.end(), 0);
    std::default_random_engine engine(seed); 
    std::shuffle(m_permutation.begin(), m_permutation.end(), engine);
    m_permutation.insert(m_permutation.end(), m_permutation.begin(), m_permutation.end());

    // Procedural Static Obstacle Spawning
    std::srand(seed + 1); // Use a derived seed for obstacles
    for (int i = 0; i < 10; ++i) {
        double ox = (std::rand() % 160) - 80.0;
        double oy = (std::rand() % 160) - 80.0;
        double radius = 3.0 + (std::rand() % 500) / 100.0;
        double height = 10.0 + (std::rand() % 20);
        
        // Buildings are added with their center coordinates.
        // SimulationEngine uses these along with groundHeight to ensure they rest upon the ground.
        m_staticObstacles.push_back({i, ox, oy, radius, height});
    }
}

double TerrainMap::getHeightAt(double x, double y) const {
    auto getNoise = [&](double nx, double ny, double nz) {
        int X = (int)std::floor(nx) & 255;
        int Y = (int)std::floor(ny) & 255;
        int Z = (int)std::floor(nz) & 255;
        nx -= std::floor(nx);
        ny -= std::floor(ny);
        nz -= std::floor(nz);
        double u = fade(nx);
        double v = fade(ny);
        double w = fade(nz);
        int a = m_permutation[X] + Y, aa = m_permutation[a] + Z, ab = m_permutation[a + 1] + Z;
        int b = m_permutation[X + 1] + Y, ba = m_permutation[b] + Z, bb = m_permutation[b + 1] + Z;

        return lerp(w, lerp(v, lerp(u, grad(m_permutation[aa], nx, ny, nz), grad(m_permutation[ba], nx - 1, ny, nz)),
                               lerp(u, grad(m_permutation[ab], nx, ny - 1, nz), grad(m_permutation[bb], nx - 1, ny - 1, nz))),
                       lerp(v, lerp(u, grad(m_permutation[aa + 1], nx, ny, nz - 1), grad(m_permutation[ba + 1], nx - 1, ny, nz - 1)),
                               lerp(u, grad(m_permutation[ab + 1], nx, ny - 1, nz - 1), grad(m_permutation[bb + 1], nx - 1, ny - 1, nz - 1))));
    };

    double freq = 0.02;
    double amp = 20.0;
    double height = 0;
    
    // Fractal noise: Sum multiple octaves to create hills and valleys
    height += getNoise(x * freq, y * freq, 0.5) * amp;
    height += getNoise(x * freq * 2.1, y * freq * 2.1, 0.1) * (amp * 0.5);
    height += getNoise(x * freq * 4.3, y * freq * 4.3, 0.9) * (amp * 0.25);

    // Basin offset: Subtracting creates low areas for lakes and rivers (height < 1.0)
    return std::max(0.0, height - 2.0);
}

int TerrainMap::getWidth() const { return m_width; }
int TerrainMap::getHeight() const { return m_height; }
double TerrainMap::getCellSize() const { return m_cellSize; }
const std::vector<StaticObstacle>& TerrainMap::getObstacles() const {
    return m_staticObstacles;
}