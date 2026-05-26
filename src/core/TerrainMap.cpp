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

TerrainMap::TerrainMap(unsigned int seed, double landProp, int width, int height, double cellSize)
    : m_width(width), m_height(height), m_cellSize(cellSize), m_permutation(512), m_staticObstacles(), m_landProp(landProp) {
    // Initialize permutation table for Perlin noise
    m_permutation.resize(256);
    std::iota(m_permutation.begin(), m_permutation.end(), 0);
    std::default_random_engine engine(seed); 
    std::shuffle(m_permutation.begin(), m_permutation.end(), engine);
    m_permutation.insert(m_permutation.end(), m_permutation.begin(), m_permutation.end());

    // Procedural Static Obstacle Spawning
    std::srand(seed + 1); 
    const double halfWorldWidth = (m_width * m_cellSize) * 0.5;
    const double halfWorldHeight = (m_height * m_cellSize) * 0.5;
    const double helipadMargin = std::min(halfWorldWidth, halfWorldHeight) * 0.2;

    for (int i = 0; i < 10; ++i) {
        double ox, oy, radius;
        bool valid;
        int attempts = 0;
        do {
            valid = true;
            ox = (std::rand() / static_cast<double>(RAND_MAX)) * (2.0 * halfWorldWidth) - halfWorldWidth;
            oy = (std::rand() / static_cast<double>(RAND_MAX)) * (2.0 * halfWorldHeight) - halfWorldHeight;
            radius = 3.0 + (std::rand() % 500) / 100.0;

            // 1. Helipad Safety Zone: Keep buildings away from the central area
            if (std::abs(ox) < helipadMargin && std::abs(oy) < helipadMargin) {
                valid = false;
            }

            // 2. Obstacle-to-Obstacle overlap check
            if (valid) {
                for (const auto& existing : m_staticObstacles) {
                    double dx = ox - existing.x;
                    double dy = oy - existing.y;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    // Maintain a minimum separation based on radii plus a safety buffer
                    if (dist < (radius + existing.radius + 5.0)) {
                        valid = false;
                        break;
                    }
                }
            }
        } while (!valid && ++attempts < 50);

        double height = 10.0 + (std::rand() % 20);
        m_staticObstacles.push_back({i, ox, oy, radius, height});
    }

    // Optimized Proportion Calculation:
    // Perlin noise is smooth and spatially correlated. Sampling every 4th pixel (stride 4)
    // provides a highly accurate distribution while performing 1/16th of the calculations.
    const int sampleStride = 4;
    std::vector<double> samples;
    samples.reserve((m_width / sampleStride) * (m_height / sampleStride));

    for (int y = 0; y < m_height; y += sampleStride) {
        for (int x = 0; x < m_width; x += sampleStride) {
            samples.push_back(calculateRawNoise(x * m_cellSize, y * m_cellSize));
        }
    }

    // Use std::nth_element for O(N) average-time selection instead of O(N log N) sorting.
    size_t thresholdIndex = static_cast<size_t>((1.0 - landProp) * (samples.size() - 1));
    thresholdIndex = std::clamp(thresholdIndex, size_t(0), samples.size() - 1);

    std::nth_element(samples.begin(), samples.begin() + thresholdIndex, samples.end());
    m_waterThreshold = samples[thresholdIndex];
}

double TerrainMap::calculateRawNoise(double x, double y) const {
    double freq = 0.02;
    double amp = 20.0;
    double height = 0;

    // Fractal noise: Sum multiple octaves to create hills and valleys
    height += getNoiseValue(x * freq, y * freq, 0.5) * amp;
    height += getNoiseValue(x * freq * 2.1, y * freq * 2.1, 0.1) * (amp * 0.5);
    height += getNoiseValue(x * freq * 4.3, y * freq * 4.3, 0.9) * (amp * 0.25);
    return height;
}

double TerrainMap::getNoiseValue(double nx, double ny, double nz) const {
    int X = (int)std::floor(nx) & 255;
    int Y = (int)std::floor(ny) & 255;
    int Z = (int)std::floor(nz) & 255;
    nx -= std::floor(nx); ny -= std::floor(ny); nz -= std::floor(nz);
    double u = fade(nx); double v = fade(ny); double w = fade(nz);
    int a = m_permutation[X] + Y, aa = m_permutation[a] + Z, ab = m_permutation[a + 1] + Z;
    int b = m_permutation[X + 1] + Y, ba = m_permutation[b] + Z, bb = m_permutation[b + 1] + Z;
    return lerp(w, lerp(v, lerp(u, grad(m_permutation[aa], nx, ny, nz), grad(m_permutation[ba], nx - 1, ny, nz)),
                           lerp(u, grad(m_permutation[ab], nx, ny - 1, nz), grad(m_permutation[bb], nx - 1, ny - 1, nz))),
                   lerp(v, lerp(u, grad(m_permutation[aa + 1], nx, ny, nz - 1), grad(m_permutation[ba + 1], nx - 1, ny, nz - 1)),
                           lerp(u, grad(m_permutation[ab + 1], nx, ny - 1, nz - 1), grad(m_permutation[bb + 1], nx - 1, ny - 1, nz - 1))));
}

double TerrainMap::getHeightAt(double x, double y) const {
    // Normalize the noise so that 1.0 is the shoreline.
    // Anything > 1.0 is land, anything < 1.0 is water.
    double h = calculateRawNoise(x, y);
    return std::max(0.0, h - m_waterThreshold + 1.0);
}

int TerrainMap::getWidth() const { return m_width; }
int TerrainMap::getHeight() const { return m_height; }
double TerrainMap::getCellSize() const { return m_cellSize; }
double TerrainMap::getWorldWidth() const { return m_width * m_cellSize; }
double TerrainMap::getWorldHeight() const { return m_height * m_cellSize; }
const std::vector<StaticObstacle>& TerrainMap::getObstacles() const {
    return m_staticObstacles;
}