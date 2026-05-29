#include "TerrainMap.h"
#include <cmath>
#include <numeric>
#include <random>
#include <algorithm>
#include <QtMath> // For qSqrt
#include "Constants.h"

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

TerrainMap::TerrainMap(unsigned int seed, double landProp, int width, int height, double cellSize, int numStaticObstacles)
    : m_width(width), m_height(height), m_cellSize(cellSize), m_permutation(512), m_staticObstacles(), m_landProp(landProp) {
    initializeNoise(seed);
    calculateWaterThreshold(landProp);
    generateStaticObstacles(seed, numStaticObstacles);
}

void TerrainMap::initializeNoise(unsigned int seed) {
    m_permutation.resize(256);
    std::iota(m_permutation.begin(), m_permutation.end(), 0);
    std::default_random_engine engine(seed); 
    std::shuffle(m_permutation.begin(), m_permutation.end(), engine);
    m_permutation.insert(m_permutation.end(), m_permutation.begin(), m_permutation.end());
}

void TerrainMap::generateStaticObstacles(unsigned int seed, int numStaticObstacles) {
    // Procedural Static Obstacle Spawning
    std::srand(seed + 1); 
    const double halfWorldWidth = (m_width * m_cellSize) * 0.5;
    const double halfWorldHeight = (m_height * m_cellSize) * 0.5;
    const double helipadMargin = std::min(halfWorldWidth, halfWorldHeight) * 0.2;

    int obstacleId = 0;

    while (static_cast<int>(m_staticObstacles.size()) < numStaticObstacles) {
        int remaining = numStaticObstacles - m_staticObstacles.size();
        if (remaining <= 0) break;

        bool valid;
        int attempts = 0;
        std::vector<StaticObstacle> group;

        do {
            valid = true;
            group.clear();

            double ox = (std::rand() / static_cast<double>(RAND_MAX)) * (2.0 * halfWorldWidth) - halfWorldWidth;
            double oy = (std::rand() / static_cast<double>(RAND_MAX)) * (2.0 * halfWorldHeight) - halfWorldHeight;

            // 1. Helipad Safety Zone: Keep building centers away from the central area
            if (std::abs(ox) < helipadMargin && std::abs(oy) < helipadMargin) {
                valid = false;
                continue;
            }

            // Determine type based on remaining slots
            int type = 0;
            if (numStaticObstacles > 1) {
                if (remaining >= 5) {
                    type = std::rand() % 6;
                } else if (remaining >= 3) {
                    type = std::rand() % 5;
                } else if (remaining == 2) {
                    type = std::rand() % 4;
                } else {
                    type = std::rand() % 3;
                }
            }

            if (type == 0) {
                // Cylinder
                double radius = 3.0 + (std::rand() % 500) / 100.0;
                double height = 10.0 + (std::rand() % 25);
                group.push_back({0, ox, oy, radius, height, ObstacleShape::Cylinder, 0.0, 0.0, 0.0});
            } else if (type == 1) {
                // Rectangle
                double width = 6.0 + (std::rand() % 800) / 100.0;
                double depth = 6.0 + (std::rand() % 800) / 100.0;
                double height = 10.0 + (std::rand() % 25);
                double rotation = (std::rand() % 628) / 100.0;
                double radius = qSqrt(width * width + depth * depth) * 0.5;
                group.push_back({0, ox, oy, radius, height, ObstacleShape::Rectangle, width, depth, rotation});
            } else if (type == 2) {
                // Triangle
                double radius = 5.0 + (std::rand() % 700) / 100.0; // Distance to vertices
                double height = 10.0 + (std::rand() % 25);
                double rotation = (std::rand() % 628) / 100.0;
                group.push_back({0, ox, oy, radius, height, ObstacleShape::Triangle, 0.0, 0.0, rotation});
            } else if (type == 3) {
                // L-shape (2 rectangles)
                double S = 10.0 + (std::rand() % 800) / 100.0;
                double height = 12.0 + (std::rand() % 20);
                double rotation = (std::rand() % 628) / 100.0;
                double cosR = qCos(rotation);
                double sinR = qSin(rotation);

                // local R1: (-S/4, 0), width S/2, depth S
                double lx1 = -S * 0.25; double ly1 = 0.0;
                double x1 = ox + lx1 * cosR - ly1 * sinR;
                double y1 = oy + lx1 * sinR + ly1 * cosR;

                // local R2: (0, -S/4), width S, depth S/2
                double lx2 = 0.0; double ly2 = -S * 0.25;
                double x2 = ox + lx2 * cosR - ly2 * sinR;
                double y2 = oy + lx2 * sinR + ly2 * cosR;

                double r1 = qSqrt((S * 0.5) * (S * 0.5) + S * S) * 0.5;
                double r2 = qSqrt(S * S + (S * 0.5) * (S * 0.5)) * 0.5;

                group.push_back({0, x1, y1, r1, height, ObstacleShape::Rectangle, S * 0.5, S, rotation});
                group.push_back({0, x2, y2, r2, height, ObstacleShape::Rectangle, S, S * 0.5, rotation});
            } else if (type == 4) {
                // Step Pyramid (2 rectangles, 1 cylinder spire)
                double S = 12.0 + (std::rand() % 800) / 100.0;
                double height = 15.0 + (std::rand() % 25);
                double rotation = (std::rand() % 628) / 100.0;

                double rBase = S * 0.707;
                double rMid = S * 0.7 * 0.707;
                double rTop = S * 0.2;

                group.push_back({0, ox, oy, rBase, height * 0.4, ObstacleShape::Rectangle, S, S, rotation});
                group.push_back({0, ox, oy, rMid, height * 0.75, ObstacleShape::Rectangle, S * 0.7, S * 0.7, rotation});
                group.push_back({0, ox, oy, rTop, height, ObstacleShape::Cylinder, 0.0, 0.0, 0.0});
            } else {
                // Fortress (1 rectangle, 4 corner cylinders)
                double W = 12.0 + (std::rand() % 600) / 100.0;
                double D = 12.0 + (std::rand() % 600) / 100.0;
                double height = 12.0 + (std::rand() % 15);
                double rotation = (std::rand() % 628) / 100.0;
                double cosR = qCos(rotation);
                double sinR = qSin(rotation);

                double rBase = qSqrt(W * W + D * D) * 0.5;
                group.push_back({0, ox, oy, rBase, height, ObstacleShape::Rectangle, W, D, rotation});

                double rc = qMin(W, D) * 0.15;
                double hc = height * 1.25;

                // local corners:
                double dxs[4] = {-W * 0.5, W * 0.5, W * 0.5, -W * 0.5};
                double dys[4] = {-D * 0.5, -D * 0.5, D * 0.5, D * 0.5};

                for (int c = 0; c < 4; ++c) {
                    double cx = ox + dxs[c] * cosR - dys[c] * sinR;
                    double cy = oy + dxs[c] * sinR + dys[c] * cosR;
                    group.push_back({0, cx, cy, rc, hc, ObstacleShape::Cylinder, 0.0, 0.0, 0.0});
                }
            }

            // 2. Enforce entire footprint is on dry land (groundHeight >= 1.0)
            for (const auto& prim : group) {
                // Check all vertices of this primitive
                auto vertices = prim.getVertices();
                // Also check the center point
                vertices.push_back({prim.x, prim.y});

                for (const auto& pt : vertices) {
                    if (getHeightAt(pt.first, pt.second) < AeroGrid::World::LAND_THRESHOLD) {
                        valid = false;
                        break;
                    }
                }
                if (!valid) break;
            }
            if (!valid) continue;

            // 3. Obstacle-to-Obstacle overlap checks against already spawned obstacles
            for (const auto& prim : group) {
                for (const auto& existing : m_staticObstacles) {
                    double dx = prim.x - existing.x;
                    double dy = prim.y - existing.y;
                    double dist = qSqrt(dx * dx + dy * dy);
                    // Prevent overlaps with existing separate buildings
                    if (dist < (prim.radius + existing.radius + AeroGrid::World::OBSTACLE_SEPARATION_MARGIN)) {
                        valid = false;
                        break;
                    }
                }
                if (!valid) break;
            }

        } while (!valid && ++attempts < AeroGrid::World::OBSTACLE_SPAWN_ATTEMPTS);

        if (valid) {
            for (auto& prim : group) {
                prim.id = obstacleId++;
                m_staticObstacles.push_back(prim);
            }
        } else {
            break;
        }
    }
}

void TerrainMap::calculateWaterThreshold(double landProp) {
    // Optimized Proportion Calculation:
    // Perlin noise is smooth and spatially correlated. Sampling every 4th pixel (stride 4)
    // provides a highly accurate distribution while performing 1/16th of the calculations.
    const int sampleStride = 4;
    std::vector<double> samples;
    samples.reserve((m_width / sampleStride + 1) * (m_height / sampleStride + 1));

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
    return qMax(0.0, h - m_waterThreshold + AeroGrid::World::LAND_THRESHOLD);
}

int TerrainMap::getWidth() const { return m_width; }
int TerrainMap::getHeight() const { return m_height; }
double TerrainMap::getCellSize() const { return m_cellSize; }
double TerrainMap::getWorldWidth() const { return m_width * m_cellSize; }
double TerrainMap::getWorldHeight() const { return m_height * m_cellSize; }
const std::vector<StaticObstacle>& TerrainMap::getObstacles() const {
    return m_staticObstacles;
}