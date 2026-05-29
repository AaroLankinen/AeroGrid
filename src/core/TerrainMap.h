#pragma once
#include <vector>
#include <cmath>
#include <algorithm>

enum class ObstacleShape {
    Cylinder,
    Rectangle,
    Triangle
};

/**
 * @brief Represents a static obstacle (building or tower) in the terrain.
 * 
 * Static obstacles are immobile environmental features that drones must avoid.
 * They are generated during terrain initialization and do not move.
 */
struct StaticObstacle {
    int id;                 ///< Unique identifier for the obstacle.
    double x, y;            ///< Center position in the world (meters).
    double radius;          ///< Horizontal radius/footprint (meters).
    double height;          ///< Vertical extent above ground level (meters).
    ObstacleShape shape = ObstacleShape::Cylinder; ///< Geometric primitive shape.
    double width = 0.0;     ///< Dimension width for rectangles.
    double depth = 0.0;     ///< Dimension depth for rectangles.
    double rotation = 0.0;  ///< Rotation in radians.

    std::vector<std::pair<double, double>> getVertices() const {
        std::vector<std::pair<double, double>> vertices;
        if (shape == ObstacleShape::Cylinder) {
            for (int i = 0; i < 8; ++i) {
                double angle = i * (M_PI / 4.0);
                vertices.push_back({x + radius * std::cos(angle), y + radius * std::sin(angle)});
            }
        } else if (shape == ObstacleShape::Rectangle) {
            double hw = width * 0.5;
            double hd = depth * 0.5;
            double cosR = std::cos(rotation);
            double sinR = std::sin(rotation);
            double dxs[4] = {-hw, hw, hw, -hw};
            double dys[4] = {-hd, -hd, hd, hd};
            for (int i = 0; i < 4; ++i) {
                double wx = x + dxs[i] * cosR - dys[i] * sinR;
                double wy = y + dxs[i] * sinR + dys[i] * cosR;
                vertices.push_back({wx, wy});
            }
        } else if (shape == ObstacleShape::Triangle) {
            double cosR = std::cos(rotation);
            double sinR = std::sin(rotation);
            double vxs[3] = {0.0, -radius * 0.866025, radius * 0.866025};
            double vys[3] = {radius, -radius * 0.5, -radius * 0.5};
            for (int i = 0; i < 3; ++i) {
                double wx = x + vxs[i] * cosR - vys[i] * sinR;
                double wy = y + vxs[i] * sinR + vys[i] * cosR;
                vertices.push_back({wx, wy});
            }
        }
        return vertices;
    }
};

/**
 * @brief Represents a dynamic obstacle (bird, other aircraft) in the airspace.
 * 
 * Dynamic obstacles move through the simulation and bounce off world boundaries
 * and altitude limits. Drones actively avoid them during flight.
 */
struct DynamicObstacle {
    int id;                 ///< Unique identifier for the obstacle.
    double x, y, z;         ///< 3D position in the world (meters).
    double vx, vy, vz;      ///< Velocity vector (meters per second).
    double radius;          ///< Collision radius (meters).
};

/**
 * @brief Procedurally generated terrain map for the drone simulation.
 * 
 * TerrainMap generates a realistic landscape using Perlin noise and controls
 * the land/water distribution. It provides:
 * - Height sampling at any coordinate for terrain queries
 * - Static obstacles (buildings) placed on the terrain
 * - World boundaries and dimensions
 * 
 * The terrain is generated once during initialization and remains static
 * throughout the simulation.
 * 
 * @section noise Noise Generation
 * Uses Perlin noise with three octaves of fractal noise combined to create
 * natural-looking hills, valleys, and terrain variation. The noise is seeded
 * for reproducibility and scaled based on the land proportion setting.
 * 
 * @section landWater Land vs Water
 * Heights below 1.0 represent water (impassable). Heights >= 1.0 represent
 * land (passable terrain). Cutoff is determined statistically via calculateWaterThreshold
 * to match the requested landProp.
 * 
 * @section obstacles Obstacle Placement
 * Static obstacles are generated with collision avoidance to prevent overlaps.
 * A helipad safety zone is maintained around the origin to prevent buildings
 * from spawning near the base station.
 */
class TerrainMap {
    friend class TestAeroGrid;
public:
    /**
     * @brief Constructs a procedurally generated terrain map.
     * 
     * Initializes the terrain with Perlin noise and places static obstacles
     * randomly while maintaining separation constraints. This is relatively
     * expensive computationally due to obstacle collision checking.
     * 
     * @param seed Random seed for reproducible terrain generation. Using the
     *             same seed will always produce the same terrain layout.
     * @param landProp Land proportion (0.0 - 1.0). Controls what fraction of
     *                 the world is land vs water. 0.5 = 50/50 split.
     * @param width Terrain grid width in cells (default 200).
     * @param height Terrain grid height in cells (default 200).
     * @param cellSize Size of each grid cell in world units (default 1.0 meter).
     */
    TerrainMap(unsigned int seed = 12345, double landProp = 0.5, int width = 200, int height = 200, double cellSize = 1.0, int numStaticObstacles = 10);

    /**
     * @brief Samples the terrain height at a specific world coordinate.
     * 
     * Returns the elevation (z-axis value) at the given X,Y location.
     * Heights < 1.0 indicate water (uncrossable). Heights >= 1.0 indicate land.
     * 
     * The height is computed by evaluating Perlin noise at the coordinate.
     * Queries are fast O(1) operations after terrain initialization.
     * 
     * @param x World X coordinate in meters.
     * @param y World Y coordinate in meters.
     * @return The terrain height in meters (>= 0.0).
     */
    double getHeightAt(double x, double y) const;

    /**
     * @brief Returns the width of the terrain grid in cells.
     * 
     * @return Width in cells (typically 200).
     */
    int getWidth() const;
    
    /**
     * @brief Returns the height of the terrain grid in cells.
     * 
     * @return Height in cells (typically 200).
     */
    int getHeight() const;
    
    /**
     * @brief Returns the size of each terrain cell in world units.
     * 
     * @return Cell size in meters (typically 1.0).
     */
    double getCellSize() const;
    
    /**
     * @brief Returns the total world width (width * cellSize).
     * 
     * @return World width in meters (typically 200.0 for a 200x1.0 grid).
     */
    double getWorldWidth() const;
    
    /**
     * @brief Returns the total world height (height * cellSize).
     * 
     * @return World height in meters (typically 200.0 for a 200x1.0 grid).
     */
    double getWorldHeight() const;
    
    /**
     * @brief Returns all static obstacles placed in the terrain.
     * 
     * Static obstacles are buildings, towers, and other immobile structures.
     * They are used by the physics engine for collision detection.
     * 
     * @return A const reference to the vector of StaticObstacle objects.
     */
    const std::vector<StaticObstacle>& getObstacles() const;

private:
    /**
     * @brief Evaluates raw Perlin noise at a world coordinate.
     * 
     * Computes the combined multi-octave noise value that forms the basis
     * of the terrain heightmap. The result is typically in range [0, 40].
     * 
     * @param x World X coordinate.
     * @param y World Y coordinate.
     * @return Raw noise value.
     */
    double calculateRawNoise(double x, double y) const;
    
    /**
     * @brief Generates the initial random permutation table for Perlin noise.
     */
    void initializeNoise(unsigned int seed);
    
    /**
     * @brief Procedurally places buildings on land while respecting safety margins.
     * 
     * Keeps obstacles away from the central helipad zone and prevents overlaps.
     */
    void generateStaticObstacles(unsigned int seed, int numStaticObstacles);
    
    /**
     * @brief Calculates the noise value threshold for the shoreline.
     * 
     * Samples the noise field and uses O(N) selection to find a threshold that
     * results in the desired land-to-water ratio.
     */
    void calculateWaterThreshold(double landProp);
    
    /**
     * @brief Computes a single Perlin noise value at 3D coordinates.
     * 
     * Implements classical Perlin noise using permutation tables and
     * gradient interpolation. This is the low-level noise function used
     * by calculateRawNoise.
     * 
     * @param nx X coordinate in noise space.
     * @param ny Y coordinate in noise space.
     * @param nz Z coordinate in noise space.
     * @return Interpolated noise value.
     */
    double getNoiseValue(double nx, double ny, double nz) const;

    // Terrain Properties
    int m_width, m_height;                      ///< Grid dimensions in cells.
    double m_cellSize;                          ///< Size of each cell in world units.
    double m_waterThreshold;                    ///< Height threshold below which terrain is water.
    double m_landProp;                          ///< Desired proportion of land (0.0 - 1.0).
    
    // Perlin Noise
    std::vector<int> m_permutation;             ///< Permutation table for Perlin noise (size 512).
    
    // Obstacles
    std::vector<StaticObstacle> m_staticObstacles;  ///< Immobile buildings and structures.
};