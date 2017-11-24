#ifndef WORLD_H
#define WORLD_H

#include <mutex>
#include <map>
#include <vector>
#include <tf/tf.h>


class World {
    struct Region {
        // Information about owner
        struct {
            double x;
            double y;
            double z;
            size_t id;
        } m_owner;

        std::mutex m_occupationMutex;

        Region();
       ~Region();

        Region(const Region &) = delete;
        Region(Region &&) = delete;

        inline bool isFree() const;
   };

    // Region parameters (in meters)
    double m_regWidth;
    double m_regLength;
    double m_regHeight;

    // Number of blocks of the world
    size_t m_dimZ;
    size_t m_dimY;
    size_t m_dimX;

    // Offset parameters (in meters)
    double m_offsetOX;
    double m_offsetOY;
    double m_offsetOZ;

    // Fixes coordinates if they are negative
    inline double moveX(double x) const;
    inline double moveY(double y) const;
    inline double moveZ(double z) const;

    std::vector<std::vector<std::vector<Region *>>> m_regions;
    std::map<size_t, Region *> m_regionInOwnership;

    // Return a pointer to region that contains a point (x, y, z)
    inline Region *region(double x, double y, double z);

public:
    World(double worldWidth, double worldLenght, double worldHeight,
          double regWidth,   double regLength,   double regHeight,
          double offsetOX,   double offsetOY,    double offsetOZ);
   ~World();

    World(const World &) = delete;
    World(World &&) = delete;

    // Checks if there are other robots nearby point (x, y, z)
    bool isSafePosition(double x, double y, double z, double eps = 0.4) const;

    /* 
     * Return the center of the nearest free region.
     * If there is no free center then return the point itself
     */
    tf::Vector3 getFreeCenter(double x, double y, double z) const;

    // Try occupy a region that contains point (x, y, z)
    bool occupyRegion(double x, double y, double z, size_t id);
};

#endif // WORLD_H
