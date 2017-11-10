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

    std::vector<std::vector<std::vector<Region *>>> m_regions;
    std::map<size_t, Region *> m_regionsInOwnership;

    // Return a pointer to region that contains a point (x, y, z)
    inline Region *region(double x, double y, double z);

public:
    World(double worldWidth, double worldLenght, double worldHeight,
          double regWidth,   double regLength,   double regHeight);
   ~World();

    World(const World &) = delete;
    World(World &&) = delete;

    /* 
     * Return distances from point (x, y, z) to nearest regions owners
     * Max size of result is 7 (when robot placed at something corner)
     * Min size of result is 0 (when robot placed at center of region)
     */
    std::vector<double> getDistancesToNeighbors(double x, double y, double z) const;

    /* 
     * Return the center of the nearest free region.
     * If there is no free center then return the point itself
     */
    tf::Vector3 getFreeCenter(double x, double y, double z) const;

    // Try occupy a region that contains point (x, y, z)
    bool occupyRegion(double x, double y, double z, size_t id);
};

#endif // WORLD_H
