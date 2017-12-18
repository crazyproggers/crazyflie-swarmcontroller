#ifndef WORLD_H
#define WORLD_H

#include <mutex>
#include <map>
#include <vector>
#include <tf/tf.h>


class Region;
class World;


class Occupator {
    friend class World;

    std::string     m_name;
    size_t          m_id;
    Region         *region;

public:
    Occupator(const std::string &name, double x0, double y0, double z0);
   ~Occupator();

    Occupator() = delete;
    Occupator(const Occupator &) = delete;
    Occupator(Occupator &&) = delete;

    std::string     name() const;
    size_t          id()   const;
    void            freeRegion();
    double          x, y, z;
    void            updateXYZ(double x, double y, double z);
};


class Region {
    friend class World;

    Occupator  *owner;
    std::mutex  occupationMutex;

public:
    Region();
   ~Region();

    Region(const Region &) = delete;
    Region(Region &&) = delete;

    bool isFree() const;
    void free();
};


class World {
    // Region parameters (in meters)
    double m_regWidth;
    double m_regLength;
    double m_regHeight;

    // Offset parameters (in meters)
    double m_offsetOX;
    double m_offsetOY;
    double m_offsetOZ;

    // Filling of the world
    std::vector<std::vector<std::vector<Region*>>> m_regions;

    // This mutex is needed for global synchronization between all occupators
    std::mutex m_globalMutex;

    /*
     * All users of the world
     * key:   id
     * value: occupator
     */
    std::map<size_t, Occupator*> m_occupators;

    // Fixes coordinates if they are negative
    double moveX(double x) const;
    double moveY(double y) const;
    double moveZ(double z) const;

public:
    World(double worldWidth, double worldLenght, double worldHeight,
          double regWidth,   double regLength,   double regHeight,
          double offsetOX,   double offsetOY,    double offsetOZ);
   ~World();

    World(const World &) = delete;
    World(World &&) = delete;

    // Try to register occupator
    bool addOccupator(Occupator &occupator);

    // Try occupy a region that contains point (x, y, z)
    bool occupyRegion(Occupator &occupator, double x, double y, double z);

    // Checks if there are other robots nearby occupator
    bool isAtSafePosition(const Occupator &occupator, double eps = 0.4) const;

    /* 
     * Return the center of the nearest free region
     * If there is no free center then return current position of occupator
     */
    tf::Vector3 getFreeCenter(const Occupator &occupator) const;

    // Return min/max value of the world at one of the axis
    double getOXMin() const;
    double getOYMin() const;
    double getOZMin() const;

    double getOXMax() const;
    double getOYMax() const;
    double getOZMax() const;
};

#endif // WORLD_H
