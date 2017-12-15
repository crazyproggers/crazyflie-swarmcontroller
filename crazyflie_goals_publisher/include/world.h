#ifndef WORLD_H
#define WORLD_H

#include <mutex>
#include <map>
#include <vector>
#include <tf/tf.h>


class Region;
class World;


struct Occupator {
    Occupator(const std::string &name, double x0, double y0, double z0);

    Occupator() = delete;
    Occupator(const Occupator &) = delete;
    Occupator(Occupator &&) = delete;

    std::string		name;
    Region 	   	   *region;
    double 			x, y, z;

    void setXYZ(double x, double y, double z);
};


class Region {
    friend class World;

    std::mutex  m_occupationMutex;
    Occupator  *m_owner;

public:
    Region();

    Region(const Region &) = delete;
    Region(Region &&) = delete;

    bool isFree() const;
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

    std::vector<std::vector<std::vector<Region *>>> m_regions;

    std::mutex m_registerMutex;
    std::vector<tf::Vector3> m_registrationPoints;

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
