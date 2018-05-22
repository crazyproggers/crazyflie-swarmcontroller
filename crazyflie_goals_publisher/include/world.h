#ifndef WORLD_H
#define WORLD_H

#include <tf/tf.h>
#include <mutex>

class Region;
class World;


class Occupator {
    friend class World;

    std::string     m_name;
    size_t          m_id;
    double          extraWaitingTime;
    const Region   *prevRegion;
    Region         *region;
    double          x, y, z;

public:
    Occupator(const std::string &name, double x0, double y0, double z0);
   ~Occupator();

    Occupator()                                 = delete;
    Occupator(const Occupator &)                = delete;
    Occupator(Occupator &&)                     = delete;
    Occupator & operator=(const Occupator &)    = delete;
    Occupator & operator=(Occupator &&)         = delete;

    std::string     name()     const noexcept;
    size_t          id()       const noexcept;
    bool            isActive() const noexcept;

    void            freeRegion()                            noexcept;
    double          ejectExtraWaitingTime()                 noexcept;
    void            updateXYZ(double x, double y, double z) noexcept;
};


class Region {
    friend class World;

    Occupator  *owner;
    std::mutex  occupationMutex;

public:
    Region();
   ~Region();

    Region(const Region &)              = delete;
    Region(Region &&)                   = delete;
    Region & operator=(const Region &)  = delete;
    Region & operator=(Region &&)       = delete;

    bool isFree() const noexcept;
    void free() noexcept;
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

    // Ranges of the world
    double m_OXmin;
    double m_OYmin;
    double m_OZmin;
    double m_OXmax;
    double m_OYmax;
    double m_OZmax;

    // Filling of the world
    std::vector<std::vector<std::vector<Region*>>> m_regions;

    // This mutex is needed for global synchronization between all occupators
    std::mutex m_globalMutex;

    /*
     * All users of the world
     * key:   id
     * value: occupator
     */
    std::map<size_t, std::weak_ptr<Occupator>> m_occupators;

    // Fixes coordinates if they are negative
    double moveX(double x) const noexcept;
    double moveY(double y) const noexcept;
    double moveZ(double z) const noexcept;

    // Checks if distances from point (x, y, z) to occupators in nearest regions are safe
    bool safeDistances(const Occupator &occupator, double x, double y, double z, double eps = 0.6) const noexcept;

public:
    World(double worldWidth, double worldLenght, double worldHeight,
          double regWidth,   double regLength,   double regHeight,
          double offsetOX,   double offsetOY,    double offsetOZ);
   ~World();

    World(const World &)                = delete;
    World(World &&)                     = delete;
    World & operator=(const World &)    = delete;
    World & operator=(World &&)         = delete;

    // Try to register occupator
    bool addOccupator(std::shared_ptr<Occupator> occupator);

    // Delete occupator from world
    void delOccupator(size_t occupatorId);

    // Try occupy a region that contains point (x, y, z)
    bool occupyRegion(Occupator &occupator, double x, double y, double z) noexcept;

    /* 
     * Return the center of the nearest free region
     * If there is no free center then return current position of occupator
     */
    tf::Vector3 retreat(const Occupator &occupator) noexcept;

    // Return min/max value of the world at one of the axis
    double getOXMin() const noexcept;
    double getOYMin() const noexcept;
    double getOZMin() const noexcept;

    double getOXMax() const noexcept;
    double getOYMax() const noexcept;
    double getOZMax() const noexcept;
};

#endif // WORLD_H