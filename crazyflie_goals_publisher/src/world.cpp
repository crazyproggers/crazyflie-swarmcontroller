#include <ros/ros.h>
#include <cmath>
#include "world.h"


World::World(
    double worldWidth, double worldLength, double worldHeight,
    double regWidth,   double regLength,   double regHeight)
    : m_regWidth            (regWidth)
    , m_regLength           (regLength)
    , m_regHeight           (regHeight)
    , m_regionsInOwnership  ()
{
    size_t dim1 = std::ceil(worldHeight / m_regHeight) + 1;
    size_t dim2 = std::ceil(worldLength / m_regLength) + 1;
    size_t dim3 = std::ceil(worldWidth  / m_regWidth)  + 1;

    // Fill the world
    for (size_t i = 0; i < dim1; ++i) {
        std::vector<std::vector<Region *>> vecOY;

        for (size_t j = 0; j < dim2; ++j) {
            std::vector<Region *> vecOX;

            for (size_t k = 0; k < dim3; ++k)
                vecOX.push_back(new Region);
            vecOY.push_back(vecOX);
        }
        m_regions.push_back(vecOY);
    }
}


World::~World() {
    size_t dim1 = m_regions.size();
    size_t dim2 = m_regions[0].size();
    size_t dim3 = m_regions[0][0].size();

    for (size_t i = 0; i < dim1; ++i)
        for (size_t j = 0; j < dim2; ++j)
            for (size_t k = 0; k < dim3; ++k)
                delete m_regions[i][j][k];
}


inline World::Region *World::region(double x, double y, double z) {
    size_t xNum = x / m_regWidth;
    size_t yNum = y / m_regLength;
    size_t zNum = z / m_regHeight;

    return m_regions[zNum][yNum][xNum];
}


bool World::isSafePosition(double x, double y, double z, double eps) const {
    size_t oldX = x / m_regWidth;
    size_t oldY = y / m_regLength;
    size_t oldZ = z / m_regHeight;
    size_t regX = m_regWidth  / 3;
    size_t regY = m_regLength / 3;
    size_t regZ = m_regHeight / 3;

    size_t dim1 = m_regions.size();
    size_t dim2 = m_regions[0].size();
    size_t dim3 = m_regions[0][0].size();

    size_t newX = oldX + ((oldX < regX && oldX > 0)? -1 : 0) + ((oldX > 2 * regX && oldX + 1 < dim3)? 1 : 0);
    size_t newY = oldY + ((oldY < regY && oldY > 0)? -1 : 0) + ((oldY > 2 * regY && oldY + 1 < dim2)? 1 : 0);
    size_t newZ = oldZ + ((oldZ < regZ && oldZ > 0)? -1 : 0) + ((oldZ > 2 * regZ && oldZ + 1 < dim1)? 1 : 0);

    // Calculate distance between point (x, y, z) and selected region
    auto dist = [](double x, double y, double z, const Region *region) -> double {
        return std::sqrt(std::pow(x - region->m_owner.x, 2) +
                         std::pow(y - region->m_owner.y, 2) +
                         std::pow(z - region->m_owner.z, 2));
    };

    Region *reg = nullptr;
    size_t isNewX = newX - oldX;
    size_t isNewY = newY - oldY;
    size_t isNewZ = newZ - oldZ;

    /*
     * Find nearest occupied regions and
     * calculate distances from their owners to point (x, y, z)
     */

    reg = m_regions[oldZ][oldY][newX];
    if (!isNewZ && !isNewY && isNewX && !reg->isFree())
        if (dist(x, y, z, reg) > eps)
            return false;

    reg = m_regions[oldZ][newY][oldX];
    if (!isNewZ && isNewY && !isNewX && !reg->isFree())
        if (dist(x, y, z, reg) > eps)
            return false;

    reg = m_regions[oldZ][newY][newX];
    if (!isNewZ && isNewY && isNewX && !reg->isFree())
        if (dist(x, y, z, reg) > eps)
            return false;

    reg = m_regions[newZ][oldY][oldX];
    if (isNewZ && !isNewY && !isNewX && !reg->isFree())
        if (dist(x, y, z, reg) > eps)
            return false;

    reg = m_regions[newZ][oldY][newX];
    if (isNewZ && !isNewY && isNewX && !reg->isFree())
        if (dist(x, y, z, reg) > eps)
            return false;

    reg = m_regions[newZ][newY][oldX];
    if (isNewZ && isNewY && !isNewX && !reg->isFree())
        if (dist(x, y, z, reg) > eps)
            return false;

    reg = m_regions[newZ][newY][newX];
    if (isNewZ && isNewY && isNewX && !reg->isFree())
        if (dist(x, y, z, reg) > eps)
            return false;

    return true;
}


tf::Vector3 World::getFreeCenter(double x, double y, double z) const {
    struct Step {
        short x, y;
        Step(short x, short y) : x(x), y(y) {}
    };

    std::vector<Step> step = {Step(0,  1), Step( 1,  1), Step( 1, 0), Step( 1, -1),
                              Step(0, -1), Step(-1, -1), Step(-1, 0), Step(-1,  1)};

    size_t currX = x / m_regWidth;
    size_t currY = y / m_regLength;

    tf::Vector3 nearestFreeCenter;
    bool isEmpty = true;

    for (size_t i = 0; i < step.size(); ++i) {
        bool freeRegion = true;
        double newX = currX + step[i].x;
        double newY = currY + step[i].y;

        for (size_t j = 0; j < m_regions.size(); ++j)
            if (!m_regions[i][newY][newX]->isFree()) {
                freeRegion = false;
                break;
            }
        if (freeRegion) {
            nearestFreeCenter = tf::Vector3((newX + 0.5) * m_regWidth, (newY + 0.5) * m_regLength, z);
            isEmpty = false;
        }
    }

    if (isEmpty)
        nearestFreeCenter = tf::Vector3(x, y, z);

   return nearestFreeCenter;
}


bool World::occupyRegion(double x, double y, double z, size_t id) {
    Region *newReg = region(x, y, z);
    std::lock_guard<std::mutex> locker(newReg->m_occupationMutex);

    if (!newReg->m_owner.id) {
        Region *oldReg = m_regionsInOwnership[id];

        // if id already has a region, need to free it
        if (oldReg) {
            oldReg->m_owner.id = 0;
            oldReg->m_owner.x  = 0.0;
            oldReg->m_owner.y  = 0.0;
            oldReg->m_owner.z  = 0.0;
        }

        // occupy new region
        newReg->m_owner.id = id;
        newReg->m_owner.x  = x;
        newReg->m_owner.y  = y;
        newReg->m_owner.z  = z;
        m_regionsInOwnership[id] = newReg;

        return true;
    }
    else if (newReg->m_owner.id == id)
        return true;

    return false;
}


World::Region::Region()
    : m_occupationMutex()
{
    m_owner.x = m_owner.y = m_owner.z = 0.0;
    m_owner.id = 0;
}


World::Region::~Region() {}


inline bool World::Region::isFree() const {
    return m_owner.id;
}
