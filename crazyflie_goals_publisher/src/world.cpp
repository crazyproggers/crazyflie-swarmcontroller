#include <cmath>
#include "world.h"


World::World(
    double worldWidth, double worldLength, double worldHeight,
    double regWidth,   double regLength,   double regHeight)
    : m_regWidth            (regWidth)
    , m_regLength           (regLength)
    , m_regHeight           (regHeight)
    , m_dimZ                (std::ceil(worldHeight / regHeight) + 1)
    , m_dimY                (std::ceil(worldLength / regLength) + 1)
    , m_dimX                (std::ceil(worldWidth  / regWidth)  + 1)
    , m_regionInOwnership  ()
{
    // Fill the world
    for (size_t i = 0; i < m_dimZ; ++i) {
        std::vector<std::vector<Region *>> vecOY;

        for (size_t j = 0; j < m_dimY; ++j) {
            std::vector<Region *> vecOX;

            for (size_t k = 0; k < m_dimX; ++k)
                vecOX.push_back(new Region);
            vecOY.push_back(vecOX);
        }
        m_regions.push_back(vecOY);
    }
}


World::~World() {
    for (size_t i = 0; i < m_dimZ; ++i)
        for (size_t j = 0; j < m_dimY; ++j)
            for (size_t k = 0; k < m_dimX; ++k)
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

    size_t newX = oldX + ((oldX < regX && oldX > 0)? -1 : 0) + ((oldX > 2 * regX && oldX + 1 < m_dimX)? 1 : 0);
    size_t newY = oldY + ((oldY < regY && oldY > 0)? -1 : 0) + ((oldY > 2 * regY && oldY + 1 < m_dimY)? 1 : 0);
    size_t newZ = oldZ + ((oldZ < regZ && oldZ > 0)? -1 : 0) + ((oldZ > 2 * regZ && oldZ + 1 < m_dimZ)? 1 : 0);

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
        if (dist(x, y, z, reg) < eps)
            return false;

    reg = m_regions[oldZ][newY][oldX];
    if (!isNewZ && isNewY && !isNewX && !reg->isFree())
        if (dist(x, y, z, reg) < eps)
            return false;

    reg = m_regions[oldZ][newY][newX];
    if (!isNewZ && isNewY && isNewX && !reg->isFree())
        if (dist(x, y, z, reg) < eps)
            return false;

    reg = m_regions[newZ][oldY][oldX];
    if (isNewZ && !isNewY && !isNewX && !reg->isFree())
        if (dist(x, y, z, reg) < eps)
            return false;

    reg = m_regions[newZ][oldY][newX];
    if (isNewZ && !isNewY && isNewX && !reg->isFree())
        if (dist(x, y, z, reg) < eps)
            return false;

    reg = m_regions[newZ][newY][oldX];
    if (isNewZ && isNewY && !isNewX && !reg->isFree())
        if (dist(x, y, z, reg) < eps)
            return false;

    reg = m_regions[newZ][newY][newX];
    if (isNewZ && isNewY && isNewX && !reg->isFree())
        if (dist(x, y, z, reg) < eps)
            return false;

    return true;
}


tf::Vector3 World::getFreeCenter(double x, double y, double z) const {
    struct Step {
        short x, y;
        Step(short x, short y) : x(x), y(y) {}
    };

    std::vector<Step> steps = {Step(0, -1), Step(-1, -1), Step(-1, 0), Step(-1,  1),
                               Step(0,  1), Step( 1,  1), Step( 1, 0), Step( 1, -1)};

    size_t currX = x / m_regWidth;
    size_t currY = y / m_regLength;

    for (auto step: steps) {
        double newX = currX + step.x;
        double newY = currY + step.y;

        if (newX > m_dimX || newX < 0 || newY > m_dimY || newY < 0)
            continue;

        bool freeRegion = true;
        for (size_t i = 0; i < m_regions.size(); ++i) {
            if (!m_regions[i][newY][newX]->isFree()) {
                freeRegion = false;
                break;
            }
        }
        if (freeRegion)
            return tf::Vector3((newX + 0.5) * m_regWidth, (newY + 0.5) * m_regLength, z);
    }

   return tf::Vector3(x, y, z);
}


bool World::occupyRegion(double x, double y, double z, size_t id) {
    Region *selectedReg = region(x, y, z);
    std::lock_guard<std::mutex> locker(selectedReg->m_occupationMutex);

    if (!selectedReg->m_owner.id) {
        Region *regInOwn = m_regionInOwnership[id];

        // if id already has a region, need to free it
        if (regInOwn) {
            regInOwn->m_owner.id = 0;
            regInOwn->m_owner.x  = 0.0;
            regInOwn->m_owner.y  = 0.0;
            regInOwn->m_owner.z  = 0.0;
        }

        // occupy new region
        selectedReg->m_owner.id = id;
        selectedReg->m_owner.x  = x;
        selectedReg->m_owner.y  = y;
        selectedReg->m_owner.z  = z;
        m_regionInOwnership[id] = selectedReg;

        return true;
    }
    else if (selectedReg->m_owner.id == id) {
        selectedReg->m_owner.x = x;
        selectedReg->m_owner.y = y;
        selectedReg->m_owner.z = z;

        return true;
    }

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
    return !m_owner.id;
}
