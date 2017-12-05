#include <cmath>
#include "world.h"


// Amount of blocks on each of axis
#define dimOZ m_regions.size()
#define dimOY m_regions[0].size()
#define dimOX m_regions[0][0].size()


World::World(
    double worldWidth, double worldLength, double worldHeight,
    double regWidth,   double regLength,   double regHeight,
    double offsetOX,   double offsetOY,    double offsetOZ)
    : m_regWidth            (regWidth)
    , m_regLength           (regLength)
    , m_regHeight           (regHeight)
    , m_offsetOX            (offsetOX)
    , m_offsetOY            (offsetOY)
    , m_offsetOZ            (offsetOZ)
    , m_regionInOwnership   ()
{
    double _dimOZ = std::ceil(worldHeight / regHeight);
    double _dimOY = std::ceil(worldLength / regLength);
    double _dimOX = std::ceil(worldWidth  / regWidth);

    // Fill the world
    for (size_t i = 0; i < _dimOZ; ++i) {
        std::vector<std::vector<Region *>> vecOY;

        for (size_t j = 0; j < _dimOY; ++j) {
            std::vector<Region *> vecOX;

            for (size_t k = 0; k < _dimOX; ++k)
                vecOX.push_back(new Region);
            vecOY.push_back(vecOX);
        }

        m_regions.push_back(vecOY);
    }
}


World::~World() {
    for (size_t i = 0; i < dimOZ; ++i)
        for (size_t j = 0; j < dimOY; ++j)
            for (size_t k = 0; k < dimOX; ++k)
                delete m_regions[i][j][k];
}


inline double World::moveX(double x) const {
    return x + m_offsetOX;
}

inline double World::moveY(double y) const {
    return y + m_offsetOY;
}

inline double World::moveZ(double z) const {
    return z + m_offsetOZ;
}


bool World::isSafePosition(double x, double y, double z, double eps) const {
    x = moveX(x);
    y = moveY(y);
    z = moveZ(z);

    size_t oldX = x / m_regWidth;
    size_t oldY = y / m_regLength;
    size_t oldZ = z / m_regHeight;

    size_t regX = m_regWidth  / 3;
    size_t regY = m_regLength / 3;
    size_t regZ = m_regHeight / 3;

    size_t newX = oldX + ((oldX < regX && oldX > 0)? -1 : 0) + ((oldX > 2 * regX && oldX + 1 < dimOX)? 1 : 0);
    size_t newY = oldY + ((oldY < regY && oldY > 0)? -1 : 0) + ((oldY > 2 * regY && oldY + 1 < dimOY)? 1 : 0);
    size_t newZ = oldZ + ((oldZ < regZ && oldZ > 0)? -1 : 0) + ((oldZ > 2 * regZ && oldZ + 1 < dimOZ)? 1 : 0);

    // Calculate distance between point (x, y, z) and selected region
    auto dist = [this](double x, double y, double z, const Region *region) -> double {
        if (fabs(z - region->m_owner.z) > m_regHeight / 2) {
            return std::sqrt(std::pow(x - region->m_owner.x, 2) +
                             std::pow(y - region->m_owner.y, 2) +
                             std::pow(z - region->m_owner.z, 2));
        }

        return std::sqrt(std::pow(x - region->m_owner.x, 2) +
                         std::pow(y - region->m_owner.y, 2));
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
    /*
     * If there is deadlock then check ways to step back like in picture
     *    |----|----|----|
     *    |    |    |    |
     *    |    | 1  |    |
     *    |----|----|----|
     *    |    |    |    |
     *    |  2 | X  | 4  |
     *    |----|----|----|
     *    |    |    |    |
     *    |    | 3  |    |
     *    |----|----|----|
     * If there are no ways to step back then check upward way
     * If it is not free then return point (x, y, z)
     */

    struct Step {
        long long x, y, z;

        Step(long long x,
             long long y,
             long long z) :
            x(x), y(y), z(z) {}
    };

    std::vector<Step> steps = {Step(0, -1, 0), Step(-1, 0, 0), Step(0, 1, 0), Step(1, 0, 0), Step(0, 0, 1)};

    long long currX = moveX(x) / m_regWidth;
    long long currY = moveY(y) / m_regLength;
    long long currZ = moveZ(z) / m_regHeight;

    for (auto step: steps) {
        long long newX = currX + step.x;
        long long newY = currY + step.y;
        long long newZ = currZ + step.z;

        // Checking if robot will cross border of the "world"
        if (newX > dimOX || newX < 0 || newY > dimOY || newY < 0 || newZ > dimOZ)
            continue;

        // Returnes center of free region
        if (m_regions[newZ][newY][newX]->isFree())
            return tf::Vector3((newX + 0.5) * m_regWidth  - m_offsetOX,
                               (newY + 0.5) * m_regLength - m_offsetOY,
                               (newZ + 0.5) * m_regHeight - m_offsetOZ);
    }

   return tf::Vector3(x, y, z);
}


bool World::occupyRegion(double x, double y, double z, size_t id) {
    x = moveX(x);
    y = moveY(y);
    z = moveZ(z);

    // Get region that containes point (x, y, z)
    size_t xNum = x / m_regWidth;
    size_t yNum = y / m_regLength;
    size_t zNum = z / m_regHeight;
    Region *selectedReg = m_regions[zNum][yNum][xNum];

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


double World::getOXMin() const {
    return -m_offsetOX;
}

double World::getOYMin() const {
    return -m_offsetOY;
}

double World::getOZMin() const {
    return -m_offsetOZ;
}

double World::getOXMax() const {
    return dimOX * m_regWidth - m_offsetOX;
}

double World::getOYMax() const {
    return dimOY * m_regLength - m_offsetOY;
}

double World::getOZMax() const {
    return dimOZ * m_regHeight - m_offsetOZ;
}
