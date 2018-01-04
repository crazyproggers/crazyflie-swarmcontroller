#include <cmath>
#include "world.h"


Occupator::Occupator(const std::string &name, double x0, double y0, double z0)
    : m_name    (name)
    , m_id      (std::hash<std::string>()(name))
    , region    (nullptr)
    , x         (x0)
    , y         (y0)
    , z         (z0)
{}


void Occupator::updateXYZ(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}


void Occupator::freeRegion() {
    region->free();
    region = nullptr;
}


std::string Occupator::name() const {
    return m_name;
}


size_t Occupator::id() const {
    return m_id;
}


Occupator::~Occupator() {
    freeRegion();
}


Region::Region()
    : owner             (nullptr)
    , occupationMutex   ()
{}


inline bool Region::isFree() const {
    return (owner == nullptr);
}


void Region::free() {
    owner = nullptr;
}


Region::~Region() {
    free();
}


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
    , m_globalMutex         ()
    , m_occupators          ()
{
    double _dimOZ = std::ceil(worldHeight / regHeight);
    double _dimOY = std::ceil(worldLength / regLength);
    double _dimOX = std::ceil(worldWidth  / regWidth);

    // Fill the world
    for (size_t i = 0; i < _dimOZ; ++i) {
        std::vector<std::vector<Region*>> vecOY;

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
            for (size_t k = 0; k < dimOX; ++k) {
                delete m_regions[i][j][k];
                m_regions[i][j][k] = nullptr;
            }
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


bool World::addOccupator(Occupator &occupator) {
    std::lock_guard<std::mutex> locker(m_globalMutex);

    double movedX = moveX(occupator.x);
    double movedY = moveY(occupator.y);
    double movedZ = moveZ(occupator.z);

    // Get region that containes occupator position
    size_t xNum = movedX / m_regWidth;
    size_t yNum = movedY / m_regLength;
    size_t zNum = movedZ / m_regHeight;

    // Checking if occupator crosses border of the world
    if (xNum >= dimOX || movedX < 0 ||
        yNum >= dimOY || movedY < 0 ||
        zNum >= dimOZ || movedZ < 0)
    {
        ROS_ERROR("%s%s", occupator.name().c_str(), " is outside of the world!");
        return false;
    }

    Region *currReg = m_regions[zNum][yNum][xNum];

    // Cheking if distances between each of occupators are ok
    bool distancesAreOk = true;
    double eps = 0.4;

    for (auto checked: m_occupators)
        if (std::sqrt(std::pow(movedX - moveX(checked.second->x), 2) +
                      std::pow(movedY - moveY(checked.second->y), 2)) < eps)
        {
            distancesAreOk = false;
            break;
        }

    if (!currReg->isFree() || !distancesAreOk) {
        ROS_ERROR("Could not register %s%s", occupator.name().c_str(),
                  ": is too close to other occupator!");
        return false;
    }

    // Occupy starting region
    currReg->owner   = &occupator;
    occupator.region = currReg;

    m_occupators[occupator.id()] = &occupator;

    return true;
}


bool World::safeDistances(const Occupator &occupator, double x, double y, double z, double eps) const {
    x = moveX(x);
    y = moveY(y);
    z = moveZ(z);

    // Calculate distance between two points
    auto dist = [this](double x1, double y1, double z1, double x2, double y2, double z2) -> double {
        if (fabs(z1 - z2) > m_regHeight / 2)
            return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2) + std::pow(z1 - z2, 2));

        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    };

    if (m_occupators.size() <= 15) {
        for (auto checked: m_occupators) {
            if (checked.second != &occupator)
                continue;

            double checked_x = checked.second->x;
            double checked_y = checked.second->y;
            double checked_z = checked.second->z;

            if (dist(x, y, z, checked_x, checked_y, checked_z) < eps)
                return false;
        }

        return true;
    }

    long long regionX = x / m_regWidth;
    long long regionY = y / m_regLength;
    long long regionZ = z / m_regHeight;

    /*
     * Find nearest occupied regions and
     * calculate distances from their owners to point (x, y, z)
     */

    long long startX  = (x > m_regWidth * (regionX + 0.5)) ?  -1 : -2;
    long long finishX = (x > m_regWidth * (regionX + 0.5)) ?   2 :  1;

    long long startY  = (y > m_regLength * (regionY + 0.5)) ? -1 : -2;
    long long finishY = (y > m_regLength * (regionY + 0.5)) ?  2 :  1;

    long long startZ  = (z > m_regHeight * (regionZ + 0.5)) ?  0 :  1;
    long long finishZ = (z > m_regHeight * (regionZ + 0.5)) ?  1 :  0;

    for (long long i = startX; i <= finishX; ++i) {
        long long X = regionX + i;
        if (X < 0 || X > dimOX - 1)
            continue;

        for (long long j = startY; j <= finishY; ++j) {
            long long Y = regionY + j;
            if (Y < 0 || Y > dimOY - 1)
                continue;

            for (long long k = startZ; k <= finishZ; ++k) {
                long long Z = regionZ + k;
                if (Z < 0 || Z > dimOZ - 1)
                    continue;

                Region *reg = m_regions[Z][Y][X];
                if (!reg->isFree() && reg != occupator.region) {
                    double occupator_x = moveX(reg->owner->x);
                    double occupator_y = moveY(reg->owner->y);
                    double occupator_z = moveZ(reg->owner->z);

                    if (dist(x, y, z, occupator_x, occupator_y, occupator_z) < eps)
                        return false;
                }
            } // for (long long k = startZ; k <= finishZ; ++k)
        } // for (long long j = startY; j <= finishY; ++j)
    } // for (long long i = startX; i <= finishX; ++i)

    return true;
}


bool World::occupyRegion(Occupator &occupator, double x, double y, double z) {
    bool areSafeDistances = safeDistances(occupator, x, y, z);

    // Get region that containes occupator position
    size_t xNum = moveX(x) / m_regWidth;
    size_t yNum = moveY(y) / m_regLength;
    size_t zNum = moveZ(z) / m_regHeight;
    Region *selectedReg = m_regions[zNum][yNum][xNum];

    std::lock_guard<std::mutex> locker(selectedReg->occupationMutex);

    if (selectedReg->isFree() && areSafeDistances) {
        // free old region of ocuppator
        occupator.region->owner = nullptr;

        // occupy new region
        occupator.region   = selectedReg;
        selectedReg->owner = &occupator;

        return true;
    }
    // If occupator have been owning selected region
    else if (selectedReg->owner == &occupator && areSafeDistances)
        return true;

    return false;
}


tf::Vector3 World::retreat(const Occupator &occupator) const {
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
     * If it is not free then return point (x, y, z) itself
     */

    struct Step {
        long long x, y, z;

        Step(long long x,
             long long y,
             long long z) :
            x(x), y(y), z(z) {}
    };

    std::vector<Step> steps = {Step(0, -1, 0), Step(-1, 0, 0), Step(0, 1, 0), Step(1, 0, 0), Step(0, 0, 1)};

    long long currX = moveX(occupator.x) / m_regWidth;
    long long currY = moveY(occupator.y) / m_regLength;
    long long currZ = moveZ(occupator.z) / m_regHeight;

    for (auto step: steps) {
        long long newX = currX + step.x;
        long long newY = currY + step.y;
        long long newZ = currZ + step.z;

        // Checking if robot will cross border of the "world"
        if (newX >= dimOX || newX < 0 || newY >= dimOY || newY < 0 || newZ >= dimOZ || newZ < 0)
            continue;

        // Returnes center of free region
        if (m_regions[newZ][newY][newX]->isFree())
            return tf::Vector3((newX + 0.5) * m_regWidth  - m_offsetOX,
                               (newY + 0.5) * m_regLength - m_offsetOY,
                               (newZ + 0.5) * m_regHeight - m_offsetOZ);
    }

    return tf::Vector3(occupator.x, occupator.y, occupator.z);
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