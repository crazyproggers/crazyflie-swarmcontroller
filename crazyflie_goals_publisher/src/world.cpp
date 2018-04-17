#include "world.h"


Occupator::Occupator(const std::string &name, double x0, double y0, double z0)
    : m_name            (name)
    , m_id              (std::hash<std::string>()(name))
    , prevRegion        (nullptr)
    , region            (nullptr)
    , extraWaitingTime  (0.0)
    , x                 (x0)
    , y                 (y0)
    , z                 (z0)
{}


void Occupator::updateXYZ(double x, double y, double z) noexcept {
    this->x = x;
    this->y = y;
    this->z = z;
}


double Occupator::ejectExtraWaitingTime() noexcept {
    double tmp = extraWaitingTime;
    extraWaitingTime = 0.0;
    return tmp;
}


void Occupator::freeRegion() noexcept {
    if (region) {
        region->free();
        region     = nullptr;
        prevRegion = nullptr;
    }
}


std::string Occupator::name() const noexcept {
    return m_name;
}


size_t Occupator::id() const noexcept {
    return m_id;
}


bool Occupator::isActive() const noexcept {
    return (region != nullptr);
}


Occupator::~Occupator() {
    freeRegion();
}


Region::Region()
    : owner             (nullptr)
    , occupationMutex   ()
{}


inline bool Region::isFree() const noexcept {
    return (owner == nullptr);
}


void Region::free() noexcept {
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
            std::vector<Region*> vecOX;

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


inline double World::moveX(double x) const noexcept {
    return x + m_offsetOX;
}

inline double World::moveY(double y) const noexcept {
    return y + m_offsetOY;
}

inline double World::moveZ(double z) const noexcept {
    return z + m_offsetOZ;
}


bool World::addOccupator(std::shared_ptr<Occupator> occupator) {
    std::lock_guard<std::mutex> locker(m_globalMutex);

    double movedX = moveX(occupator->x);
    double movedY = moveY(occupator->y);
    double movedZ = moveZ(occupator->z);

    // Get region that containes occupator position
    size_t xNum = movedX / m_regWidth;
    size_t yNum = movedY / m_regLength;
    size_t zNum = movedZ / m_regHeight;

    // Checking if occupator crosses border of the world
    if (xNum >= dimOX || movedX < 0 ||
        yNum >= dimOY || movedY < 0 ||
        zNum >= dimOZ || movedZ < 0)
    {
        ROS_ERROR("%s%s", occupator->name().c_str(), " is outside of the world!");
        return false;
    }

    Region *currReg = m_regions[zNum][yNum][xNum];

    // Cheking if distances between each of occupators are ok
    bool distancesAreOk = true;
    constexpr double eps = 0.4;

    for (auto pair: m_occupators) {
        if (pair.second.expired()) continue;
        std::shared_ptr<Occupator> checked(pair.second);

        if (std::sqrt(std::pow(movedX - moveX(checked->x), 2) +
                      std::pow(movedY - moveY(checked->y), 2)) < eps)
        {
            distancesAreOk = false;
            break;
        }
    }

    if (!currReg->isFree() || !distancesAreOk) {
        ROS_ERROR("Could not register %s%s", occupator->name().c_str(),
                  ": is too close to other occupator!");
        return false;
    }

    // Occupy starting region
    currReg->owner    = occupator.get();
    occupator->region = currReg;

    m_occupators[occupator->id()] = occupator;

    return true;
}


void World::delOccupator(size_t occupatorId) {
    std::lock_guard<std::mutex> locker(m_globalMutex);
    m_occupators.erase(occupatorId);
}


bool World::safeDistances(const Occupator &occupator, double x, double y, double z, double eps) const noexcept {
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
        for (auto pair: m_occupators) {
            if (pair.second.expired())
                continue;

            std::shared_ptr<Occupator> checked(pair.second);

            if (checked.get() == &occupator || !checked->isActive())
                continue;

            if (dist(x, y, z, checked->x, checked->y, checked->z) < eps)
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

    long long startX = -1, finishX = 2;
    if (m_regWidth * (regionX + 0.5) > x) {
        startX  = -2;
        finishX =  1;
    }

    long long startY = -1, finishY = 2;
    if (m_regLength * (regionY + 0.5) > y) {
        startY  = -2;
        finishY =  1;
    }

    long long startZ = 0, finishZ = 1;
    if (m_regHeight * (regionZ + 0.5) > z) {
        startZ  = 1;
        finishZ = 0;
    }

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


bool World::occupyRegion(Occupator &occupator, double x, double y, double z) noexcept {
    bool areSafeDistances = safeDistances(occupator, x, y, z);

    // Get region that containes occupator position
    size_t xNum = moveX(x) / m_regWidth;
    size_t yNum = moveY(y) / m_regLength;
    size_t zNum = moveZ(z) / m_regHeight;
    Region *selectedReg = m_regions[zNum][yNum][xNum];

    std::lock_guard<std::mutex> locker(selectedReg->occupationMutex);

    if (selectedReg->isFree() && areSafeDistances) {
        // free old region of ocuppator
        if (occupator.region)
            occupator.region->owner = nullptr;
        occupator.extraWaitingTime = 0.0;

        // occupy new region and memorize old one
        occupator.prevRegion = occupator.region;
        occupator.region     = selectedReg;
        selectedReg->owner   = &occupator;

        return true;
    }
    // If occupator have been owning selected region
    else if (selectedReg->owner == &occupator && areSafeDistances)
        return true;

    return false;
}


tf::Vector3 World::retreat(const Occupator &occupator) noexcept {
    std::lock_guard<std::mutex> locker(m_globalMutex);

    double x = occupator.x;
    double y = occupator.y;
    double z = occupator.z;

    if (occupator.extraWaitingTime)
        return tf::Vector3(x, y, z);

    struct Step {
        long long x;
        long long y;
    };
    Step steps[4] = {Step{2, 0}, Step{0, 2}, Step{-2, 0}, Step{0, -2}};

    long long currRegionX = moveX(x) / m_regWidth;
    long long currRegionY = moveY(y) / m_regLength;
    long long currRegionZ = moveZ(z) / m_regHeight;

    auto inRangeOX = [this](double indexX) -> bool { return (indexX < dimOX && indexX >= 0); };
    auto inRangeOY = [this](double indexY) -> bool { return (indexY < dimOY && indexY >= 0); };
    auto inRangeOZ = [this](double indexZ) -> bool { return (indexZ < dimOZ && indexZ >= 0); };

    constexpr double extraTime = 3.0;

    auto startX  = currRegionX - 2;
    auto finishX = currRegionX + 3;

    auto startY  = currRegionY - 2;
    auto finishY = currRegionY + 3;

    auto startZ  = currRegionZ - 1;
    auto finishZ = currRegionZ + 2;

    for (short indexX = startX; indexX < finishX; ++indexX)
        for (short indexY = startY; indexY < finishY; ++indexY)
            for (short indexZ = startZ; indexZ < finishZ; ++indexZ)
            {
                if (!(inRangeOX(indexX) && inRangeOY(indexY) && inRangeOZ(indexZ)))
                    continue;

                Region *reg = m_regions[indexZ][indexY][indexX];
                if (!reg->isFree() && reg != occupator.region)
                    reg->owner->extraWaitingTime = extraTime;
            }

    for (Step step: steps) {
        bool positionIsOk = true;
        startX  = currRegionX  + step.x - 1;
        finishX = startX + 3;
        startY  = currRegionY  + step.y - 1;
        finishY = startY + 3;

        for (auto indexX = startX; indexX < finishX; ++indexX)
            for (auto indexY = startY; indexY < finishY; ++indexY)
                for (auto indexZ = startZ; indexZ < finishZ; ++indexZ)
                {
                    if (!(inRangeOX(indexX) && inRangeOY(indexY) && inRangeOZ(indexZ)))
                        continue;

                    if (!m_regions[indexZ][indexY][indexX]->isFree())
                        positionIsOk = false;
                } // for (indexZ = ...)

        if (positionIsOk) {
            auto chekingIndexX = currRegionX + step.x;
            auto chekingIndexY = currRegionY + step.y;

            if (!(inRangeOX(chekingIndexX)) || !(inRangeOY(chekingIndexY)))
                continue;

            startX  = std::min(chekingIndexX, currRegionX);
            finishX = std::max(chekingIndexX, currRegionX) + 1;

            startY  = std::min(chekingIndexY, currRegionY);
            finishY = std::max(chekingIndexY, currRegionY) + 1;

            bool isHeadingBack = false;

            for (auto indexX = startX; indexX < finishX; ++indexX)
                for (auto indexY = startY; indexY < finishY; ++indexY)
                    if (m_regions[currRegionZ][indexY][indexX] == occupator.prevRegion) {
                        isHeadingBack = true;
                        goto end;
                    }
            end:

            x = ((double) chekingIndexX + 0.5) * m_regWidth  - m_offsetOX;
            y = ((double) chekingIndexY + 0.5) * m_regLength - m_offsetOY;

            if (!isHeadingBack) break;
        } // if (positionIsOk)
    } // for (Step step: steps)

    return tf::Vector3(x, y, z);
}


double World::getOXMin() const noexcept {
    return -m_offsetOX;
}

double World::getOYMin() const noexcept {
    return -m_offsetOY;
}

double World::getOZMin() const noexcept {
    return -m_offsetOZ;
}

double World::getOXMax() const noexcept {
    return dimOX * m_regWidth - m_offsetOX;
}

double World::getOYMax() const noexcept {
    return dimOY * m_regLength - m_offsetOY;
}

double World::getOZMax() const noexcept {
    return dimOZ * m_regHeight - m_offsetOZ;
}