#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include "goals_publisher.h"
#include "interpolations.h"
#include "commands.h"


constexpr double degToRad(double deg) {
    return deg / 180.0 * M_PI;
}


template<typename T, typename ...Args>
std::unique_ptr<T> make_unique( Args&& ...args ) {
    return std::unique_ptr<T>(new T( std::forward<Args>(args)... ));
}


std::unique_ptr<World> GoalsPublisher::m_world;


GoalsPublisher::GoalsPublisher(
    const std::string &worldFrame,
    const std::string &frame,
    size_t publishRate,
    std::list<Goal> path)
    : m_node                      ()
    , m_worldFrame                (worldFrame)
    , m_frame                     (frame)
    , m_publisher                 ()
    , m_listener                  ()
    , m_publishRate               (publishRate)
    , m_direction                 (0)
{
    m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(5.0));
    m_publisher = m_node.advertise<geometry_msgs::PoseStamped>(m_frame + "/goal", 1);
    std::srand(std::time(NULL));

    if (!path.empty())
        m_runThread = std::thread(&GoalsPublisher::runAutomatic,  this, path);
    else
        m_runThread = std::thread(&GoalsPublisher::runControlled, this);
}


GoalsPublisher::~GoalsPublisher() {
    m_runThread.join();
}


void GoalsPublisher::initWorld(
    double worldWidth, double worldLength, double worldHeight,
    double regWidth,   double regLength,   double regHeight,
    double offsetOX,   double offsetOY,    double offsetOZ)
{
    m_world = make_unique<World>(worldWidth, worldLength, worldHeight, 
                                 regWidth,   regLength,   regHeight,
                                 offsetOX,   offsetOY,    offsetOZ);
}


inline Goal GoalsPublisher::getPosition() const {
    tf::StampedTransform position;

    try {
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), position);
    }
    catch (tf::TransformException &exc) {
        ROS_ERROR("%s%s", m_frame.c_str(), ": could not get current position!");
        ROS_ERROR("An exception was caught: %s", exc.what());
        return Goal();
    }

    double x = position.getOrigin().x();
    double y = position.getOrigin().y();
    double z = position.getOrigin().z();

    double roll, pitch, yaw;
    tf::Quaternion quaternion = position.getRotation();
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    return Goal(x, y, z, roll, pitch, yaw);
}


void GoalsPublisher::runAutomatic(std::list<Goal> path) {
    // Get an extra waiting time in case of deadlock
    auto getExtraWaitingTime = [](double maxWaiting = 3.0) {
        return (rand() % 2)? maxWaiting : 0.0;
    };

    Goal starting = getPosition();
    Occupator occupator(m_frame, starting.x(), starting.y(), starting.z());

    if (!m_world->addOccupator(occupator))
        return;

    for (auto goal = path.begin(); goal != path.end(); ++goal) {
        Goal position = getPosition();
        Goal tmpGoal;

        // Update exact position of occupator
        occupator.updateXYZ(position.x(), position.y(), position.z());

        ros::Duration duration(5.0);
        ros::Rate     waitLoop(2);
        bool participatedInToss = false;
        ros::Time begin = ros::Time::now();

        while (!m_world->occupyRegion(occupator, goal->x(), goal->y(), goal->z())) {
            ROS_INFO("%s%s", m_frame.c_str(), " is waiting");
            m_publisher.publish(position.getMsg());

            ros::Time end = ros::Time::now();

            // If happened deadlock or we wait too long
            if ((end - begin) >= duration) {
                double extraTime = getExtraWaitingTime();

                if (extraTime && !participatedInToss) {
                    participatedInToss = true;
                    duration += ros::Duration(extraTime);
                    continue;
                }

                // Searching the nearest free region center
                tf::Vector3 center = m_world->getFreeCenter(occupator);
                tmpGoal = Goal(center.x(), center.y(), center.z(), 0.0, 0.0, 0.0);

                if (tmpGoal.x() != position.x() || tmpGoal.y() != position.y() || tmpGoal.z() != position.z())
                    break;
                else begin = ros::Time::now();
            }

            waitLoop.sleep();
        }

        if (tmpGoal.empty()) {
            while (ros::ok()) {
                m_publisher.publish(goal->getMsg());

                // Update exact position of occupator
                Goal position = getPosition();
                occupator.updateXYZ(position.x(), position.y(), position.z());

                // Check that |position - goal| < E
                if ((fabs(position.x()     - goal->x()) < 0.2) &&
                    (fabs(position.y()     - goal->y()) < 0.2) &&
                    (fabs(position.z()     - goal->z()) < 0.2) &&
                    (fabs(position.roll()  - goal->roll())  < degToRad(10)) &&
                    (fabs(position.pitch() - goal->pitch()) < degToRad(10)) &&
                    (fabs(position.yaw()   - goal->yaw())   < degToRad(10)))
                {
                    ros::Duration(goal->delay()).sleep();
                    break; // go to next goal
                }

                m_publishRate.sleep();
            }
        }
        else {
            // Interpolate from position to tmpGoal and backward
            std::list<Goal> tmpPath  = interpolate(position, tmpGoal);
            std::list<Goal> backPath = interpolate(tmpGoal, *goal);

            auto pos = std::next(goal);

            path.splice(pos, tmpPath);
            path.splice(pos, backPath);
        }
    } // for (goal = path.begin(); goal != path.end(); ++goal)

    std_srvs::Empty empty_srv;
    ros::service::call(m_frame + "/land", empty_srv);
} // run(std::vector<Goal> path)


void GoalsPublisher::runControlled() {
    ros::Subscriber subscriber = m_node.subscribe("/swarm/commands", 1, &GoalsPublisher::directionChanged, this);
    std::thread goingToGoalThr(&GoalsPublisher::goToGoal, this);

    ros::Rate loop(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop.sleep();
    }
    goingToGoalThr.join();
}


void GoalsPublisher::directionChanged(const std_msgs::Byte::ConstPtr &direction) {
    m_direction = direction->data;
}


inline Goal GoalsPublisher::getGoal() {
    Goal position   = getPosition();
    double x        = position.x();
    double y        = position.y();
    double z        = position.z();
    double roll     = position.roll();
    double pitch    = position.pitch();
    double yaw      = position.yaw();

    double movingStep   = 0.1; // meters
    double eps          = 0.2; // meters

    auto moveByX = [=](double x, double slope) {
        double shiftX = x + slope * movingStep;
        x = (shiftX < m_world->getOXMax() - eps)? shiftX : x;
        x = (shiftX > m_world->getOXMin() + eps)? shiftX : x;

        return x;
    };

    auto moveByY = [=](double y, double slope) {
        double shiftY = y + slope * movingStep;
        y = (shiftY < m_world->getOYMax() - eps)? shiftY : y;
        y = (shiftY > m_world->getOYMin() + eps)? shiftY : y;

        return y;
    };

    auto moveByZ = [=](double z, double shift) {
        double shiftZ = z + shift;
        z = (shiftZ < m_world->getOZMax() - eps)? shiftZ : z;
        z = (shiftZ > m_world->getOZMin() + eps)? shiftZ : z;

        return z;
    };

    auto rotate = [](double currAngle, double shift) {
        double minAngle = degToRad(-180);
        double maxAngle = degToRad( 180);

        currAngle += (currAngle + shift > maxAngle)? shift : shift - maxAngle;
        currAngle += (currAngle + shift < minAngle)? shift : shift - minAngle;

        return currAngle;
    };


    if (m_direction == commands::forward) {
        x = moveByX(x, std::cos(yaw));
        y = moveByY(y, std::sin(yaw));
    }

    else if (m_direction == commands::backward) {
        x = moveByX(x, -std::cos(yaw));
        y = moveByY(y, -std::sin(yaw));
    }

    else if (m_direction == commands::rightward) {
        x = moveByX(x,  std::sin(yaw));
        y = moveByY(y, -std::cos(yaw));
    }

    else if (m_direction == commands::leftward) {
        x = moveByX(x, -std::sin(yaw));
        y = moveByY(y,  std::cos(yaw));
    }

    else if (m_direction == commands::yawright)
        yaw = rotate(yaw, degToRad(-10));

    else if (m_direction == commands::yawleft)
        yaw = rotate(yaw, degToRad(10));

    else if (m_direction == commands::upward)
        z = moveByZ(z,  movingStep);

    else if (m_direction == commands::downward)
        z = moveByZ(z, -movingStep);

    else if (m_direction == commands::takeoff)
        z += 0.5;

    m_direction = 0;
    
    return Goal(x, y, z, roll, pitch, yaw);
}


void GoalsPublisher::goToGoal() {
    Goal starting = getPosition();
    Occupator occupator(m_frame, starting.x(), starting.y(), starting.z());

    if (!m_world->addOccupator(occupator))
        return;

    while (ros::ok()) {
        if (!m_direction) continue;

        Goal position = getPosition();
        Goal goal     = getGoal();

        // Update exact position of occupator
        occupator.updateXYZ(position.x(), position.y(), position.z());

        // If m_direction != 0 then it is meant that we have got interrupt from the world
        while (!m_direction && (!m_world->occupyRegion(occupator, goal.x(), goal.y(), goal.z()))) {
            ROS_INFO("%s%s", m_frame.c_str(), " is waiting");
            m_publisher.publish(position.getMsg());
            m_publishRate.sleep();
        }

        while (!m_direction) {
            m_publisher.publish(goal.getMsg());

            // Update exact position of occupator
            Goal position = getPosition();
            occupator.updateXYZ(position.x(), position.y(), position.z());

            m_publishRate.sleep();
        }
    } // while (ros::ok())
}