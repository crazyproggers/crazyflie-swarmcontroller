#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include "goals_publisher.h"
#include "interpolations.h"


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
    , m_robot_id                  (std::hash<std::string>()(frame))
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
    ros::Rate waitLoop(2);


    for (auto goal = path.begin(); goal != path.end(); ++goal) {
        Goal position = getPosition();
        Goal tmpGoal;

        ros::Duration duration(5.0);
        bool participatedInToss = false;
        ros::Time begin = ros::Time::now();

        while (!m_world->occupyRegion  (goal->x(), goal->y(), goal->z(), m_robot_id) ||
               !m_world->isSafePosition(goal->x(), goal->y(), goal->z()))
        {
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
                tf::Vector3 freeCenter = m_world->getFreeCenter(position.x(), position.y(), position.z());
                tmpGoal = Goal(freeCenter.x(), freeCenter.y(), freeCenter.z(), 0.0, 0.0, 0.0);

                if (tmpGoal.x() != position.x() || tmpGoal.y() != position.y() || tmpGoal.z() != position.z())
                    break;
                else begin = ros::Time::now();
            }

            waitLoop.sleep();
        }

        if (tmpGoal.empty()) {
            while (ros::ok()) {
                m_publisher.publish(goal->getMsg());

                Goal position = getPosition();
                if (position.empty()) continue;

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
    double z        = position.y();
    double roll     = position.roll();
    double pitch    = position.pitch();
    double yaw      = position.yaw();

    double movingStep   = 0.05; // meters
    double rotatingStep = degToRad(10);
    double eps          = 0.1;  // meters

    if (m_direction == DIRECTION::forward) {
        double shift = y + movingStep;
        y = (shift < m_world->getOYMax() - eps)? shift : y;
    }

    else if (m_direction == DIRECTION::backward) {
        double shift = y - movingStep;
        y = (shift > m_world->getOYMin() + eps)? shift : y;
    }

    else if (m_direction == DIRECTION::rightward) {
        double shift = x + movingStep;
        x = (shift < m_world->getOXMax() - eps)? shift : x;
    }

    else if (m_direction == DIRECTION::leftward) {
        double shift = x - movingStep;
        x = (shift > m_world->getOXMin() + eps)? shift : x;
    }

    else if (m_direction == DIRECTION::upward) {
        double shift = z + movingStep;
        z = (shift < m_world->getOZMin() - eps)? shift : z;
    }

    else if (m_direction == DIRECTION::downward) {
        double shift = z - movingStep;
        z = (shift > m_world->getOZMin() + eps)? shift : z;
    }

    else if (m_direction == DIRECTION::yawright) {
        double shift = yaw - rotatingStep;
        yaw = (shift > degToRad(-180.0))? shift : shift + degToRad(360.0);
    }

    else if (m_direction == DIRECTION::yawleft) {
        double shift = yaw + rotatingStep;
        yaw = (shift < degToRad(180.0))? shift : shift - degToRad(360.0);
    }

    m_direction = 0;
    
    return Goal(x, y, z, roll, pitch, yaw);
}


void GoalsPublisher::goToGoal() {
    while (ros::ok()) {
        if (!m_direction) continue;

        Goal position = getPosition();
        Goal goal = getGoal();

        // If m_direction != 0 then it is meant that we have got interrupt from the world
        while (!m_direction &&
              (!m_world->occupyRegion  (goal.x(), goal.y(), goal.z(), m_robot_id) ||
               !m_world->isSafePosition(goal.x(), goal.y(), goal.z())))
        {
            ROS_INFO("%s%s", m_frame.c_str(), " is waiting");
            m_publisher.publish(position.getMsg());
            m_publishRate.sleep();
        }

        while (!m_direction) {
            m_publisher.publish(goal.getMsg());
            m_publishRate.sleep();
        }
    }
}
