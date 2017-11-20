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


std::mutex GoalsPublisher::m_errMutex;
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
    , m_subscriber                ()
    , m_listener                  ()
    , m_publishRate               (publishRate)
{
    m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(5.0));
    m_publisher = m_node.advertise<geometry_msgs::PoseStamped>(m_frame + "/goal", 1);
    srand(time(NULL));

    if (!path.empty())
        m_runThread = std::thread(&GoalsPublisher::runAutomatic,  this, path);
    else
        m_runThread = std::thread(&GoalsPublisher::runControlled, this, 50.0);
}


GoalsPublisher::~GoalsPublisher() {
    m_runThread.join();
}


void GoalsPublisher::initWorld(
    double worldWidth, double worldLength, double worldHeight,
    double regWidth,   double regLength,   double regHeight)
{
    m_world = make_unique<World>(worldWidth, worldLength, worldHeight, regWidth, regLength, regHeight);
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


inline bool GoalsPublisher::goalIsReached(const Goal &position, const Goal &goal) const {
    return (fabs(position.x()     - goal.x()) < 0.2) &&
           (fabs(position.y()     - goal.y()) < 0.2) &&
           (fabs(position.z()     - goal.z()) < 0.2) &&
           (fabs(position.roll()  - goal.roll())  < degToRad(10)) &&
           (fabs(position.pitch() - goal.pitch()) < degToRad(10)) &&
           (fabs(position.yaw()   - goal.yaw())   < degToRad(10));
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
            //ROS_WARN("%s%s", m_frame.c_str(), " is waiting");
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

                if (goalIsReached(position, *goal)) {
                    ros::Duration(goal->delay()).sleep();
                    break; // go to next goal
                }
                m_publishRate.sleep();
            }
        }
        else {
            // Interpolating from position to tmpGoal and backward
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


void GoalsPublisher::runControlled(double frequency) {
    m_subscriber = m_node.subscribe("/swarm/commands", 1, &GoalsPublisher::directionChanged, this);
    ros::Timer timer = m_node.createTimer(ros::Duration(1.0/frequency), &GoalsPublisher::goToGoal, this);

    ros::Rate loop(10);
    while (ros::ok) {
        ros::spinOnce();
        loop.sleep();
    }
}


void GoalsPublisher::directionChanged(const std_msgs::Byte::ConstPtr &direction) {
    m_direction = direction->data;
}


inline Goal GoalsPublisher::getGoal() const {
    Goal position   = getPosition();

    double x        = position.x();
    double y        = position.y();
    double z        = position.y();
    double roll     = position.roll();
    double pitch    = position.pitch();
    double yaw      = position.yaw();

    double step     = 0.05;

    if (m_direction == DIRECTION::forward)
        y += step;
    else if (m_direction == DIRECTION::backward)
        y = ((y - step) > 0)? y : y - step;
    else if (m_direction == DIRECTION::rightward)
        x += step;
    else if (m_direction == DIRECTION::leftward)
        x = ((x - step) > 0)? x : x - step;
    else if (m_direction == DIRECTION::upward)
        z += step;
    else if (m_direction == DIRECTION::downward)
        z = ((z - step) > 0)? z : z - step;

    m_direction = 0;
    
    return Goal(x, y, z, roll, pitch, yaw);
}


void GoalsPublisher::goToGoal(const ros::TimerEvent &event) {
    if (!m_direction) return;

    Goal position = getPosition();
    Goal goal = getGoal();

    while (!m_world->occupyRegion  (goal.x(), goal.y(), goal.z(), m_robot_id) ||
           !m_world->isSafePosition(goal.x(), goal.y(), goal.z()))
    {
        if (m_direction != 0) return; // interrupt from world
        ROS_WARN("%s%s", m_frame.c_str(), " is waiting");
        m_publisher.publish(position.getMsg());
        m_publishRate.sleep();
    }

    while (ros::ok()) {
        if (m_direction != 0) return; // interrupt from world
        m_publisher.publish(goal.getMsg());
        m_publishRate.sleep();
    }
}
