#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <cmath>
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
    size_t rate,
    std::list<Goal> path)
    : m_node                      ()
    , m_worldFrame                (worldFrame)
    , m_frame                     (frame)
    , m_id                        (std::hash<std::string>()(frame))
    , m_publisher                 ()
    , m_subscriber                ()
    , m_listener                  ()
    , m_loopRate                  (rate)
{
    m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(5.0));
    m_publisher = m_node.advertise<geometry_msgs::PoseStamped>(m_frame + "/goal", 1);

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
    ros::Time commonTime;
    tf::StampedTransform position;
    std::string errMsg;

    m_listener.getLatestCommonTime(m_worldFrame, m_frame, commonTime, &errMsg);

    if (!errMsg.empty()) {
        std::lock_guard<std::mutex> locker(m_errMutex);
        ROS_ERROR("%s%s%s", m_frame.c_str(), ": ", errMsg.c_str());
    }

    if (m_listener.canTransform(m_worldFrame, m_frame, commonTime, &errMsg))
        m_listener.lookupTransform(m_worldFrame, m_frame, commonTime, position);
    else {
        std::lock_guard<std::mutex> locker(m_errMutex);
        ROS_ERROR("%s%s", m_frame.c_str(), ": could not get current position!");
        return Goal();
    }

    if (!errMsg.empty()) {
        std::lock_guard<std::mutex> locker(m_errMutex);
        ROS_ERROR("%s%s%s", m_frame.c_str(), ": ", errMsg.c_str());
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
    for (auto goal = path.begin(); goal != path.end(); ++goal) {
        std::vector<double> distances = m_world->distancesToNearestOwners(goal->x(), goal->y(), goal->z());

        // Checking that distance from current crazyflie to the nearest ones to it is ok
        auto distancesAreOk = [&](double E = 0.2) -> bool {
            for (double d: distances)
                if (d <= E) return false;

            return true;
        };

        Goal position = getPosition();
        Goal tmpGoal;

        ros::Duration duration(5.0);
        ros::Time begin = ros::Time::now();

        while (!m_world->occupyRegion(goal->x(), goal->y(), goal->z(), m_id) && !distancesAreOk()) {
            m_publisher.publish(position.getMsg());

            ros::Time end = ros::Time::now();

            // If happened deadlock or we wait too long
            if ((end - begin) >= duration) {
                // tmpGoal = correct(goal);
                break;
            }
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
                m_loopRate.sleep();
            }
        }
        else {
            // here need to interpolate from position to tmpGoal
            // std::list<Goal> tmpPath = interpolation(position, tmpGoal);
            // path.splice(goal, tmpPath);
        }
    } // for (Goal goal: path)

    std_srvs::Empty empty_srv;
    ros::service::call(m_frame + "/land", empty_srv);
} // run(std::vector<Goal> path)


void GoalsPublisher::runControlled(double frequency) {
    m_subscriber = m_node.subscribe("/swarm/commands", 1, &GoalsPublisher::directionChanged, this);
    ros::Timer timer = m_node.createTimer(ros::Duration(1.0/frequency), &GoalsPublisher::goToGoal, this);

    while (ros::ok) {
        ros::spinOnce();
        m_loopRate.sleep();
    }
}


void GoalsPublisher::directionChanged(const std_msgs::Byte::ConstPtr &direction) {
    m_direction = direction->data;
}


Goal GoalsPublisher::getNewGoal(const Goal &oldGoal) {
    /*
     * Directions:
     * forward          -- Y += 0.01;
     * backward         -- Y -= 0.01;
     * rightward        -- X += 0.01;
     * leftward         -- X -= 0.01;
     * upward           -- Z += 0.01;
     * downward         -- Z -= 0.01;
     */
 
    double x        = oldGoal.x();
    double y        = oldGoal.y();
    double z        = oldGoal.y();
    double roll     = oldGoal.roll();
    double pitch    = oldGoal.pitch();
    double yaw      = oldGoal.yaw();

    double step     = 0.1;

    if (m_direction == DIRECTION::forward)
        ;
    else if (m_direction == DIRECTION::backward)
        ;
    else if (m_direction == DIRECTION::rightward)
        ;
    else if (m_direction == DIRECTION::leftward)
        ;
    else if (m_direction == DIRECTION::upward)
        z += step;
    else if (m_direction == DIRECTION::downward)
        z = (z - step)? z : z - step;

    m_direction = 0;
    
    return Goal(x, y, z, roll, pitch, yaw);
}


void GoalsPublisher::goToGoal(const ros::TimerEvent &e) {
    if (!m_direction) return;

    // Create the path
    Goal goal1 = getPosition();
    Goal goal2 = getNewGoal(goal1);

    std::list<Goal> path = interpolate(goal1, goal2);
    if (path.empty()) return;

    for (Goal goal: path) {
        while (ros::ok()) {
            if (m_direction != 0) return; // interupt from world

            m_publisher.publish(goal.getMsg());
            
            Goal position = getPosition();
            if (position.empty()) continue;
            
            if (goalIsReached(position, goal))
                break; // go to next goal

            m_loopRate.sleep();
        } // while (ros::ok())
    }// for (Goal goal: path)
    
    // Hovering at last goal
    Goal goal = path.back();
    while (ros::ok()) {
        // Interupt from world
        if (m_direction != 0) return;

        m_publisher.publish(goal.getMsg());
        m_loopRate.sleep();
    } // while (ros::ok())
}
