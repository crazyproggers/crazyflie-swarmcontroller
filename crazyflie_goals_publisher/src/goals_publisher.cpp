#include <tf/transform_listener.h>

#include "world.h"
#include "goal.h"
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
    const std::string   &worldFrame,
    const std::string   &frame,
    size_t               publishRate,
    std::list<Goal>      path)
    : m_node                      ()
    , m_worldFrame                (worldFrame)
    , m_frame                     (frame)
    , m_occupator                 ()
    , m_publisher                 ()
    , m_listener                  ()
    , m_publishingIsStopped       (false)
    , m_publishRate               (publishRate)
    , m_direction                 (0)

{
    m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(5.0));
    m_publisher       = m_node.advertise<geometry_msgs::PoseStamped>(m_frame + "/goal", 1);
    m_stopPublishing  = m_node.advertiseService(m_frame + "/stop_publishing",  &GoalsPublisher::stopPublishing,  this);
    m_startPublishing = m_node.advertiseService(m_frame + "/start_publishing", &GoalsPublisher::startPublishing, this);

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


inline Pose GoalsPublisher::getPose() const {
    tf::StampedTransform poseAtSpace;

    try {
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), poseAtSpace);
    }
    catch (tf::TransformException &exc) {
        ROS_ERROR("%s%s", m_frame.c_str(), ": could not get current pose!");
        ROS_ERROR("An exception was caught: %s", exc.what());
        return Pose();
    }

    double x = poseAtSpace.getOrigin().x();
    double y = poseAtSpace.getOrigin().y();
    double z = poseAtSpace.getOrigin().z();

    double roll, pitch, yaw;
    tf::Quaternion quaternion = poseAtSpace.getRotation();
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    return Pose(x, y, z, roll, pitch, yaw);
}


inline bool GoalsPublisher::isFarFromGoal(const Pose &pose, const Goal &goal, double eps) const noexcept {
    auto dist = [](const Pose &pose, const Goal &goal) -> double {
        return std::sqrt(std::pow(pose.x() - goal.x(), 2) +
                         std::pow(pose.y() - goal.y(), 2) +
                         std::pow(pose.z() - goal.z(), 2));
    };

    if (dist(pose, goal) < eps)
        return false;

    return true;
}


bool GoalsPublisher::stopPublishing(
        std_srvs::Empty::Request  &req,
        std_srvs::Empty::Response &res)
{
    if (m_occupator != nullptr && m_world != nullptr) {
        m_occupator->freeRegion();
        m_publishingIsStopped = true;
        return true;
    }
    return false;
}


bool GoalsPublisher::startPublishing(
        std_srvs::Empty::Request  &req,
        std_srvs::Empty::Response &res)
{
    m_publishingIsStopped = false;
    return true;
}


void GoalsPublisher::runAutomatic(std::list<Goal> path) {    
    Pose pose   = getPose();
    m_occupator = make_unique<Occupator>(m_frame, pose.x(), pose.y(), pose.z());

    if (!m_world->addOccupator(*m_occupator))
        return;

    bool exactMoving = false;

    for (auto goal = path.begin(); goal != path.end(); ++goal) {
        pose = getPose();
        m_occupator->updateXYZ(pose.x(), pose.y(), pose.z());

        Goal tmpGoal;

        ros::Duration duration(3.0);
        ros::Rate loop(2);
        ros::Time begin = ros::Time::now();

        while (m_publishingIsStopped) {
            ros::spinOnce();
            loop.sleep();
        }

        while (!m_world->occupyRegion(*m_occupator, goal->x(), goal->y(), goal->z())) {
            ROS_INFO("%s%s", m_frame.c_str(), " is waiting");
            m_publisher.publish(pose.msg());

            {
                Pose exactPose = getPose();
                m_occupator->updateXYZ(exactPose.x(), exactPose.y(), exactPose.z());
            }

            ros::Time end = ros::Time::now();

            // If happened deadlock or we wait too long
            if ((end - begin) >= duration) {
                // Retreat into the nearest free region
                tf::Vector3 safe = m_world->retreat(*m_occupator);
                tmpGoal = Goal(safe.x(), safe.y(), safe.z(), 0.0, 0.0, 0.0, 1.0);

                if (!m_occupator->extraWaitingTime)
                    break;
                else {
                    duration += ros::Duration(m_occupator->extraWaitingTime);
                    m_occupator->extraWaitingTime = 0.0;
                    exactMoving = true;
                }
            }

            ros::spinOnce();
            if (m_publishingIsStopped) break;
            loop.sleep();
        }

        if (tmpGoal.isNull()) {
            while (ros::ok()) {
                m_publisher.publish(goal->msg());

                pose = getPose();
                m_occupator->updateXYZ(pose.x(), pose.y(), pose.z());

                // Check that |pose - goal| < E
                if (!exactMoving) {
                    if ((fabs(pose.x()     - goal->x()) < 0.2) &&
                        (fabs(pose.y()     - goal->y()) < 0.2) &&
                        (fabs(pose.z()     - goal->z()) < 0.2) &&
                        (fabs(pose.roll()  - goal->roll())  < degToRad(10)) &&
                        (fabs(pose.pitch() - goal->pitch()) < degToRad(10)) &&
                        (fabs(pose.yaw()   - goal->yaw())   < degToRad(10)))
                    {
                        ros::Duration(goal->delay()).sleep();
                        break; // go to next goal
                    }
                }
                else {
                    if ((fabs(pose.x() - goal->x()) < 0.05) &&
                        (fabs(pose.y() - goal->y()) < 0.05) &&
                        (fabs(pose.z() - goal->z()) < 0.05))
                    {
                        ros::Duration(goal->delay()).sleep();
                        exactMoving = false;
                        break; // go to next goal
                    }
                }

                ros::spinOnce();
                if (m_publishingIsStopped) break;
                m_publishRate.sleep();
            }
        }
        else {
            // Interpolate from pose to tmpGoal and backward
            std::list<Goal> tmpPath  = interpolate(static_cast<Goal&>(pose), tmpGoal);
            std::list<Goal> backPath = interpolate(tmpGoal, *goal);

            auto position = std::next(goal);

            path.splice(position, tmpPath);
            path.splice(position, backPath);
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
    double x     = prev.x;
    double y     = prev.y;
    double z     = prev.z;
    double roll  = prev.roll;
    double pitch = prev.pitch;
    double yaw   = prev.yaw;

    constexpr double movingStep   = 0.1; // meters
    constexpr double eps          = 0.2; // meters

    auto moveByX = [=](double x, double slope) -> double {
        double shiftX = x + slope * movingStep;
        x = (shiftX < m_world->getOXMax() - eps)? shiftX : x;
        x = (shiftX > m_world->getOXMin() + eps)? shiftX : x;
        prev.x = x;

        return x;
    };

    auto moveByY = [=](double y, double slope) -> double {
        double shiftY = y + slope * movingStep;
        y = (shiftY < m_world->getOYMax() - eps)? shiftY : y;
        y = (shiftY > m_world->getOYMin() + eps)? shiftY : y;
        prev.y = y;

        return y;
    };

    auto moveByZ = [=](double z, double shift) -> double {
        double shiftZ = z + shift;
        z = (shiftZ < m_world->getOZMax() - eps)? shiftZ : z;
        z = (shiftZ > m_world->getOZMin() + eps)? shiftZ : z;
        prev.z = z;

        return z;
    };

    auto rotate = [=](double currAngle, double shift) -> double {
        double minAngle = degToRad(-180);
        double maxAngle = degToRad( 180);

        currAngle += (currAngle + shift > maxAngle)? shift - 2 * maxAngle : shift;
        currAngle += (currAngle + shift < minAngle)? shift - 2 * minAngle : shift;
        prev.yaw = currAngle;

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
        z = moveByZ(z, 1.0);

    m_direction = 0;
    
    return Goal(x, y, z, roll, pitch, yaw);
}


void GoalsPublisher::goToGoal() {
    Pose pose   = getPose();
    m_occupator = make_unique<Occupator>(m_frame, pose.x(), pose.y(), pose.z());
    prev.x      = pose.x();
    prev.y      = pose.y();
    prev.z      = pose.z();
    prev.yaw    = pose.yaw();

    if (!m_world->addOccupator(*m_occupator))
        return;

    while (ros::ok()) {
        if (!m_direction) continue;

        Goal goal = getGoal();

        pose = getPose();
        m_occupator->updateXYZ(pose.x(), pose.y(), pose.z());

        ros::Rate loop(2);
        while (m_publishingIsStopped) {
            ros::spinOnce();
            loop.sleep();
        }

        // If m_direction != 0 then it is meant that we have got interrupt from the world
        while ((!m_direction) && !m_world->occupyRegion(*m_occupator, goal.x(), goal.y(), goal.z())) {
            ROS_INFO("%s%s", m_frame.c_str(), " is waiting");
            m_publisher.publish(pose.msg());

             {
                Pose exactPose = getPose();
                m_occupator->updateXYZ(exactPose.x(), exactPose.y(), exactPose.z());
            }

            ros::spinOnce();
            if (m_publishingIsStopped) break;
            loop.sleep();
        }

        while (!m_direction) {
            m_publisher.publish(goal.msg());

            pose = getPose();
            m_occupator->updateXYZ(pose.x(), pose.y(), pose.z());

            ros::spinOnce();
            if (m_publishingIsStopped) break;
            m_publishRate.sleep();
        }
    } // while (ros::ok())
}