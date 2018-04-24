#include <tf/transform_listener.h>

#include "world.h"
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

    m_currPose = getPose();
    if (m_currPose.isNull())
        return;

    m_occupator = std::make_shared<Occupator>(m_frame, m_currPose.x(), m_currPose.y(), m_currPose.z());

    if (m_world == nullptr) {
        ROS_FATAL("You should to call initWorld!");
        return;
    }

    if (!m_world->addOccupator(m_occupator))
        return;

    if (!path.empty())
        std::thread { &GoalsPublisher::runAutomatic,  this, path }.detach();
    else
        std::thread { &GoalsPublisher::runControlled, this       }.detach();
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
    bool exactMoving = false;
    ros::Rate loop   = 2;
    Goal retreatGoal;

    // Return true if |pose - goal| < E
    auto isCloseEnough = [exactMoving](const Pose& pose, const Goal& goal) -> bool {
        double eps = (exactMoving)? 0.05 : 0.2;

        return (fabs(pose.x()     - goal.x()) < eps) &&
               (fabs(pose.y()     - goal.y()) < eps) &&
               (fabs(pose.z()     - goal.z()) < eps) &&
               (fabs(pose.roll()  - goal.roll())  < degToRad(10)) &&
               (fabs(pose.pitch() - goal.pitch()) < degToRad(10)) &&
               (fabs(pose.yaw()   - goal.yaw())   < degToRad(10));
    };


    for (auto goal = path.begin(); goal != path.end(); ++goal) {
        Pose pose = getPose();

        while (m_publishingIsStopped || pose.isNull()) {
            ros::spinOnce();
            loop.sleep();
        }
        m_occupator->updateXYZ(pose.x(), pose.y(), pose.z());

        Goal tmpGoal;
        ros::Duration duration(3.0);
        ros::Time begin = ros::Time::now();

        // Try to occupy current region
        while (!m_publishingIsStopped && !m_world->occupyRegion(*m_occupator, goal->x(), goal->y(), goal->z())) {
            //ROS_INFO("%s%s", m_frame.c_str(), " is waiting");
            m_publisher.publish(pose.msg());

            Pose exactPose = getPose();
            if (!exactPose.isNull()) {
                m_occupator->updateXYZ(exactPose.x(), exactPose.y(), exactPose.z());

                // If a lock happened or we wait too long
                if ((ros::Time::now() - begin) >= duration) {
                    // Retreat into the nearest free region
                    tf::Vector3 safe = m_world->retreat(*m_occupator);

                    if (exactPose.x() != safe.x() || exactPose.y() != safe.y() || exactPose.z() != safe.z()) {
                        retreatGoal = tmpGoal = Goal(safe.x(), safe.y(), safe.z(), 0.0, 0.0, 0.0, 2.0);
                        exactMoving = true;
                        break;
                    }

                    duration += ros::Duration(m_occupator->ejectExtraWaitingTime());
                }
            } // if (!exactPose.isNull())

            ros::spinOnce();
            loop.sleep();
        }

        // If need to reatreat
        if (!tmpGoal.isNull()) {
            // Interpolate from pose to tmpGoal and backward
            std::list<Goal> tmpPath  = interpolate(static_cast<const Goal&>(pose), tmpGoal);
            std::list<Goal> backPath = interpolate(tmpGoal, *goal);

            auto position = std::next(goal);

            path.splice(position, tmpPath);
            path.splice(position, backPath);

            continue; // go to next goal
        }

        // Adjust the pose to the goal
        while (!m_publishingIsStopped && ros::ok()) {
            m_publisher.publish(goal->msg());

            pose = getPose();
            if (!pose.isNull()) {
                m_occupator->updateXYZ(pose.x(), pose.y(), pose.z());

                if (isCloseEnough(pose, *goal)) {
                    ros::Duration(goal->delay()).sleep();

                    if (*goal == retreatGoal) {
                        retreatGoal = Goal();
                        exactMoving = false;
                    }
                    break; // go to next goal
                }
            } // if (!pose.isNull())

            ros::spinOnce();
            m_publishRate.sleep();
        } // while (!m_publishingIsStopped && ros::ok())
    } // for (goal = path.begin(); goal != path.end(); ++goal)

    std_srvs::Empty empty_srv;
    ros::service::call(m_frame + "/finish", empty_srv);
    m_occupator->freeRegion();
} // run(std::vector<Goal> path)


void GoalsPublisher::runControlled() {
    ros::Subscriber subscriber = m_node.subscribe("/swarm/commands", 1, &GoalsPublisher::directionChanged, this);
    std::thread goingToGoalThr(&GoalsPublisher::goToGoal, this);

    ros::Rate loop(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop.sleep();
    }

    if (goingToGoalThr.joinable())
        goingToGoalThr.join();
}


void GoalsPublisher::directionChanged(const std_msgs::Byte::ConstPtr &direction) {
    m_direction = direction->data;
}


inline Goal GoalsPublisher::getGoal() {
    double x     = m_currPose.x();
    double y     = m_currPose.y();
    double z     = m_currPose.z();
    double roll  = m_currPose.roll();
    double pitch = m_currPose.pitch();
    double yaw   = m_currPose.yaw();

    // all parameters are measured in meters
    constexpr double movingStep = 0.1;
    constexpr double eps        = 0.2;

    auto moveByX = [=](double x, double slope) -> double {
        double shiftX = x + slope * movingStep;
        if (shiftX < (m_world->getOXMax() - eps) && shiftX > (m_world->getOXMin() + eps))
            x = shiftX;

        return x;
    };

    auto moveByY = [=](double y, double slope) -> double {
        double shiftY = y + slope * movingStep;
        if (shiftY < (m_world->getOYMax() - eps) && shiftY > (m_world->getOYMin() + eps))
            y = shiftY;

        return y;
    };

    auto moveByZ = [=](double z, double shift) -> double {
        double shiftZ = z + shift;
        if (shiftZ < (m_world->getOZMax() - eps) && shiftZ > (m_world->getOZMin() + eps))
            z = shiftZ;

        return z;
    };

    auto rotate = [=](double currAngle, double shift) -> double {
        constexpr double minAngle = degToRad(-180);
        constexpr double maxAngle = degToRad( 180);

        currAngle += ((currAngle + shift > maxAngle)? shift - 2 * maxAngle : shift);
        currAngle += ((currAngle + shift < minAngle)? shift - 2 * minAngle : shift);

        return currAngle;
    };

    if (commands::forward == m_direction) {
        x = moveByX(x, std::cos(yaw));
        y = moveByY(y, std::sin(yaw));
    }

    else if (commands::backward == m_direction) {
        x = moveByX(x, -std::cos(yaw));
        y = moveByY(y, -std::sin(yaw));
    }

    else if (commands::rightward == m_direction) {
        x = moveByX(x,  std::sin(yaw));
        y = moveByY(y, -std::cos(yaw));
    }

    else if (commands::leftward == m_direction) {
        x = moveByX(x, -std::sin(yaw));
        y = moveByY(y,  std::cos(yaw));
    }

    else if (commands::yawright == m_direction)
        yaw = rotate(yaw, degToRad(-10));

    else if (commands::yawleft == m_direction)
        yaw = rotate(yaw, degToRad( 10));

    else if (commands::upward == m_direction)
        z = moveByZ(z,  movingStep);

    else if (commands::downward == m_direction)
        z = moveByZ(z, -movingStep);

    else if (commands::takeoff == m_direction) {
        std_srvs::Empty empty_srv;
        ros::service::call(m_frame + "/takeoff", empty_srv);
        z = moveByZ(z, 1.0);
    }

    else if (commands::landing == m_direction) {
        std_srvs::Empty empty_srv;
        ros::service::call(m_frame + "/land", empty_srv);
    }

    m_direction = 0;

    if (x != m_currPose.x() || y != m_currPose.y() || z != m_currPose.z() || yaw != m_currPose.yaw())
        m_currPose = Pose(x, y, z, roll, pitch, yaw);
    
    return Goal(x, y, z, roll, pitch, yaw);
}


void GoalsPublisher::goToGoal() {
    m_currPose = Pose(m_currPose.x(), m_currPose.y(), m_currPose.z(), 0.0, 0.0, 0.0);

    while (ros::ok()) {
        Pose pose = getPose();

        // If m_direction != 0 then it is meant that we have got interrupt from the world
        if ((m_direction == 0) || m_publishingIsStopped || pose.isNull()) {
            m_publishRate.sleep();
            continue;
        }

        Goal goal = getGoal();
        m_occupator->updateXYZ(pose.x(), pose.y(), pose.z());

        // If m_direction != 0 then it is meant that we have got interrupt from the world
        while (((m_direction == 0) && !m_publishingIsStopped) &&
               !m_world->occupyRegion(*m_occupator, goal.x(), goal.y(), goal.z()))
        {
            //ROS_INFO("%s%s", m_frame.c_str(), " is waiting");
            m_publisher.publish(pose.msg());

            Pose exactPose = getPose();
            if (!exactPose.isNull())
                m_occupator->updateXYZ(exactPose.x(), exactPose.y(), exactPose.z());
            m_publishRate.sleep();
        }

        // Adjust the pose to the goal
        while ((m_direction == 0) && !m_publishingIsStopped) {
            m_publisher.publish(goal.msg());

            pose = getPose();
            if (!pose.isNull())
                m_occupator->updateXYZ(pose.x(), pose.y(), pose.z());
            m_publishRate.sleep();
        }
    } // while (ros::ok())
}