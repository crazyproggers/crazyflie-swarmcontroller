#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include "goals_publisher.h"


constexpr double degToRad(double deg) {
    return deg / 180.0 * M_PI;
}


// Static variables
std::atomic<uint> GoalsPublisher::m_amountCrazyfliesAtAnchors(0);
uint GoalsPublisher::m_totalCrazyflies = 0;


GoalsPublisher::GoalsPublisher(
    const std::string &worldFrame,
    const std::string &frame,
    uint publishRate)
    : m_nh                        ()
    , m_worldFrame                (worldFrame)
    , m_frame                     (frame)
    , m_publisher                 ()
    , m_subscriber                ()
    , m_transformListener         ()
    , m_loopRate                  (publishRate)
    , m_errMutex                  ()
    , m_synchAtAnchors            (false)
{
    m_transformListener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(5.0));
    m_publisher = m_nh.advertise<msg_t>(m_frame + "/goal", 1);
    m_totalCrazyflies++; // register new crazyflie
}


void GoalsPublisher::enableSynchAtAnchors() {
    m_synchAtAnchors = true;
}


void GoalsPublisher::run(std::vector<Goal> path) {
    for (Goal goal: path) {
        while (ros::ok()) {
            m_publisher.publish(goal.getMsg());

            ros::Time common_time;
            std::string errMsg;
            m_transformListener.getLatestCommonTime(m_worldFrame, m_frame, common_time, &errMsg);

            if (errMsg.size()) {
                std::lock_guard<std::mutex> locker(m_errMutex);
                std::cerr << errMsg << std::endl;
            }

            if (m_transformListener.canTransform(m_worldFrame, m_frame, common_time)) {
                tf::StampedTransform position;
                m_transformListener.lookupTransform(m_worldFrame, m_frame, common_time, position);

                double curr_x = position.getOrigin().x();
                double curr_y = position.getOrigin().y();
                double curr_z = position.getOrigin().z();

                double curr_roll, curr_pitch, curr_yaw;
                tf::Quaternion quaternion = position.getRotation();
                tf::Matrix3x3(quaternion).getRPY(curr_roll, curr_pitch, curr_yaw);

                if ((fabs(curr_x     - goal.x()) < 0.3) &&
                    (fabs(curr_y     - goal.y()) < 0.3) &&
                    (fabs(curr_z     - goal.z()) < 0.3) &&
                    (fabs(curr_roll  - goal.roll())  < degToRad(10)) &&
                    (fabs(curr_pitch - goal.pitch()) < degToRad(10)) &&
                    (fabs(curr_yaw   - goal.yaw())   < degToRad(10)))
                {
                    ros::Duration(goal.delay()).sleep();
                    break; // go to next goal
                }
            } // if (canTransform(m_worldFrame, m_frame, common_time))
         
            m_loopRate.sleep();
        } // while (ros::ok())

        if (m_synchAtAnchors && goal.isAnchor()) {
            if (m_amountCrazyfliesAtAnchors == m_totalCrazyflies)
                m_amountCrazyfliesAtAnchors = 0;
            m_amountCrazyfliesAtAnchors++;

            while (ros::ok()) {
                m_publisher.publish(goal.getMsg());
                if (m_amountCrazyfliesAtAnchors == m_totalCrazyflies) break;
                m_loopRate.sleep();
            }
        } // if (m_waitForAllAtAnchor && goal.isAnchor())
    } // for (Goal goal: path)

    std_srvs::Empty empty_srv;
    ros::service::call(m_frame + "/land", empty_srv);
} // run(std::vector<Goal> path)


void GoalsPublisher::run() {
    m_subscriber = m_nh.subscribe("/swarm/commands", 1, &GoalsPublisher::checkCommand, this);
}


void GoalsPublisher::checkCommand(const std_msgs::String::ConstPtr &msg) {
    /*
     * COMMANDS:
     * up           -- Z += 0.01;
     * down         -- Z -= 0.01;
     * left         -- X += 0.01;
     * right        -- X -= 0.01;
     * forward      -- Y += 0.01;
     * backward     -- Y -= 0.01;
     */
}

