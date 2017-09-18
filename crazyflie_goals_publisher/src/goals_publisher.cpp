#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include "goals_publisher.h"

constexpr double degToRad(double deg) {
    return deg / 180.0 * M_PI;
}

std::atomic<uint> GoalsPublisher::m_amountCrazyfliesAtAnchors(0);

GoalsPublisher::GoalsPublisher(
    const std::string &worldFrame,
    const std::string &frame,
    const std::vector<Goal> &goal,
    uint publishRate,
    bool waitForAllAtAnchor,
    uint totalCrazyflies)
    : m_worldFrame                (worldFrame)
    , m_frame                     (frame)
    , m_goal                      (goal)
    , m_publisher                 ()
    , m_transformListener         ()
    , m_publishRate               (publishRate)
    , m_waitForAllAtAnchor        (waitForAllAtAnchor)
    , m_totalCrazyflies           (totalCrazyflies)
{
    ros::NodeHandle n;
    m_transformListener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(5.0));
    m_publisher = n.advertise<msg_t>(m_frame + "/goal", 1);
}

void GoalsPublisher::run() {
    ros::Rate loop_rate(m_publishRate);

    for (size_t i = 0; i < m_goal.size(); ++i) {
        while (ros::ok()) {
            m_publisher.publish(m_goal[i].getMsg());

            ros::Time common_time;
            std::string err_msg;
            m_transformListener.getLatestCommonTime(m_worldFrame, m_frame, common_time, &err_msg);
            if (err_msg.size()) std::cerr << err_msg << std::endl;

            if (m_transformListener.canTransform(m_worldFrame, m_frame, common_time)) {
                tf::StampedTransform position;
                m_transformListener.lookupTransform(m_worldFrame, m_frame, common_time, position);

                double curr_x = position.getOrigin().x();
                double curr_y = position.getOrigin().y();
                double curr_z = position.getOrigin().z();

                double curr_roll, curr_pitch, curr_yaw;
                tf::Quaternion quaternion = position.getRotation();
                tf::Matrix3x3(quaternion).getRPY(curr_roll, curr_pitch, curr_yaw);

                if ((fabs(curr_x     - m_goal[i].x()) < 0.3) &&
                    (fabs(curr_y     - m_goal[i].y()) < 0.3) &&
                    (fabs(curr_z     - m_goal[i].z()) < 0.3) &&
                    (fabs(curr_roll  - m_goal[i].roll())  < degToRad(10)) &&
                    (fabs(curr_pitch - m_goal[i].pitch()) < degToRad(10)) &&
                    (fabs(curr_yaw   - m_goal[i].yaw())   < degToRad(10)))
                {
                    ros::Duration(m_goal[i].delay()).sleep();
                    break; // go to next goal
                }
            } // if (canTransform(m_worldFrame, m_frame, common_time))
         
            loop_rate.sleep();
        } // while (ros::ok())

        if (m_waitForAllAtAnchor && m_goal[i].isAnchor()) {
            if (m_amountCrazyfliesAtAnchors == m_totalCrazyflies)
                m_amountCrazyfliesAtAnchors = 0;
            m_amountCrazyfliesAtAnchors++;

            while (ros::ok()) {
                m_publisher.publish(m_goal[i].getMsg());
                if (m_amountCrazyfliesAtAnchors == m_totalCrazyflies) break;
                loop_rate.sleep();
            }
        } // if (m_waitForAllAtAnchor && m_goal[i].isAnchor())
    } // for (size_t i = 0; i < m_goal.size(); ++i)

    std_srvs::Empty empty_srv;
    ros::service::call(m_frame + "/land", empty_srv);
} // run()

