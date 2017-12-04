#ifndef GOAL_H
#define GOAL_H

#include <tf/transform_datatypes.h>

// This class provides a wrapper over geometry_msgs::PoseStamped

class Goal: private geometry_msgs::PoseStamped {
    double m_roll;   // angle around OX
    double m_pitch;  // angle around OY
    double m_yaw;    // angle around OZ
    double m_delay;  // waiting at point (by default is zero)
    bool   m_empty;  // true if goal was not denote (by default is false)

public:
    Goal() : m_empty(true) {}

    Goal(
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw,
        double delay = 0.0)
        : m_roll    (roll)
        , m_pitch   (pitch)
        , m_yaw     (yaw)
        , m_delay   (delay)
        , m_empty   (false)
    {
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(roll, pitch, yaw);

        header.seq         = 0;
        header.stamp       = ros::Time(0);
        pose.position.x    = x;
        pose.position.y    = y;
        pose.position.z    = z;
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();
    }

    Goal(const Goal &goal)
        : m_roll    (goal.m_roll)
        , m_pitch   (goal.m_pitch)
        , m_yaw     (goal.m_yaw)
        , m_delay   (goal.m_delay)
        , m_empty   (false)
    {
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(m_roll, m_pitch, m_yaw);

        header.seq         = 0;
        header.stamp       = ros::Time(0);
        pose.position.x    = goal.pose.position.x;
        pose.position.y    = goal.pose.position.y;
        pose.position.z    = goal.pose.position.z;
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();
    }

    Goal & operator=(const Goal &goal) {
        if (this != &goal)
            Goal(goal);

        return *this;
    }

    double x()        const   { return pose.position.x; }
    double y()        const   { return pose.position.y; }
    double z()        const   { return pose.position.z; }
    double roll()     const   { return m_roll;          }
    double pitch()    const   { return m_pitch;         }
    double yaw()      const   { return m_yaw;           }
    double delay()    const   { return m_delay;         }
    bool   empty()    const   { return m_empty;         }

    geometry_msgs::PoseStamped getMsg() {
        ++header.seq;
        header.stamp = ros::Time::now();
        return static_cast<geometry_msgs::PoseStamped>(*this);
    }
};

#endif // GOAL_H