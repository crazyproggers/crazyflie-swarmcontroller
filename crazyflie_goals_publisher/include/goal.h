#ifndef GOAL_H
#define GOAL_H

#include <tf/transform_datatypes.h>


// This class provides a wrapper over geometry_msgs::PoseStamped
class Pose: private geometry_msgs::PoseStamped {
    double m_roll;   // angle around OX
    double m_pitch;  // angle around OY
    double m_yaw;    // angle around OZ

public:
    Pose() {}

    Pose(
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw)
        : m_roll    (roll)
        , m_pitch   (pitch)
        , m_yaw     (yaw)
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

    Pose(const Pose &newPose)
        : m_roll    (newPose.m_roll)
        , m_pitch   (newPose.m_pitch)
        , m_yaw     (newPose.m_yaw)
    {
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(m_roll, m_pitch, m_yaw);

        header.seq         = 0;
        header.stamp       = ros::Time(0);
        pose.position.x    = newPose.pose.position.x;
        pose.position.y    = newPose.pose.position.y;
        pose.position.z    = newPose.pose.position.z;
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();
        pose.orientation.w = quaternion.w();
    }

    Pose & operator=(const Pose &newPose) {
        if (this != &newPose)
            Pose(newPose);

        return *this;
    }

    double x()        const   { return pose.position.x; }
    double y()        const   { return pose.position.y; }
    double z()        const   { return pose.position.z; }
    double roll()     const   { return m_roll;          }
    double pitch()    const   { return m_pitch;         }
    double yaw()      const   { return m_yaw;           }

    geometry_msgs::PoseStamped getMsg() {
        ++header.seq;
        header.stamp = ros::Time::now();
        return static_cast<geometry_msgs::PoseStamped>(*this);
    }
};


class Goal: public Pose {
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
        : Pose      (x, y, z, roll, pitch, yaw)
        , m_delay   (delay)
        , m_empty   (false)
    {}

    Goal (const Goal &goal)
        : Pose    (goal.x(), goal.y(), goal.z(), goal.roll(), goal.pitch(), goal.yaw())
        , m_delay (goal.m_delay)
        , m_empty (goal.m_empty)
    {}

    Goal & operator=(const Goal &goal) {
        if (this != &goal)
            Goal(goal);

        return *this;
    }

    Goal (const Pose &pose)
        : Pose    (pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw())
        , m_delay (0.0)
        , m_empty (false)
    {}

    Goal & operator=(const Pose &pose) {
        if (this != &pose)
            Goal(pose);

        return *this;
    }

    double delay()    const   { return m_delay; }
    bool   empty()    const   { return m_empty; }
};

#endif // GOAL_H