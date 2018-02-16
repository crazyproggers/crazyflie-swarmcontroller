#ifndef GOAL_H
#define GOAL_H

#include <tf/transform_datatypes.h>


// This class provides a wrapper over geometry_msgs::PoseStamped
class Pose: protected geometry_msgs::PoseStamped {
protected:
    double m_roll;   // angle around OX (in radians)
    double m_pitch;  // angle around OY (in radians)
    double m_yaw;    // angle around OZ (in radians)
    bool   m_empty;  // true if pose is not denoted

    // Copy original to *this
    void copy(const Pose &original) noexcept {
        m_roll             = original.m_roll;
        m_pitch            = original.m_pitch;
        m_yaw              = original.m_yaw;
        m_empty            = original.m_empty;
        header.seq         = original.header.seq;
        header.stamp       = original.header.stamp;
        pose.position.x    = original.pose.position.x;
        pose.position.y    = original.pose.position.y;
        pose.position.z    = original.pose.position.z;
        pose.orientation.x = original.pose.orientation.x;
        pose.orientation.y = original.pose.orientation.y;
        pose.orientation.z = original.pose.orientation.z;
        pose.orientation.w = original.pose.orientation.w;
    }

    // Move original to *this
    void move(Pose &original) noexcept {
        copy(original);

        original.m_empty        = true;
        original.header.seq     = 0;
        original.header.stamp   = ros::Time(0);
    }

public:
    Pose(): m_empty             (true)
    {
        header.seq              = 0;
        header.stamp            = ros::Time(0);
    }

    Pose(
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw)
        : m_roll                (roll)
        , m_pitch               (pitch)
        , m_yaw                 (yaw)
        , m_empty               (false)
    {
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(roll, pitch, yaw);

        header.seq              = 0;
        header.stamp            = ros::Time(0);
        pose.position.x         = x;
        pose.position.y         = y;
        pose.position.z         = z;
        pose.orientation.x      = quaternion.x();
        pose.orientation.y      = quaternion.y();
        pose.orientation.z      = quaternion.z();
        pose.orientation.w      = quaternion.w();
    }

    Pose(const Pose &pose) {
        copy(pose);
    }

    Pose & operator=(const Pose &pose) {
        if (this != &pose)
            copy(pose);

        return *this;
    }

    Pose(Pose &&pose) {
        move(pose);
    }

    Pose & operator=(Pose &&pose) {
        if (this != &pose)
            move(pose);

        return *this;
    }

    double x()        const noexcept  { return pose.position.x; }
    double y()        const noexcept  { return pose.position.y; }
    double z()        const noexcept  { return pose.position.z; }
    double roll()     const noexcept  { return m_roll;          }
    double pitch()    const noexcept  { return m_pitch;         }
    double yaw()      const noexcept  { return m_yaw;           }
    bool   empty()    const noexcept  { return m_empty;         }

    geometry_msgs::PoseStamped msg() {
        ++header.seq;
        header.stamp = ros::Time::now();
        return static_cast<geometry_msgs::PoseStamped>(*this);
    }
};


class Goal: public Pose {
    double m_delay;  // waiting at point (by default is zero)

    // Move goal to *this
    void move(Goal &goal) {
        copy(static_cast<Pose>(goal));

        goal.m_empty        = true;
        goal.m_delay        = 0.0;
        goal.header.seq     = 0;
        goal.header.stamp   = ros::Time(0);
    }

public:
    Goal() : Pose() {}

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
    {}

    Goal(const Goal &goal)
        : Pose    (goal.x(), goal.y(), goal.z(), goal.roll(), goal.pitch(), goal.yaw())
        , m_delay (goal.m_delay)
    {}

    Goal & operator=(const Goal &goal) {
        if (this != &goal) {
            Pose pose(goal.x(), goal.y(), goal.z(), goal.roll(), goal.pitch(), goal.yaw());
            copy(pose);
            m_delay = goal.m_delay;
        }

        return *this;
    }

    Goal(Goal &&goal): m_delay (goal.m_delay) {
        move(goal);
    }

    Goal & operator=(Goal &&goal) {
        if (this != &goal) {
            m_delay = goal.m_delay;
            move(goal);
        }

        return *this;
    }

    Goal(const Pose &pose)
        : Pose    (pose.x(), pose.y(), pose.z(), pose.roll(), pose.pitch(), pose.yaw())
        , m_delay (0.0)
    {}

    Goal & operator=(const Pose &pose) {
        if (this != &pose) {
            copy(pose);
            m_delay = 0.0;
        }

        return *this;
    }

    Goal(Pose &&pose)               = delete;
    Goal & operator=(Pose &&pose)   = delete;

    double delay() const noexcept { return m_delay; }
};

#endif // GOAL_H