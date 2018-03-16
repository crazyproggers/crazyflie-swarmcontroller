#ifndef GOAL_H
#define GOAL_H

#include <tf/transform_datatypes.h>


// This class provides a wrapper over geometry_msgs::PoseStamped
class Pose: protected geometry_msgs::PoseStamped {
protected:
    double m_roll;    // angle around OX (in radians)
    double m_pitch;   // angle around OY (in radians)
    double m_yaw;     // angle around OZ (in radians)
    bool   m_isNull;  // true if was called default constuctor

    // Copy original to *this
    void copy(const Pose &original) noexcept {
        m_roll             = original.m_roll;
        m_pitch            = original.m_pitch;
        m_yaw              = original.m_yaw;
        m_isNull           = original.m_isNull;
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

    void setNull(Pose &pose) noexcept {
        pose.m_isNull      = true;
        pose.header.seq    = 0;
        pose.header.stamp  = ros::Time(0);
    }

    // Move original to *this
    void move(Pose &original) noexcept {
        copy(original);
        setNull(original);
    }

    bool isEqual(const Pose &pose1, const Pose &pose2) const noexcept {
        return (pose1.x()     == pose2.x())     &&
               (pose1.y()     == pose2.y())     &&
               (pose1.z()     == pose2.z())     &&
               (pose1.m_roll  == pose2.m_roll)  &&
               (pose1.m_pitch == pose2.m_pitch) &&
               (pose1.m_yaw   == pose2.m_yaw);
    }

public:
    Pose(): m_isNull            (true)
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
        , m_isNull              (false)
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
        if (this != &pose) {
            if (!pose.m_isNull) copy(pose);
            else setNull(*this);
        }
    }

    Pose & operator=(const Pose &pose) {
        if (this != &pose) {
            if (!pose.m_isNull) copy(pose);
            else setNull(*this);
        }

        return *this;
    }

    Pose(Pose &&pose) {
        if (this != &pose) {
            if (!pose.m_isNull) move(pose);
            else setNull(*this);
        }
    }

    Pose & operator=(Pose &&pose) {
        if (this != &pose) {
            if (!pose.m_isNull) move(pose);
            else setNull(*this);
        }

        return *this;
    }

    bool operator==(const Pose &pose) const noexcept {
        return isEqual(*this, pose);
    }

    bool operator!=(const Pose &pose) const noexcept {
        return !isEqual(*this, pose);
    }

    double x()        const noexcept  { return pose.position.x; }
    double y()        const noexcept  { return pose.position.y; }
    double z()        const noexcept  { return pose.position.z; }
    double roll()     const noexcept  { return m_roll;          }
    double pitch()    const noexcept  { return m_pitch;         }
    double yaw()      const noexcept  { return m_yaw;           }
    bool   isNull()   const noexcept  { return m_isNull;        }

    geometry_msgs::PoseStamped msg() {
        ++header.seq;
        header.stamp = ros::Time::now();
        return *this;
    }
};


class Goal: public Pose {
    double m_delay = 0.0;  // waiting at point (by default is zero)

    // Move goal to *this
    void move(Goal &goal) noexcept {
        copy(goal);
        m_delay = goal.m_delay;
        setNull(goal);
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

    Goal(const Goal &goal) {
        if (this != &goal) {
            if (!goal.m_isNull) {
                copy(goal);
                m_delay = goal.m_delay;
            }
            else setNull(*this);
        }
    }

    Goal & operator=(const Goal &goal) {
        if (this != &goal) {
            if (!goal.m_isNull) {
                copy(goal);
                m_delay = goal.m_delay;
            }
            else setNull(*this);
        }

        return *this;
    }

    Goal(Goal &&goal) {
        if (this != &goal) {
            if (!goal.m_isNull) move(goal);
            else setNull(*this);
        }
    }

    Goal & operator=(Goal &&goal) {
        if (this != &goal) {
            if (!goal.m_isNull) move(goal);
            else setNull(*this);
        }

        return *this;
    }

    bool operator==(const Goal &goal) const {
        return isEqual(*this, goal);
    }

    bool operator!=(const Goal &goal) const {
        return !isEqual(*this, goal);
    }

    double delay() const noexcept { return m_delay; }
};

#endif // GOAL_H
