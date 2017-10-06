#ifndef GOAL_H
#define GOAL_H

class Goal {
    typedef geometry_msgs::PoseStamped msg_t;

    /*
     * roll  - angle around X
     * pitch - angle around Y
     * yaw   - angle around Z
     */
    double m_roll;
    double m_pitch;
    double m_yaw;
    double m_delay;
    bool   m_isAnchor;
    msg_t  m_msg;

public:
    Goal() = delete;

    Goal(
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw,
        double delay = 0.0,
        bool   isAnchor = false)
        : m_roll    (roll)
        , m_pitch   (pitch)
        , m_yaw     (yaw)
        , m_delay   (delay)
        , m_isAnchor(isAnchor)
    {
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(roll, pitch, yaw);

        m_msg.header.seq         = 0;
        m_msg.header.stamp       = ros::Time(0);
        m_msg.pose.position.x    = x;
        m_msg.pose.position.y    = y;
        m_msg.pose.position.z    = z;
        m_msg.pose.orientation.x = quaternion.x();
        m_msg.pose.orientation.y = quaternion.y();
        m_msg.pose.orientation.z = quaternion.z();
        m_msg.pose.orientation.w = quaternion.w();
    }

    Goal(const Goal &goal)
        : m_roll    (goal.m_roll)
        , m_pitch   (goal.m_pitch)
        , m_yaw     (goal.m_yaw)
        , m_delay   (goal.m_delay)
        , m_isAnchor(goal.m_isAnchor)
    {
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(m_roll, m_pitch, m_yaw);

        m_msg.header.seq         = 0;
        m_msg.header.stamp       = ros::Time(0);
        m_msg.pose.position.x    = goal.x();
        m_msg.pose.position.y    = goal.y();
        m_msg.pose.position.z    = goal.z();
        m_msg.pose.orientation.x = quaternion.x();
        m_msg.pose.orientation.y = quaternion.y();
        m_msg.pose.orientation.z = quaternion.z();
        m_msg.pose.orientation.w = quaternion.w();
    }

    Goal & operator=(const Goal &goal) {
        if (this != &goal) {
            this->~Goal();
            new (this) Goal(goal);
        }
        return *this;
    }

    double x()        const   { return m_msg.pose.position.x; }
    double y()        const   { return m_msg.pose.position.y; }
    double z()        const   { return m_msg.pose.position.z; }
    double roll()     const   { return m_roll;                }
    double pitch()    const   { return m_pitch;               }
    double yaw()      const   { return m_yaw;                 }
    double delay()    const   { return m_delay;               }
    bool   isAnchor() const   { return m_isAnchor;            }

    msg_t getMsg() {
        m_msg.header.seq++;
        m_msg.header.stamp = ros::Time::now();
        return m_msg;
    }
};

#endif // GOAL_H
