#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <thread>

typedef unsigned int uint;
typedef geometry_msgs::PoseStamped msg_t;

inline double degToRad(double deg) {
    return deg / 180.0 * M_PI;
}


class Goal {
    double m_roll;
    double m_pitch;
    double m_yaw;
    double m_delay;
    msg_t  m_msg;

public:
    Goal(
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw,
        double delay)
        : m_roll 	(roll)
        , m_pitch 	(pitch)
        , m_yaw 	(yaw)
        , m_delay 	(delay)
    {
        /*
         * roll  - angle around X
         * pitch - angle around Y
         * yaw   - angle around Z
         */
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

    double x()      const 	{ return m_msg.pose.position.x;    }
    double y()      const 	{ return m_msg.pose.position.y;    }
    double z() 	     const	{ return m_msg.pose.position.z;    }
    double roll()   const 	{ return m_roll; 					 }
    double pitch()  const 	{ return m_pitch; 					 }
    double yaw()    const 	{ return m_yaw; 					 }
    double delay()  const 	{ return m_delay;                  }

    msg_t getMsg() {
        m_msg.header.seq++;
        m_msg.header.stamp = ros::Time::now();
        return m_msg;
    }
};


class GoalPublisher {
    std::string 			m_worldFrame;
    std::string 			m_frame;
    std::vector	<Goal>		m_goal;
    ros::Publisher 		m_publisher;
    tf::TransformListener 	m_transformListener;
    uint					m_publishRate;

public:
    GoalPublisher(
        const std::string &worldFrame,
        const std::string &frame,
        const std::vector<Goal> &goal,
        uint publishRate)
        : m_worldFrame  		(worldFrame)
        , m_frame       		(frame)
        , m_goal				(goal)
        , m_publisher 			()
        , m_transformListener 	()
        , m_publishRate		(publishRate)
    {
        ros::NodeHandle n;
        m_transformListener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(5.0));
        m_publisher = n.advertise<msg_t>(m_frame + "/goal", 1);
        ros::service::waitForService(m_frame + "/land");
    }

    void run() {
        ros::Rate loop_rate(m_publishRate);

        for (size_t i = 0; i < m_goal.size(); ++i)
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

                //ros::Rate(m_publishRate).sleep();
                loop_rate.sleep();
            } // while (ros::ok())

        std_srvs::Empty empty_srv;
        ros::service::call(m_frame + "/land", empty_srv);
    } // run()
};


std::vector<std::vector<Goal>> readGoals(const std::string &map_path) {
    std::ifstream map;
    map.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    std::vector<std::vector<Goal>> goals_table;

    try {
        map.open(map_path.c_str(), std::ios_base::in);

        while (!map.eof()) {
            std::vector<Goal> entry;
            uint goalsNum;
            map >> goalsNum;

            for (size_t i = 0; i < goalsNum; ++i) {
                double x, y, z, pitch, roll, yaw, delay;
                map >> x >> y >> z >> roll >> pitch >> yaw >> delay;
                entry.push_back(Goal(x, y, z, degToRad(roll), degToRad(pitch), degToRad(yaw), delay));
            }
            goals_table.push_back(entry);
        } // while (!map.eof())
        map.close();
    }
    catch (std::ifstream::failure exp) {
        std::cerr << "Could not read/close map-file!" << std::endl;
    }

    return goals_table;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "goals_publisher");

    // Read parameters
    ros::NodeHandle n("~");
    std::string worldFrame;
    n.param<std::string>("worldFrame", worldFrame, "/world");

    std::string frames_str;
    n.getParam("frames", frames_str);

    // Split frames_str by whitespace
    std::istringstream iss(frames_str);
    std::vector<std::string> frame {std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};

    std::string map_path;
    n.getParam("map", map_path);
    std::vector<std::vector<Goal>> goals = readGoals(map_path);
    if (!goals.size()) return -1;

    int rate;
    n.getParam("rate", rate);

    GoalPublisher *goalPublisher[frame.size()];
    std::thread   *thr[frame.size()];
    for (size_t i = 0; i < frame.size(); ++i) {
        goalPublisher[i] = new GoalPublisher(worldFrame, frame[i], goals[i], rate);
        thr[i] = new std::thread(&GoalPublisher::run, goalPublisher[i]);
    }

    for (size_t i = 0; i < frame.size(); ++i) {
        thr[i]->join();
        delete thr[i];
        delete goalPublisher[i];
    }

    return 0;
}
