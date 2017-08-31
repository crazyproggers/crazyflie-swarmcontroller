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
        : m_roll    (roll)
        , m_pitch   (pitch)
        , m_yaw     (yaw)
        , m_delay   (delay)
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

    double x()      const   { return m_msg.pose.position.x; }
    double y()      const   { return m_msg.pose.position.y; }
    double z()      const   { return m_msg.pose.position.z; }
    double roll()   const   { return m_roll;                }
    double pitch()  const   { return m_pitch;               }
    double yaw()    const   { return m_yaw;                 }
    double delay()  const   { return m_delay;               }

    msg_t getMsg() {
        m_msg.header.seq++;
        m_msg.header.stamp = ros::Time::now();
        return m_msg;
    }
};


class GoalPublisher {
    std::string             m_worldFrame;
    std::string             m_frame;
    std::vector	<Goal>      m_goal;
    ros::Publisher          m_publisher;
    tf::TransformListener   m_transformListener;
    uint                    m_publishRate;

public:
    GoalPublisher(
        const std::string &worldFrame,
        const std::string &frame,
        const std::vector<Goal> &goal,
        uint publishRate)
        : m_worldFrame          (worldFrame)
        , m_frame               (frame)
        , m_goal                (goal)
        , m_publisher           ()
        , m_transformListener   ()
        , m_publishRate         (publishRate)
    {
        ros::NodeHandle n;
        m_transformListener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(5.0));
        m_publisher = n.advertise<msg_t>(m_frame + "/goal", 1);
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

                loop_rate.sleep();
            } // while (ros::ok())

        std_srvs::Empty empty_srv;
        ros::service::call(m_frame + "/land", empty_srv);
    } // run()
};

inline double fixCoordinate(double value, size_t num) {
    bool isAngle = ((num >= 3) && (num <= 5))? true : false;
    bool isPosition = (num < 3)? true : false;

    if (isAngle) {
        if (value > 180)
            value -= 360;
        else if (value < -180)
            value += 360;
    }
    else if (isPosition) {
        if (value > 2)
            value = 2;
        if (value < 0)
            value = 0;
    }

    return value;
}

std::vector<std::vector<Goal>> getGoals(
    const std::string &map_path,
    double intermediatePointsDelay  = 0.01,
    double distanceBetweenDots      = 0.01,
    double angleBetweenDots         = 18.0) {

    std::ifstream map;
    map.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    std::vector<std::vector<Goal>> goals_table;

    try {
        map.open(map_path.c_str(), std::ios_base::in);

        std::vector<std::vector<double>> anchor;
        std::vector<double> intermediateDots;
        
        const uint PARAMETERS_AMOUNT = 7;

        while (!map.eof()) {
            uint anchorPointsAmount;
            map >> anchorPointsAmount;
            
            // Get anchor points from map-file
            for (size_t i = 0; i < anchorPointsAmount; ++i) {
                std::vector<double> tmp;
                double anchorPointsValue;
                
                for (size_t j = 0; j < PARAMETERS_AMOUNT; ++j) {
                    map >> anchorPointsValue;
                    tmp.push_back(anchorPointsValue);
                }
                anchor.push_back(tmp);
            }

            for (size_t i = 0; i < anchor.size() - 1; ++i) {
                double delta_x = anchor[i][0] - anchor[i+1][0];
                double delta_y = anchor[i][1] - anchor[i+1][1];
                double delta_z = anchor[i][2] - anchor[i+1][2];
                double delta_yaw = abs(anchor[i][5] - anchor[i+1][5]);

                double intermediatePointsAmount = std::sqrt(std::pow(delta_x, 2) 
                                                          + std::pow(delta_y, 2) 
                                                          + std::pow(delta_z, 2))
                                                          / distanceBetweenDots;

                double intermediateAnglesAmount = delta_yaw / angleBetweenDots;
                uint   intermediateDotsAmount   = std::max(intermediatePointsAmount, intermediateAnglesAmount);
                
                if (!intermediateDotsAmount) 
                    continue;

                double distanceBetweenCordinates[PARAMETERS_AMOUNT];
                double qurentPosition[PARAMETERS_AMOUNT];

                for (size_t j = 0; j < anchor[0].size(); ++j) {
                    distanceBetweenCordinates[j] = anchor[i+1][j] - anchor[i][j];
                    qurentPosition[j] = anchor[i][j] + distanceBetweenCordinates[j] / intermediateDotsAmount;
                    anchor[i][j] = fixCoordinate(anchor[i][j], j);
                    intermediateDots.push_back(anchor[i][j]);
                }

                for (size_t k = 0; k < intermediateDotsAmount; ++k) {
                    for (size_t j = 0; j < PARAMETERS_AMOUNT - 1; ++j) {
                        qurentPosition[j] = fixCoordinate(qurentPosition[j], j);
                        intermediateDots.push_back(qurentPosition[j]);
                        qurentPosition[j] += distanceBetweenCordinates[j] / intermediateDotsAmount;

                    }
                    intermediateDots.push_back(intermediatePointsDelay);
                }
            } // for (size_t i = 0; i < anchor.size() - 1; ++i)

            for (size_t i = 0; i < anchor[0].size(); ++i) {
               anchor[anchor.size()-1][i] = fixCoordinate(anchor[anchor.size()-1][i], i);
               intermediateDots.push_back(anchor[anchor.size()-1][i]);
            }
            
            std::vector<Goal> entry;
            for (size_t i = 0; i < intermediateDots.size(); i += PARAMETERS_AMOUNT) {
                double &x     = intermediateDots[i];
                double &y     = intermediateDots[i+1];
                double &z     = intermediateDots[i+2];
                double &roll  = intermediateDots[i+3];
                double &pitch = intermediateDots[i+4];
                double &yaw   = intermediateDots[i+5];
                double &delay = intermediateDots[i+6];
                entry.push_back(Goal(x, y, z, degToRad(roll), degToRad(pitch), degToRad(yaw), delay));
            }
            goals_table.push_back(entry);
       } // while ((map >> anchorPointsAmount) > 0)
       map.close();
   } // try
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

    // Read goals from map-file and to interpolate they
    double intermediatePointsDelay;
    double distanceBetweenDots;
    double angleBetweenDots;
    n.getParam("intermediatePointsDelay",   intermediatePointsDelay);
    n.getParam("distanceBetweenDots",       distanceBetweenDots);
    n.getParam("angleBetweenDots",          angleBetweenDots);

    std::vector<std::vector<Goal>> goals = std::move(getGoals(map_path, intermediatePointsDelay, distanceBetweenDots, angleBetweenDots));
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
