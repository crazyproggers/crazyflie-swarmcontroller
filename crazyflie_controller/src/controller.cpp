#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <memory>

#include "pid.hpp"

template<typename T, typename ...Args>
std::unique_ptr<T> make_unique( Args&& ...args ) {
    return std::unique_ptr<T>(new T( std::forward<Args>(args)... ));
}

double get(
    const ros::NodeHandle &node,
    const std::string &name) {
    double value;
    node.getParam(name, value);
    return value;
}

class Controller {
public:
    Controller(
        const  std::string &worldFrame,
        const  std::string &frame,
        const  ros::NodeHandle &node,
        double frequency = 50.0)
        : m_worldFrame  (worldFrame)
        , m_frame       (frame)
        , m_pubNav      ()
        , m_listener    ()
        , m_pidX(
            get(node, "PIDs/X/kp"),
            get(node, "PIDs/X/kd"),
            get(node, "PIDs/X/ki"),
            get(node, "PIDs/X/minOutput"),
            get(node, "PIDs/X/maxOutput"),
            get(node, "PIDs/X/integratorMin"),
            get(node, "PIDs/X/integratorMax"),
            "x")
        , m_pidY(
            get(node, "PIDs/Y/kp"),
            get(node, "PIDs/Y/kd"),
            get(node, "PIDs/Y/ki"),
            get(node, "PIDs/Y/minOutput"),
            get(node, "PIDs/Y/maxOutput"),
            get(node, "PIDs/Y/integratorMin"),
            get(node, "PIDs/Y/integratorMax"),
            "y")
        , m_pidZ(
            get(node, "PIDs/Z/kp"),
            get(node, "PIDs/Z/kd"),
            get(node, "PIDs/Z/ki"),
            get(node, "PIDs/Z/minOutput"),
            get(node, "PIDs/Z/maxOutput"),
            get(node, "PIDs/Z/integratorMin"),
            get(node, "PIDs/Z/integratorMax"),
            "z")
        , m_pidYaw(
            get(node, "PIDs/Yaw/kp"),
            get(node, "PIDs/Yaw/kd"),
            get(node, "PIDs/Yaw/ki"),
            get(node, "PIDs/Yaw/minOutput"),
            get(node, "PIDs/Yaw/maxOutput"),
            get(node, "PIDs/Yaw/integratorMin"),
            get(node, "PIDs/Yaw/integratorMax"),
            "yaw")
        , m_state           (Idle)
        , m_goal            ()
        , m_subscribeGoal   ()
        , m_subscribeBattery()
        , m_serviceTakeoff  ()
        , m_serviceLand     ()
        , m_thrust          (0)
        , m_startZ          (0)
    {
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(5.0));

        m_pubNav           = nh.advertise<geometry_msgs::Twist>(m_frame + "/cmd_vel", 1);
        m_subscribeGoal    = nh.subscribe       (m_frame + "/goal",    1, &Controller::goalChanged,  this);
        m_subscribeBattery = nh.subscribe       (m_frame + "/battery", 1, &Controller::checkBattery, this);
        m_serviceTakeoff   = nh.advertiseService(m_frame + "/takeoff",    &Controller::takeoff,      this);
        m_serviceLand      = nh.advertiseService(m_frame + "/land",       &Controller::land,         this);

        m_runThread = std::thread(&Controller::run, this, frequency);
    }

    ~Controller() {
        m_runThread.join();
    }

private:
    void run(double frequency) {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0 / frequency), &Controller::iteration, this);

        ros::Rate loop(10);
        while (ros::ok()) {
            ros::spinOnce();
            loop.sleep();
        }
    }

    void goalChanged(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        m_goal = *msg;
    }

    void checkBattery(const std_msgs::Float32::ConstPtr &msg) {
        float cur_lvl = msg->data;
        float low_lvl = 2.6;

        if (cur_lvl <= low_lvl) {
            ROS_WARN("%s%s", m_frame.c_str(), " have too low battery charge level! You should to charge it.");
            m_state = Landing;
        }
    }

    bool takeoff(
        std_srvs::Empty::Request  &req,
        std_srvs::Empty::Response &res)
    {
        ROS_INFO("%s%s", m_frame.c_str(), ": takeoff requested!");
        m_state = TakingOff;

        tf::StampedTransform transform;
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
        m_startZ = transform.getOrigin().z();

        return true;
    }

    bool land(
        std_srvs::Empty::Request  &req,
        std_srvs::Empty::Response &res)
    {
        ROS_INFO("%s%s", m_frame.c_str(), ": landing requested!");
        m_state = Landing;

        return true;
    }

    void pidReset() {
        m_pidX.reset();
        m_pidY.reset();
        m_pidZ.reset();
        m_pidYaw.reset();
    }

    void iteration(const ros::TimerEvent &e) {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch (m_state) {
            case TakingOff: {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

                if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 50000) {
                    pidReset();
                    m_pidZ.setIntegral(m_thrust / m_pidZ.ki());
                    m_state = Automatic;
                    m_thrust = 0;
                } else {
                    m_thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }

                std_srvs::Empty empty_srv;
                ros::service::call(m_frame + "/start_publishing", empty_srv);
            } // case TakingOff:
            break;

            case Landing: {
                m_goal.pose.position.z = m_startZ + 0.05;
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

                if (transform.getOrigin().z() <= m_startZ + 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);

                    std_srvs::Empty empty_srv;
                    ros::service::call(m_frame + "/stop_publishing", empty_srv);
                }
            }

            case Automatic: {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                geometry_msgs::PoseStamped targetWorld;

                targetWorld.header.stamp    = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                targetWorld.pose            = m_goal.pose;

                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_frame, targetWorld, targetDrone);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

                geometry_msgs::Twist msg;
                msg.linear.x  = m_pidX.update  (0.0, targetDrone.pose.position.x);
                msg.linear.y  = m_pidY.update  (0.0, targetDrone.pose.position.y);
                msg.linear.z  = m_pidZ.update  (0.0, targetDrone.pose.position.z);
                msg.angular.z = m_pidYaw.update(0.0, yaw);
                m_pubNav.publish(msg);
            }
            break;

            case Idle: {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
            break;
        } // switch (m_state)
    }

private:
    enum State {
        Idle      = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing   = 3
    };

private:
    std::string                 m_worldFrame;
    std::string                 m_frame;
    ros::Publisher              m_pubNav;
    tf::TransformListener       m_listener;
    std::thread                 m_runThread;
    PID                         m_pidX;
    PID                         m_pidY;
    PID                         m_pidZ;
    PID                         m_pidYaw;
    State                       m_state;
    geometry_msgs::PoseStamped  m_goal;
    ros::Subscriber             m_subscribeGoal;
    ros::Subscriber             m_subscribeBattery;
    ros::ServiceServer          m_serviceTakeoff;
    ros::ServiceServer          m_serviceLand;
    float                       m_thrust;
    float                       m_startZ;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");

    // Read parameters
    ros::NodeHandle node("~");
    std::string worldFrame;
    node.param<std::string>("worldFrame", worldFrame, "/world");

    std::string frames_str;
    node.getParam("/swarm/frames", frames_str);

    // Split frames_str by whitespace
    std::istringstream iss(frames_str);
    std::vector<std::string> frames {std::istream_iterator<std::string>{iss},
                                     std::istream_iterator<std::string>{}};

    double frequency;
    node.param("frequency", frequency, 50.0);

    std::vector<std::unique_ptr<Controller>> controllers;

    for (size_t i = 0; i < frames.size(); ++i)
        controllers.push_back(make_unique<Controller>(worldFrame, frames[i], node, frequency));

    while (ros::ok())
        ros::Rate(1).sleep();

    return 0;
}
