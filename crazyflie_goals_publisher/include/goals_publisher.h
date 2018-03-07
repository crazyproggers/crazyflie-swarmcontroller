#ifndef GOALS_PUBLISHER_H
#define GOALS_PUBLISHER_H

#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Byte.h>
#include <thread>
#include "goal.h"

class World;
class Occupator;


class GoalsPublisher {
    ros::NodeHandle               m_node;
    std::string                   m_worldFrame;
    std::string                   m_frame;
    std::shared_ptr<Occupator>    m_occupator;
    ros::Publisher                m_publisher;
    tf::TransformListener         m_listener;
    std::thread                   m_runThread;

    ros::ServiceServer            m_stopPublishing;
    ros::ServiceServer            m_startPublishing;
    bool                          m_publishingIsStopped;
    ros::Rate                     m_publishRate;

    int8_t                        m_direction;
    Pose                          m_prevPose;

    // class World realize synchronization mode
    static std::unique_ptr<World> m_world;

private:
    // Automatic flight
    void runAutomatic(std::list<Goal> path);

    // Controlled flight
    void runControlled();

    // Get current pose at the space
    Pose getPose() const;

    /*
     * Return true if |pose - goal| >= eps
     * It is possible if robot have landed or pushed away from it is current pose
     */
    bool isFarFromGoal(const Pose &pose, const Goal &goal, double eps = 0.4) const noexcept;


    // Create a goal on current direction and position
    Goal getGoal();

    // Subscriber callback
    void directionChanged(const std_msgs::Byte::ConstPtr &direction);

    // Go to the current goal
    void goToGoal();

    bool stopPublishing (std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    bool startPublishing(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

public:
    GoalsPublisher()                                    = delete;
    GoalsPublisher(const GoalsPublisher &)              = delete;
    GoalsPublisher(GoalsPublisher&&)                    = delete;
    GoalsPublisher & operator=(const GoalsPublisher &)  = delete;
    GoalsPublisher & operator=(GoalsPublisher &&)       = delete;

    GoalsPublisher(const std::string &worldFrame,
                   const std::string &frame,
                   size_t             publishRate,
                   std::list<Goal>    path = {});

    ~GoalsPublisher();

    static void initWorld(double worldWidth, double worldLength, double worldHeight,
                          double regWidth,   double regLength,   double regHeight,
                          double offsetOX,   double offsetOY,    double offsetOZ);
};

#endif // GOALS_PUBLISHER_H