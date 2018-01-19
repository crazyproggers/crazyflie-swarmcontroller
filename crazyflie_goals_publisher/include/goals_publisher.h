#ifndef GOALS_PUBLISHER_H
#define GOALS_PUBLISHER_H

#include <tf/transform_listener.h>
#include <std_msgs/Byte.h>
#include <thread>
#include "goal.h"
#include "world.h"


class GoalsPublisher {
    ros::NodeHandle               m_node;
    std::string                   m_worldFrame;
    std::string                   m_frame;
    Pose                          m_pose;
    std::unique_ptr<Occupator>    m_occupator;
    ros::Publisher                m_publisher;
    tf::TransformListener         m_listener;
    ros::Rate                     m_publishRate;
    int8_t                        m_direction;
    std::thread                   m_updatePoseThread;
    std::thread                   m_runThread;

    // class World realize synchronization mode
    static std::unique_ptr<World> m_world;

private:
    // Automatic flight
    void runAutomatic(std::list<Goal> path);

    // Controlled flight
    void runControlled();

    // Update pose of occupator at the space
    void updatePose();

    // Create a goal on current direction and position
    Goal getGoal();

    // Subscriber callback
    void directionChanged(const std_msgs::Byte::ConstPtr &direction);

    // Go to the current goal
    void goToGoal();

public:
    GoalsPublisher() = delete;
    GoalsPublisher(const GoalsPublisher &) = delete;
    GoalsPublisher & operator=(const GoalsPublisher &) = delete;

    GoalsPublisher(const std::string &worldFrame,
                   const std::string &frame,
                   size_t publishRate,
                   std::list<Goal> path = {});
    ~GoalsPublisher();

    static void initWorld(double worldWidth, double worldLength, double worldHeight,
                          double regWidth,   double regLength,   double regHeight,
                          double offsetOX,   double offsetOY,    double offsetOZ);
};

#endif // GOALS_PUBLISHER_H