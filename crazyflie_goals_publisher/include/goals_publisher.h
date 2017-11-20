#ifndef GOALS_PUBLISHER_H
#define GOALS_PUBLISHER_H

#include <tf/transform_listener.h>
#include <std_msgs/Byte.h>
#include <thread>
#include <mutex>
#include "goal.h"
#include "world.h"


class GoalsPublisher {
    ros::NodeHandle               m_node;
    std::string                   m_worldFrame;
    std::string                   m_frame;
    size_t                        m_robot_id;
    ros::Publisher                m_publisher;
    ros::Subscriber               m_subscriber;
    tf::TransformListener         m_listener;
    ros::Rate                     m_publishRate;
    mutable int8_t                m_direction;

    std::thread                   m_runThread;
    static std::mutex             m_errMutex;

    // class World realize synchronization mode
    static std::unique_ptr<World> m_world;

private:
    enum DIRECTION {
        forward     = 1,
        backward    = 2,
        rightward   = 3,
        leftward    = 4,
        upward      = 5,
        downward    = 6,
    };

    // Automatic flight
    void runAutomatic(std::list<Goal> path);

    // Controlled flight
    void runControlled(double frequency);

    // Get current position at the space
    inline Goal getPosition() const;

    // Checking that |position - goal| < E
    inline bool goalIsReached(const Goal &position, const Goal &goal) const;

    // Create a goal on current direction and position
    inline Goal getGoal() const;

    // Subscriber callback
    void directionChanged(const std_msgs::Byte::ConstPtr &direction);

    // Timer callback
    void goToGoal(const ros::TimerEvent &event);

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
                          double regWidth,   double regLength,   double regHeight);
};

#endif // GOALS_PUBLISHER_H
