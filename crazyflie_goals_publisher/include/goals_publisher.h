#ifndef GOALS_PUBLISHER_H
#define GOALS_PUBLISHER_H

#include <tf/transform_listener.h>
#include <atomic>
#include "goal.h"

class GoalsPublisher {
    typedef unsigned int uint;

    std::string              m_worldFrame;
    std::string              m_frame;
    std::vector <Goal>       m_goal;
    ros::Publisher           m_publisher;
    tf::TransformListener    m_transformListener;
    uint                     m_publishRate;

    // If the mode "wait other crazyflies in anchors points" is enabled 
    bool                     m_waitForAllAtAnchor;
    uint                     m_totalCrazyflies;
    static std::atomic<uint> m_amountCrazyfliesAtAnchors;

public:
    GoalsPublisher() = delete;
    GoalsPublisher(const GoalsPublisher &) = delete;
    GoalsPublisher & operator=(const GoalsPublisher &) = delete;

    GoalsPublisher(const std::string &worldFrame, 
                   const std::string &frame, 
                   const std::vector<Goal> &goal, 
                   uint publishRate,
                   bool waitForAllInAnchors = false,
                   uint totalCrazyflies = 1);

    void run();
};

#endif // GOALS_PUBLISHER_H
