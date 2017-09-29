#ifndef GOALS_PUBLISHER_H
#define GOALS_PUBLISHER_H

#include <tf/transform_listener.h>
#include <atomic>
#include <mutex>
#include "goal.h"


class GoalsPublisher {
    typedef unsigned int uint;

    std::string              m_worldFrame;
    std::string              m_frame;
    ros::Publisher           m_publisher;
    tf::TransformListener    m_transformListener;
    ros::Rate                m_loopRate;
    std::mutex               m_errMutex;

    bool                     m_synchAtAnchors;
    static uint              m_totalCrazyflies;
    static std::atomic<uint> m_amountCrazyfliesAtAnchors;

public:
    GoalsPublisher() = delete;
    GoalsPublisher(const GoalsPublisher &) = delete;
    GoalsPublisher & operator=(const GoalsPublisher &) = delete;

    GoalsPublisher(const std::string &worldFrame, 
                   const std::string &frame,
                   uint publishRate);

    /*
     * If this mode is enabled crazyflies that are located at anchor points 
     * begin to wait lagging copters. This method need to call before run(...)
     */ 
    void enableSynchAtAnchors();
    
    // Automatic flight
    void run(std::vector<Goal> path);
    
    // Controlled flight
    void run();
};

#endif // GOALS_PUBLISHER_H
