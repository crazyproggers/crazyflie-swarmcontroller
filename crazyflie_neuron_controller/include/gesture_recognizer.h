#ifndef GESTURE_RECOGNIZER_H
#define GESTURE_RECOGNIZER_H

#include <ros/ros.h>
#include "observer.h"

class Gesture;
class Hand;


class GestureRecognizer: public Observer, public std::enable_shared_from_this<GestureRecognizer> {
public:
    GestureRecognizer(const GestureRecognizer&)            = delete;
    GestureRecognizer& operator=(const GestureRecognizer&) = delete;

    GestureRecognizer(ros::NodeHandle &node);
   ~GestureRecognizer();

private:
    std::array<std::unique_ptr<Gesture>, 11> m_gestures;
    std::unique_ptr<Hand>                    m_hand;
    ros::Publisher                           m_commandsPublisher;
    size_t                                   m_sameGesturesCounter;
    size_t                                   m_prevGestureIdx;
    size_t                                   m_emptyGestureIdx;

    void update();
};

#endif // GESTURE_RECOGNIZER_H