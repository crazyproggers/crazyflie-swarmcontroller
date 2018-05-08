#include "gesture_recognizer.h"
#include "gesture.h"
#include "hand.h"
#include "commands.h"


template<typename T, typename ...Args>
std::unique_ptr<T> make_unique( Args&& ...args ) {
    return std::unique_ptr<T>(new T( std::forward<Args>(args)... ));
}


GestureRecognizer::~GestureRecognizer() {}


GestureRecognizer::GestureRecognizer(ros::NodeHandle &node)
    : m_sameGesturesCounter(0)
    , m_prevGestureIdx     (0)  // index of Takeoff gesture
    , m_emptyGestureIdx    (10) // index of Empty gesture
{
    m_commandsPublisher = node.advertise<std_msgs::Byte>("/swarm/commands", 1);

    std::vector<BoneInfo> bonesInfo = {{"RightHand"        , "RightHandThumb1" }, {"RightInHandIndex", "RightHandIndex1"},
                                       {"RightInHandMiddle", "RightHandMiddle1"}, {"RightInHandRing" , "RightHandRing1" },
                                       {"RightInHandPinky" , "RightHandPinky1" }};

    m_hand = make_unique<Hand>(bonesInfo);

    auto wptr = std::shared_ptr<GestureRecognizer>(this, [](GestureRecognizer*) {});
    m_hand->attach(shared_from_this());

    m_gestures[0]  = make_unique<Takeoff>  ();
    m_gestures[1]  = make_unique<Landing>  ();
    m_gestures[2]  = make_unique<Upward>   ();
    m_gestures[3]  = make_unique<Downward> ();
    m_gestures[4]  = make_unique<Leftward> ();
    m_gestures[5]  = make_unique<Rightward>();
    m_gestures[6]  = make_unique<Forward>  ();
    m_gestures[7]  = make_unique<Backward> ();
    m_gestures[8]  = make_unique<Yawleft>  ();
    m_gestures[9]  = make_unique<Yawright> ();
    m_gestures[10] = make_unique<Empty>    ();
}


void GestureRecognizer::update() {
    m_hand->resetMovedBonesAmount();

    // Find the max coincidence among all gestures
    double maxCoincidence = 0.0;
    size_t gestureIdx = 0;

    for (size_t i = 0; i < m_gestures.size(); ++i) {
        double res = m_gestures[i]->recognize(m_hand->bones());

        if (res > maxCoincidence) {
            maxCoincidence = res;
            gestureIdx = i;
        }
    }

    /*
     * If max coincidence > E
     * and found gesture is not Empty
     * and we have 10 same gestures in sequence
     * then we can publish command into topic
     */
    constexpr double eps = 0.6;
    constexpr size_t minSameGesturesForCommand = 5;

    if (maxCoincidence > eps && gestureIdx != m_emptyGestureIdx) {
        if (gestureIdx == m_prevGestureIdx)
            ++m_sameGesturesCounter;
        else
            m_prevGestureIdx = gestureIdx;

        if (m_sameGesturesCounter < minSameGesturesForCommand)
            return;

        //std::cout << typeid(*m_gestures[gestureIdx]).name() << "   " << maxCoincidence << std::endl;
        m_sameGesturesCounter = 0;
        m_commandsPublisher.publish(m_gestures[gestureIdx]->getCommand().msg());
    }
}
