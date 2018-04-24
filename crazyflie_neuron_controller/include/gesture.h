#ifndef GESTURE_H
#define GESTURE_H

#include "hand.h"

class Bone;
class Command;


class Gesture {
public:
    Gesture() = default;
   ~Gesture()                          = default;
    Gesture(const Gesture&)            = delete;
    Gesture& operator=(const Gesture&) = delete;

    // Returns percentage of similarity of inputed bones to gesture
    double recognize(const bones_t &bones) const noexcept;

    virtual Command getCommand() const noexcept = 0;

protected:
    struct StandardBone {
        struct  { double x, y, w; } mean, std;
    };

    std::vector<StandardBone> m_standardBones;
};


class Empty: public Gesture {
public:
    Empty();
    Command getCommand() const noexcept;
};


class Takeoff: public Gesture {
public:
    Takeoff();
    Command getCommand() const noexcept;
};


class Landing: public Gesture {
public:
    Landing();
    Command getCommand() const noexcept;
};


class Upward: public Gesture {
public:
    Upward();
    Command getCommand() const noexcept;
};


class Downward: public Gesture {
public:
    Downward();
    Command getCommand() const noexcept;
};


class Leftward: public Gesture {
public:
    Leftward();
    Command getCommand() const noexcept;
};


class Rightward: public Gesture {
public:
    Rightward();
    Command getCommand() const noexcept;
};


class Forward: public Gesture {
public:
    Forward();
    Command getCommand() const noexcept;
};


class Backward: public Gesture {
public:
    Backward();
    Command getCommand() const noexcept;
};


class Yawright: public Gesture {
public:
    Yawright();
    Command getCommand() const noexcept;
};


class Yawleft: public Gesture {
public:
    Yawleft();
    Command getCommand() const noexcept;
};

#endif // GESTURE_H