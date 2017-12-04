#ifndef COMMANDS_H
#define COMMANDS_H

#include <std_msgs/Byte.h>
#include <cstdint>


class Command: private std_msgs::Byte {
public:
    Command(int8_t value) {
        data = value;
    }

    Command & operator=(int8_t value) {
        data = value;
        return *this;
    }

    bool operator==(int8_t value) const {
        return value == data;
    }

    bool operator!=(int8_t value) const {
        return value != data;
    }

    friend bool operator==(int8_t value, Command command);
    friend bool operator!=(int8_t value, Command command);

    std_msgs::Byte getMsg() const {
        return static_cast<std_msgs::Byte>(*this);
    }
};

bool operator==(int8_t value, Command command) {
    return value == command.data;
}

bool operator!=(int8_t value, Command command) {
    return value != command.data;
}


// Write other commands here
namespace commands {
    const Command forward   = 1;
    const Command backward  = 2;
    const Command rightward = 3;
    const Command leftward  = 4;
    const Command upward    = 5;
    const Command downward  = 6;
    const Command yawright  = 7;
    const Command yawleft   = 8;
    const Command takeoff   = 9;
}

#endif // COMMANDS_H