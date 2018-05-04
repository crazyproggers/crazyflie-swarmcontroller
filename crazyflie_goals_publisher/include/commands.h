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

    std_msgs::Byte msg() const noexcept {
        return *this;
    }

    int8_t value() const noexcept {
        return data;
    }
};


namespace {
    bool operator==(int8_t value, Command command) {
        return command.value() == value;
    }

    bool operator!=(int8_t value, Command command) {
        return command.value() != value;
    }
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
    const Command landing   = 10;
    const Command empty     = 11;
}

#endif // COMMANDS_H