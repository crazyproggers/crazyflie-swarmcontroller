#include <ros/ros.h>
#include <termios.h>
#include <std_srvs/Empty.h>
#include "crazyflie_driver/UpdateParams.h"
#include "commands.h"

using namespace commands;


int getKey() {
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);          // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);               // disable buffering
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

    int key = getchar();                     // read character (non-blocking)

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
    return key;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard");
    ros::NodeHandle node("~");

    std::string frames_str;
    node.getParam("/swarm/frames", frames_str);

    // Split frames_str by whitespace
    std::istringstream iss(frames_str);
    std::vector<std::string> frames {std::istream_iterator<std::string>{iss},
                                     std::istream_iterator<std::string>{}};
    
    ros::Publisher commandsPublisher = node.advertise<std_msgs::Byte>("/swarm/commands", 1);

    std_srvs::Empty empty_srv;
    crazyflie_driver::UpdateParams update_srv;

    enum KEY {
        // Commands:
        TAKEOFF       = 0x41, // arrow up
        LANDING       = 0x42, // arrow down
        UPDATE        = 0x43, // arrow left
        EMERGENCY     = 0x44, // arrow right
        QUIT          = 0x71, // key q

        // Motions (using numpad keys):
        FORWARD       = 0x38, // key 8
        BACKWARD      = 0x32, // key 2
        RIGHTWARD     = 0x34, // key 6
        LEFTWARD      = 0x36, // key 4
        UPWARD        = 0x35, // key 5
        DOWNWARD      = 0x30, // key 0
        YAWLEFT       = 0x37, // key 7
        YAWRIGHT      = 0x39  // key 9
    };

    for (auto frame: frames) {
        ROS_INFO("%s%s", frame.c_str(), ": waiting for takeoff services");
        ros::service::waitForService(frame + "/takeoff");
        ROS_INFO("%s%s", frame.c_str(), ": found takeoff sevices");

        ROS_INFO("%s%s", frame.c_str(), ": waiting for land services");
        ros::service::waitForService(frame + "/land");
        ROS_INFO("%s%s", frame.c_str(), ": found land services");

        ROS_INFO("%s%s", frame.c_str(), ": waiting for emergency services");
        ros::service::waitForService(frame + "/emergency");
        ROS_INFO("%s%s", frame.c_str(), ": found emergency services");

        ROS_INFO("%s%s", frame.c_str(), ": waiting for update_params services");
        ros::service::waitForService(frame + "/update_params");
        ROS_INFO("%s%s", frame.c_str(), ": found update_params services");
    }

    ros::Rate loop(10);
    while (ros::ok()) {
        int key = getKey();

        if (key == KEY::TAKEOFF) {
            commandsPublisher.publish(takeoff.getMsg());

            for (auto frame: frames)
                ros::service::call(frame + "/takeoff", empty_srv);
        }

        else if (key == KEY::LANDING)
            for (auto frame: frames)
                ros::service::call(frame + "/land", empty_srv);

        else if (key == KEY::EMERGENCY)
            for (auto frame: frames)
                ros::service::call(frame + "/emergency", empty_srv);

        else if (key == KEY::UPDATE)
            for (auto frame: frames) {
                int value;
                node.getParam(frame + "/ring/headlightEnable", value);

                if (value)
                    node.setParam(frame + "/ring/headlightEnable", 1);
                else
                    node.setParam(frame + "/ring/headlightEnable", 0);

                ros::service::call(frame + "/update_params", update_srv);
        }

        else if (key == KEY::QUIT) {
            for (auto frame: frames)
                ros::service::call(frame + "/land", empty_srv);
            ros::shutdown();
        }

        else if (key == KEY::FORWARD)
            commandsPublisher.publish(forward.getMsg());

        else if (key == KEY::BACKWARD) 
            commandsPublisher.publish(backward.getMsg());

        else if (key == KEY::RIGHTWARD) 
            commandsPublisher.publish(rightward.getMsg());

        else if (key == KEY::LEFTWARD)
            commandsPublisher.publish(leftward.getMsg());

        else if (key == KEY::UPWARD)
            commandsPublisher.publish(upward.getMsg());

        else if (key == KEY::DOWNWARD)
            commandsPublisher.publish(downward.getMsg());

        else if (key == KEY::YAWRIGHT)
            commandsPublisher.publish(yawright.getMsg());

        else if (key == KEY::YAWLEFT)
            commandsPublisher.publish(yawleft.getMsg());

        ros::spinOnce();
        loop.sleep();
    } // while (ros::ok())

    return 0;
}
