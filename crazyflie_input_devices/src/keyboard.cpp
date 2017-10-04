#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "crazyflie_driver/UpdateParams.h"
#include <termios.h>

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

int main (int argc, char **argv) {
    ros::init(argc, argv, "keyboard");
    ros::NodeHandle n("~");

    std::string frames_str;
    n.getParam("frames", frames_str);

    // Split frames_str by whitespace
    std::istringstream iss(frames_str);
    std::vector<std::string> frame {std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};
    
    std_srvs::Empty empty_srv;
    crazyflie_driver::UpdateParams update_srv;

    enum KEYBOARD {
        ARROW_UP      = 0x41, // take off
        ARROW_DOWN    = 0x42, // landing
        ARROW_RIGHT   = 0x43, // update params
        ARROW_LEFT    = 0x44, // emergency
        KEY_Q         = 0x71, // quit 
        KEY_8         = 0x38, // move forward
        KEY_2         = 0x32, // move backward
        KEY_4         = 0x34, // move to right
        KEY_6         = 0x36, // move to left
        KEY_5         = 0x35, // move up
        KEY_0         = 0x30  // move down    
    };

    for (size_t i = 0; i < frame.size(); ++i) {
        ROS_INFO("Waiting for takeoff services");
        ros::service::waitForService(frame[i] + "/takeoff");
        ROS_INFO("Found takeoff sevices");

        ROS_INFO("Waiting for land services");
        ros::service::waitForService(frame[i] + "/land");
        ROS_INFO("Found land services");

        ROS_INFO("Waiting for emergency services");
        ros::service::waitForService(frame[i] + "/emergency");
        ROS_INFO("Found emergency services");

        ROS_INFO("Waiting for update_params services");
        ros::service::waitForService(frame[i] + "/update_params");
        ROS_INFO("Found update_params services");
    }

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        int key = getKey();

        switch (key) {
            case ARROW_UP:
                for (size_t i = 0; i < frame.size(); ++i)
                    ros::service::call(frame[i] + "/takeoff", empty_srv);
                break;
            case ARROW_DOWN:
                for (size_t i = 0; i < frame.size(); ++i)
                    ros::service::call(frame[i] + "/land", empty_srv);
                break;
            case ARROW_LEFT:
                for (size_t i = 0; i < frame.size(); ++i)
                    ros::service::call(frame[i] + "/emergency", empty_srv);
                break;
            case ARROW_RIGHT:
                for (size_t i = 0; i < frame.size(); ++i) {
                    int value;
                    n.getParam(frame[i] + "/ring/headlightEnable", value);

                    if (value)
                        n.setParam(frame[i] + "/ring/headlightEnable", 1);
                    else
                        n.setParam(frame[i] + "/ring/headlightEnable", 0);

                    ros::service::call(frame[i] + "/update_params", update_srv);
                }
                break;
            case KEY_Q:
                for (size_t i = 0; i < frame.size(); ++i)
                    ros::service::call(frame[i] + "/land", empty_srv);
                ros::shutdown();
                break;
        } // switch (key)

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
