#include "ros/ros.h"
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
    ros::NodeHandle n;
    ros::ServiceClient takeoff_client           = n.serviceClient<std_srvs::Empty>("takeoff");
    ros::ServiceClient landing_client           = n.serviceClient<std_srvs::Empty>("land");
    ros::ServiceClient emergency_client         = n.serviceClient<std_srvs::Empty>("emergency");
    ros::ServiceClient update_params_client     = n.serviceClient<crazyflie_driver::UpdateParams>("update_params");

    std_srvs::Empty takeoff_srv;
    std_srvs::Empty landing_srv;
    std_srvs::Empty emergency_srv;
    crazyflie_driver::UpdateParams update_params_srv;

    enum {
        KEYCODE_UP      = 0x41,
        KEYCODE_DOWN    = 0x42,
        KEYCODE_RIGHT   = 0x43,
        KEYCODE_LEFT    = 0x44,
        KEYCODE_QUIT    = 0x71
    };

    while (ros::ok()) {
        int key = getKey();

        switch (key) {
            case KEYCODE_UP:
                if (!takeoff_client.call(takeoff_srv))
                    std::cerr << "Could not call takeoff service" << std::endl;
                break;
            case KEYCODE_DOWN:
                if (!landing_client.call(landing_srv))
                    std::cerr << "Could not call landing service" << std::endl;
                break;
            case KEYCODE_LEFT:
                if (!emergency_client.call(emergency_srv))
                    std::cerr << "Could not call emergency service" << std::endl;
                break;
            case KEYCODE_RIGHT:
                int value;
                n.getParam("ring/headlightEnable", value);

                if (value)
                    n.setParam("ring/headlightEnable", 1);
                else
                    n.setParam("ring/headlightEnable", 0);

                if (!update_params_client.call(update_params_srv))
                    std::cerr << "Could not call update_params service" << std::endl;
                break;
            case KEYCODE_QUIT:
                if (!landing_client.call(landing_srv))
                    std::cerr << "Could not call landing service" << std::endl;
                ros::shutdown();
                break;
        } // switch (key)

        ros::spinOnce();
    }

    return 0;
}
