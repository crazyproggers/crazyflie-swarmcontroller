#include <ros/ros.h>
#include <std_srvs/Empty.h>
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
                for (size_t i = 0; i < frame.size(); ++i) {
                    if (!ros::service::call(frame[i] + "/takeoff", empty_srv))
                        std::cerr << "Could not call takeoff service" << std::endl;
                }
                break;
            case KEYCODE_DOWN:
                for (size_t i = 0; i < frame.size(); ++i) {
                    if (!ros::service::call(frame[i] + "/land", empty_srv))
                        std::cerr << "Could not call landing service" << std::endl;
                }
                break;
            case KEYCODE_LEFT:
                for (size_t i = 0; i < frame.size(); ++i) {
                    if (!ros::service::call(frame[i] + "/emergency", empty_srv))
                        std::cerr << "Could not call emergency service" << std::endl;
                }
                break;
            case KEYCODE_RIGHT:
                for (size_t i = 0; i < frame.size(); ++i) {
                    int value;
                    n.getParam(frame[i] + "/ring/headlightEnable", value);

                    if (value)
                        n.setParam(frame[i] + "/ring/headlightEnable", 1);
                    else
                        n.setParam(frame[i] + "ring/headlightEnable", 0);

                    if (!ros::service::call(frame[i] + "/update_params", empty_srv))
                        std::cerr << "Could not call update_params service" << std::endl;
                }
                break;
            case KEYCODE_QUIT:
                for (size_t i = 0; i < frame.size(); ++i) {
                    if (!ros::service::call(frame[i] + "/land", empty_srv))
                        std::cerr << "Could not call landing service" << std::endl;
                }
                ros::shutdown();
                break;
        } // switch (key)

        ros::spinOnce();
    }

    return 0;
}
