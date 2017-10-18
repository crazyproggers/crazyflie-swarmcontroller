#include "ros/ros.h"
#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/LogBlock.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "crazyflie_add_group");

    ros::NodeHandle n("~");

    // read paramaters
    std::string uri_str;
    std::string frames_str;

    double roll_trim;
    double pitch_trim;

    bool use_ros_time;
    bool enable_logging;
    bool enable_parameters;
    bool enable_logging_imu;
    bool enable_logging_temperature;
    bool enable_logging_magnetic_field;
    bool enable_logging_pressure;
    bool enable_logging_battery;

    n.getParam("uri",              uri_str);
    n.getParam("/swarm/frames",    frames_str);

    n.param("roll_trim",                     roll_trim,                      0.0);
    n.param("pitch_trim",                    pitch_trim,                     0.0);
    n.param("use_ros_time",                  use_ros_time,                  true);
    n.param("enable_logging",                enable_logging,                true);
    n.param("enable_parameters",             enable_parameters,             true);
    n.param("enable_logging_imu",            enable_logging_imu,            true);
    n.param("enable_logging_temperature",    enable_logging_temperature,    true);
    n.param("enable_logging_magnetic_field", enable_logging_magnetic_field, true);
    n.param("enable_logging_pressure",       enable_logging_pressure,       true);
    n.param("enable_logging_battery",        enable_logging_battery,        true);

    ROS_INFO("wait_for_service /add_crazyflie");
    ros::ServiceClient addCrazyflieService = n.serviceClient<crazyflie_driver::AddCrazyflie>("/add_crazyflie");
    addCrazyflieService.waitForExistence();
    ROS_INFO("found /add_crazyflie");

    // Split uri_str by whitespace
    std::istringstream iss_uri(uri_str);
    std::vector<std::string> uri {std::istream_iterator<std::string>{iss_uri},
                                  std::istream_iterator<std::string>{}};
    // Split frames_str by whitespace
    std::istringstream iss_frames(frames_str);
    std::vector<std::string> frames {std::istream_iterator<std::string>{iss_frames},
                                     std::istream_iterator<std::string>{}};

    // uri.size() must equal to frames.size()
    for (size_t i = 0; i < uri.size(); ++i) {
        crazyflie_driver::AddCrazyflie addCrazyflie;

        addCrazyflie.request.uri                            = uri[i];
        addCrazyflie.request.frame                          = frames[i];
        addCrazyflie.request.roll_trim                      = roll_trim;
        addCrazyflie.request.pitch_trim                     = pitch_trim;
        addCrazyflie.request.use_ros_time                   = use_ros_time;
        addCrazyflie.request.enable_logging                 = enable_logging;
        addCrazyflie.request.enable_parameters              = enable_parameters;
        addCrazyflie.request.enable_logging_imu             = enable_logging_imu;
        addCrazyflie.request.enable_logging_temperature     = enable_logging_temperature;
        addCrazyflie.request.enable_logging_magnetic_field  = enable_logging_magnetic_field;
        addCrazyflie.request.enable_logging_pressure        = enable_logging_pressure;
        addCrazyflie.request.enable_logging_battery         = enable_logging_battery;

        std::vector<std::string> genericLogTopics;
        n.param("genericLogTopics", genericLogTopics, std::vector<std::string>());
        std::vector<int> genericLogTopicFrequencies;
        n.param("genericLogTopicFrequencies", genericLogTopicFrequencies, std::vector<int>());

        if (genericLogTopics.size() == genericLogTopicFrequencies.size()) {
            size_t k = 0;

            for (auto &topic: genericLogTopics) {
                crazyflie_driver::LogBlock logBlock;
                logBlock.topic_name = topic;
                logBlock.frequency  = genericLogTopicFrequencies[k];
                n.getParam("genericLogTopic_" + topic + "_Variables", logBlock.variables);
                addCrazyflie.request.log_blocks.push_back(logBlock);
                ++k;
            }
        }
        else ROS_ERROR("Cardinality of genericLogTopics and"
                       "genericLogTopicFrequencies does not match!");

        addCrazyflieService.call(addCrazyflie);
    } // for (size_t i = 0; i < uri.size(); ++i)

    return 0;
}
