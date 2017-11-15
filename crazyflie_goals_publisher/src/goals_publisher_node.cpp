#include <thread>
#include "goals_publisher.h"
#include "paths_creator.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "goals_publisher");

    // Read parameters
    ros::NodeHandle nh("~");
    std::string worldFrame;
    nh.param<std::string>("worldFrame", worldFrame, "/world");

    std::string frames_str;
    nh.getParam("/swarm/frames", frames_str);

    // Split frames_str by whitespace
    std::istringstream iss(frames_str);
    std::vector<std::string> frames {std::istream_iterator<std::string>{iss},
                                     std::istream_iterator<std::string>{}};

    std::string pathToMap;
    nh.getParam("map", pathToMap);

    int rate;
    nh.getParam("rate", rate);

    bool splinesMode = false;
    nh.getParam("splinesMode", splinesMode);

    double worldWidth, worldLength, worldHeight;
    double regWidth,   regLength,   regHeight;
    nh.getParam("worldWidth",  worldWidth);
    nh.getParam("worldLength", worldLength);
    nh.getParam("worldHeight", worldHeight);
    nh.getParam("regWidth",    regWidth);
    nh.getParam("regLength",   regLength);
    nh.getParam("regHeight",   regHeight);

    // Initialize the synchronization mode
    GoalsPublisher::initWorld(worldWidth, worldLength, worldHeight, regWidth, regLength, regHeight);
    GoalsPublisher *publishers[frames.size()];

    if (!pathToMap.empty()) {
        PathsCreator creator(worldFrame, frames, pathToMap, splinesMode);

        for (size_t i = 0; i < frames.size(); ++i)
            publishers[i] = new GoalsPublisher(worldFrame, frames[i], rate, creator.paths[i]);
    }
    else {
        for (size_t i = 0; i < frames.size(); ++i)
            publishers[i] = new GoalsPublisher(worldFrame, frames[i], rate);
    }

    for (size_t i = 0; i < frames.size(); ++i)
        delete publishers[i];

    return 0;
}
