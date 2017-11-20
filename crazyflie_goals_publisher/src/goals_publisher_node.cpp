#include "goals_publisher.h"
#include "paths_creator.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "goals_publisher");

    // Read parameters
    ros::NodeHandle node("~");
    std::string worldFrame;
    node.param<std::string>("worldFrame", worldFrame, "/world");

    std::string frames_str;
    node.getParam("/swarm/frames", frames_str);

    // Split frames_str by whitespace
    std::istringstream iss(frames_str);
    std::vector<std::string> frames {std::istream_iterator<std::string>{iss},
                                     std::istream_iterator<std::string>{}};

    std::string pathToMap;
    node.getParam("map", pathToMap);

    int rate;
    node.getParam("rate", rate);

    bool splinesMode = false;
    node.getParam("splinesMode", splinesMode);

    double worldWidth, worldLength, worldHeight;
    double regWidth,   regLength,   regHeight;
    node.getParam("worldWidth",  worldWidth);
    node.getParam("worldLength", worldLength);
    node.getParam("worldHeight", worldHeight);
    node.getParam("regWidth",    regWidth);
    node.getParam("regLength",   regLength);
    node.getParam("regHeight",   regHeight);

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
