#include <memory>
#include "goals_publisher.h"
#include "paths_creator.h"

template<typename T, typename ...Args>
std::unique_ptr<T> make_unique( Args&& ...args ) {
    return std::unique_ptr<T>(new T( std::forward<Args>(args)... ));
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "goals_publisher");

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

    double worldWidth, worldLength, worldHeight;
    double regWidth,   regLength,   regHeight;
    double offsetOX,   offsetOY,    offsetOZ;

    node.getParam("worldWidth",  worldWidth);
    node.getParam("worldLength", worldLength);
    node.getParam("worldHeight", worldHeight);
    node.getParam("regWidth",    regWidth);
    node.getParam("regLength",   regLength);
    node.getParam("regHeight",   regHeight);
    node.getParam("offsetOX",    offsetOX);
    node.getParam("offsetOY",    offsetOY);
    node.getParam("offsetOZ",    offsetOZ);

    // Initialize the synchronization mode
    GoalsPublisher::initWorld(worldWidth, worldLength, worldHeight, 
                              regWidth,   regLength,   regHeight,
                              offsetOX,   offsetOY,    offsetOZ);

    std::vector<std::unique_ptr<GoalsPublisher>> publishers;

    if (!pathToMap.empty()) {
        bool splinesMode = false;
        node.getParam("splinesMode", splinesMode);

        PathsCreator creator(pathToMap, worldFrame, splinesMode);

        if (!creator.canGenPaths())
            return -1;

        for (size_t i = 0; i < frames.size(); ++i)
            publishers.push_back(make_unique<GoalsPublisher>(worldFrame, frames[i], rate, creator.genPath(frames[i])));
    }
    else {
        for (size_t i = 0; i < frames.size(); ++i)
            publishers.push_back(make_unique<GoalsPublisher>(worldFrame, frames[i], rate));
    }

    while (ros::ok())
        ros::Rate(1).sleep();

    return 0;
}