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

    std::string mapPath;
    nh.getParam("map", mapPath);

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
    GoalsPublisher::world = new World(worldWidth, worldLength, worldHeight, regWidth, regLength, regHeight);

    GoalsPublisher *goalsPublisher[frames.size()];
    std::thread    *thr[frames.size()];
    PathsCreator pathsCreator(worldFrame, frames, mapPath, splinesMode);
    
    for (size_t i = 0; i < frames.size(); ++i) {
        goalsPublisher[i] = new GoalsPublisher(worldFrame, frames[i], rate);
        
        // Automatic flight
        std::list<Goal> path = std::move(pathsCreator.getPath(frames[i])); 
        thr[i] = new std::thread([&] (GoalsPublisher *gp) { gp->run(std::move(path)); }, goalsPublisher[i]);
    }

    for (size_t i = 0; i < frames.size(); ++i) {
        thr[i]->join();
        delete thr[i];
        delete goalsPublisher[i];
    }
    delete GoalsPublisher::world;

    return 0;
}
