#include <thread>
#include "goals_publisher.h"
#include "paths_creator.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "goals_publisher");

    // Read parameters
    ros::NodeHandle n("~");
    std::string worldFrame;
    n.param<std::string>("worldFrame", worldFrame, "/world");

    std::string frames_str;
    n.getParam("/swarm/frames", frames_str);

    // Split frames_str by whitespace
    std::istringstream iss(frames_str);
    std::vector<std::string> frames {std::istream_iterator<std::string>{iss},
                                     std::istream_iterator<std::string>{}};

    std::string mapPath;
    n.getParam("map", mapPath);

    int rate;
    n.getParam("rate", rate);

    bool splinesMode = false;
    n.getParam("splinesMode", splinesMode);

    double worldWidth, worldLength, worldHeight;
    double regWidth,   regLength,   regHeight;
    n.getParam("worldWidth",  worldWidth);
    n.getParam("worldLength", worldLength);
    n.getParam("worldHeight", worldHeight);
    n.getParam("regWidth",    regWidth);
    n.getParam("regLength",   regLength);
    n.getParam("regHeight",   regHeight);

    // Initialize the synchronization mode
    GoalsPublisher::world = new World(worldWidth, worldLength, worldHeight, regWidth, regLength, regHeight);


    GoalsPublisher *goalsPublisher[frames.size()];
    std::thread    *thr[frames.size()];

    PathsCreator pathsCreator(worldFrame, frames, mapPath, splinesMode);
    
    for (size_t i = 0; i < frames.size(); ++i) {
        goalsPublisher[i] = new GoalsPublisher(worldFrame, frames[i], rate);
        
        // Automatic flight
        std::vector<Goal> path = std::move(pathsCreator.getPath(frames[i])); 
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
