#include <tf/transform_listener.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include "paths_creator.h"
#include "interpolations.h"


PathsCreator::PathsCreator(
        const  std::string &worldFrame,
        const  std::vector<std::string> &frames,
        const  std::string &pathToMap,
        bool   splinesMode)
{
    if (readTable(pathToMap, worldFrame, frames))
        createPaths(splinesMode);
}


PathsCreator::~PathsCreator() {
    paths.clear();
}


bool PathsCreator::readTable(
        const std::string &pathToMap,
        const std::string &worldFrame,
        const std::vector<std::string> &frames)
{
    // #########################################################
    // ####### FINDING STARTING POINTS FOR ALL CRAZYFLIES ######
    // #########################################################
    const size_t TOTAL_CRAZYFLIES = frames.size();

    tf::TransformListener listeners[TOTAL_CRAZYFLIES];
    tf::StampedTransform  startPoints[TOTAL_CRAZYFLIES];

    for (size_t i = 0; i < TOTAL_CRAZYFLIES; ++i) {
        listeners[i].waitForTransform(worldFrame, frames[i], ros::Time(0), ros::Duration(5.0));

        try {
            listeners[i].lookupTransform(worldFrame, frames[i], ros::Time(0), startPoints[i]);
        }
        catch (tf::TransformException &exc) {
            ROS_ERROR("%s%s", frames[i].c_str(), ": could not get current position!");
            ROS_ERROR("An exception was caught: %s", exc.what());
            return false;
        }
    }

    // #########################################################
    // ################# SOME LAMBDA FUNCTIONS #################
    // #########################################################
    size_t repeat_number        = 0;
    size_t repeated_goals_count = 0;

    auto repeat = [&](std::list<Goal> &path) mutable {
        std::list<Goal> tmp(std::prev(path.end(), repeated_goals_count), path.end());

        for (; repeat_number > 0; --repeat_number)
            path.splice(path.end(), tmp);
        repeated_goals_count = 0;
    };

    auto fixAngle = [](double degree) {
        if (degree > 180.0)
            degree -= 360.0;
        else if (degree < -180.0)
            degree += 360.0;

        return degree;
    };

    auto degToRad = [](double degree) {
        return degree / 180.0 * M_PI;
    };

    // #########################################################
    // ########### READING AND PARSING THE MAP-FILE ############
    // #########################################################
    std::list<Goal> path;

    enum AMOUNT {
        REPEAT_COMPONENTS   = 2,
        PARAMETERS          = 5
    };

    std::ifstream map(pathToMap.c_str());
    if (!map.is_open()) {
        ROS_FATAL("Could not open map-file: %s", pathToMap.c_str());
        return false;
    }

    std::string line;
    while (std::getline(map, line)) {
        if ((line == "") && (!path.empty())) {
            if (repeat_number > 0)
                repeat(path);

            // Add the finishing goal in the table
            Goal   last       = path.back();
            double last_z     = 0.1;
            double last_roll  = 0.0;
            double last_pitch = 0.0;
            double last_yaw   = 0.0;

            path.push_back(Goal(last.x(), last.y(), last_z, last_roll, last_pitch, last_yaw));
            paths.push_back(path);
            path.clear();
        }
        else if (line != "") {
            std::istringstream iss(line);
            std::vector<std::string> words {std::istream_iterator<std::string>{iss},
                                            std::istream_iterator<std::string>{}};

            if (words.size() == AMOUNT::REPEAT_COMPONENTS) {
                 // repeat N  -- repeating next N goals
                 std::string command(words[0]);
                 int arg = std::atoi(words[1].c_str());

                 if (command == "repeat") {
                     if (arg < 0) {
                         ROS_ERROR("Wrong arg for command \"repeat\": %d", arg);
                         return false;
                     }
                     repeat_number = arg;
                 }
                 else {
                    ROS_ERROR("Wrong command: %s", command.c_str());
                    return false;
                 }
            }
            else if (words.size() == AMOUNT::PARAMETERS) {
                // Add the starting goal in the table
                double roll = 0.0, pitch = 0.0;

                if (path.empty()) {
                    size_t num = paths.size();

                    double x = startPoints[num].getOrigin().x();
                    double y = startPoints[num].getOrigin().y();
                    double z = startPoints[num].getOrigin().z() + 0.5;
                    double yaw = 0.0;

                    path.push_back(Goal(x, y, z, roll, pitch, yaw));
                }

                double x     = std::stod(words[0]);
                double y     = std::stod(words[1]);
                double z     = std::stod(words[2]);
                double yaw   = std::stod(words[3]);
                double delay = std::stod(words[4]);

                yaw = degToRad(fixAngle(yaw));

                path.push_back(Goal(x, y, z, roll, pitch, yaw, delay));
                if (repeat_number) repeated_goals_count++;
            }
            else {
                ROS_ERROR("It's wrong map-file: %s", pathToMap.c_str());
                return false;
            }
        } //  else if (line != "")
    } // while (std::getline(map, line))
    map.close();
    
    if (!path.empty()) {
        if (repeat_number > 0)
            repeat(path);
        
        // Add the finishing goal in the table
        Goal   last       = path.back();
        double last_z     = 0.1;
        double last_roll  = 0.0;
        double last_pitch = 0.0;
        double last_yaw   = 0.0;

        path.push_back(Goal(last.x(), last.y(), last_z, last_roll, last_pitch, last_yaw));
        paths.push_back(path);
    }

    return true;
}


void PathsCreator::createPaths(bool splinesMode) {
    if (splinesMode) {
        for (std::list<Goal> &path: paths)
            path = createSpline(std::move(path));
    }
    else {
        for (size_t i = 0; i < paths.size(); ++i) {
            std::list<Goal> tmp;
            std::list<Goal> path;

            auto finish = std::prev(paths[i].end());

            for (auto it = paths[i].begin(); it != finish; ++it) {
                Goal curr = *it;
                Goal next = *(std::next(it));

                tmp = interpolate(curr, next);
                path.splice(path.end(), tmp);
            }

            path.push_back(paths[i].back());
            paths[i] = std::move(path);
        }
    }
}

