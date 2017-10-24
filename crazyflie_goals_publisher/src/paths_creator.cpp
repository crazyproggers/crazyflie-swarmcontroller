#include <tf/transform_listener.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include "paths_creator.h"
#include "interpolations.h"


PathsCreator::PathsCreator(
        const  std::string &worldFrame,
        const  std::vector<std::string> &frames,
        const  std::string &mapPath,
        bool   splinesMode)
        : m_worldFrame      (worldFrame)
        , m_frames          (frames)
{
    if (readTable(mapPath))
        createPaths(splinesMode);
}


bool PathsCreator::readTable(const std::string &mapPath) {
    // #########################################################
    // ####### FINDING STARTING POINTS FOR ALL CRAZYFLIES ######
    // #########################################################
    const size_t TOTAL_CRAZYFLIES = m_frames.size();

    tf::TransformListener transformListeners[TOTAL_CRAZYFLIES];
    ros::Time             common_times[TOTAL_CRAZYFLIES];
    tf::StampedTransform  startingPoints[TOTAL_CRAZYFLIES];

    for (size_t i = 0; i < TOTAL_CRAZYFLIES; ++i) {
        transformListeners[i].waitForTransform(m_worldFrame, m_frames[i], ros::Time(0), ros::Duration(5.0));
        transformListeners[i].getLatestCommonTime(m_worldFrame, m_frames[i], common_times[i], NULL);

        if (transformListeners[i].canTransform(m_worldFrame, m_frames[i], common_times[i]))
            transformListeners[i].lookupTransform(m_worldFrame, m_frames[i], common_times[i], startingPoints[i]);
    }

    // #########################################################
    // ################# SOME LAMBDA FUNCTIONS #################
    // #########################################################
    size_t repeat_number        = 0;
    size_t repeated_goals_count = 0;

    auto repeat = [&](std::list<Goal> &entry) mutable {
        std::list<Goal> tmp(std::prev(entry.end(), repeated_goals_count), entry.end());

        for (; repeat_number > 0; --repeat_number)
            entry.insert(entry.end(), tmp.begin(), tmp.end());
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
    std::ifstream map;
    //map.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    std::list<Goal> entry;
    enum AMOUNT {
        COMMAND_COMPONENTS  = 2,
        PARAMETERS          = 5
    };

    //try {
        map.open(mapPath.c_str(), std::ios_base::in);

        while (!map.eof()) {
            std::string line;
            std::getline(map, line);

            if ((line == "") && (entry.size())) {
                if (repeat_number > 0)
                    repeat(entry);

                // Add the finishing goal in the table
                Goal last = entry.back();
                double last_z = 0.1;

                entry.push_back(Goal(last.x(), last.y(), last_z, 0.0, 0.0, 0.0));
                m_goalsTable.push_back(entry);
                entry.clear();
            }
            else {
                std::istringstream iss(line);
                std::vector<std::string> words {std::istream_iterator<std::string>{iss},
                                                std::istream_iterator<std::string>{}};

                if (words.size() == AMOUNT::COMMAND_COMPONENTS) {
                    /*
                     * COMMANDS:
                     * repeat N  -- to repeat next N goals
                     */
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

                    if (!entry.size()) {
                        size_t num = m_goalsTable.size();

                        double x = startingPoints[num].getOrigin().x();
                        double y = startingPoints[num].getOrigin().y();
                        double z = startingPoints[num].getOrigin().z() + 0.2;
                        double yaw = 0.0;

                        entry.push_back(Goal(x, y, z, roll, pitch, yaw));
                    }

                    double x     = std::stod(words[0]);
                    double y     = std::stod(words[1]);
                    double z     = std::stod(words[2]);
                    double yaw   = std::stod(words[3]);
                    double delay = std::stod(words[4]);

                    yaw = degToRad(fixAngle(yaw));

                    entry.push_back(Goal(x, y, z, roll, pitch, yaw, delay));
                    if (repeat_number) repeated_goals_count++;
                }
                else {
                    ROS_ERROR("It's wrong map-file!");
                    return false;
                }
            } // else
        } // while (!map.eof())

        if (entry.size()) {
            if (repeat_number > 0)
                repeat(entry);
            
            // Add the finishing goal in the table
            Goal last = entry.back();
            double last_z = 0.1;

            entry.push_back(Goal(last.x(), last.y(), last_z, 0.0, 0.0, 0.0));
            m_goalsTable.push_back(entry);
        }

        map.close();
    //} // try
    /*
    catch (std::ifstream::failure exp) {
        ROS_ERROR("Could not read/close map-file!");
        return false;
    }
    */

    return true;
}


void PathsCreator::createPaths(bool splinesMode) {
    if (splinesMode) {
        for (auto entry: m_goalsTable)
            entry = createSpline(std::move(entry));
    }
    else {
        for (size_t i = 0; i < m_goalsTable.size(); ++i) {
            std::list<Goal> tmp;
            std::list<Goal> entry;

            auto finish = std::prev(m_goalsTable[i].end());

            for (auto it = m_goalsTable[i].begin(); it != finish; ++it) {
                Goal curr = *it;
                Goal next = *(std::next(it));

                tmp = interpolate(curr, next);
                entry.insert(entry.end(), tmp.begin(), tmp.end());
            }

            entry.push_back(m_goalsTable[i].back());
            m_goalsTable[i] = std::move(entry);
        }
    }
}


std::list<Goal> PathsCreator::getPath(const std::string &frame) const {
    for (size_t i = 0; i < m_frames.size(); ++i)
        if (m_frames[i] == frame) 
            return m_goalsTable[i];
        else
            ROS_ERROR("Unknown frame: %s", frame.c_str());
 
    return std::list<Goal>();
}


PathsCreator::~PathsCreator() {}
