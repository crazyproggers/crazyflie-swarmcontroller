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
    if (readGoals(mapPath))
        createPaths(splinesMode);
}


bool PathsCreator::readGoals(const std::string &mapPath) {
    // #########################################################
    // ####### TO FIND STARTING POINTS FOR ALL CRAZYFLIES ######
    // #########################################################
    const uint TOTAL_CRAZYFLIES = m_frames.size();

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
    uint repeat_number        = 0;
    uint repeated_goals_count = 0;

    auto repeat = [&](std::vector<Goal> &entry) mutable {
        size_t begin = entry.size() - repeated_goals_count;
        size_t end   = entry.size();
        std::vector<Goal> tmp(&entry[begin], &entry[end]);

        for (; repeat_number > 0; --repeat_number)
            entry.insert(std::end(entry), std::begin(tmp), std::end(tmp));
        repeated_goals_count = 0;
    };

    auto fixAngle = [](double degree) {
        if (degree > 180)
            degree -= 360;
        else if (degree < -180)
            degree += 360;

        return degree;
    };

    auto degToRad = [](double degree) {
        return degree / 180.0 * M_PI;
    };


    // #########################################################
    // ########### TO READ AND TO PARSE THE MAP-FILE ###########
    // #########################################################
    std::ifstream map;
    map.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    std::vector<Goal> entry;
    enum AMOUNT {
        COMMAND_COMPONENTS  = 2,
        PARAMETERS          = 5
    };

    try {
        map.open(mapPath.c_str(), std::ios_base::in);

        while (!map.eof()) {
            std::string line;
            std::getline(map, line);

            if ((line == "") && (entry.size())) {
                if (repeat_number > 0)
                    repeat(entry);

                // Add the finishing goal in the table
                Goal last = entry[entry.size()-1];
                entry.push_back(Goal(last.x(), last.y(), 0.1, 0.0, 0.0, 0.0));

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
                             std::cerr << "Wrong arg for command \"repeat\": " << arg << std::endl;
                             return false;
                         }
                         repeat_number = arg;
                     }
                     else {
                        std::cerr << "Wrong command: " << command << std::endl;
                        return false;
                     }
                }
                else if (words.size() == AMOUNT::PARAMETERS) {
                    double x = 0.0, y = 0.0, z = 0.0;
                    double roll = 0.0, pitch = 0.0, yaw = 0.0;
                    double delay = 0.0;

                    // Add the starting goal in the table
                    if (!entry.size()) {
                        size_t num = m_goalsTable.size();

                        x = startingPoints[num].getOrigin().x();
                        y = startingPoints[num].getOrigin().y();
                        z = startingPoints[num].getOrigin().z() + 0.2;
                    }
                    else {
                        x     = std::stod(words[0]);
                        y     = std::stod(words[1]);
                        z     = std::stod(words[2]);
                        yaw   = std::stod(words[3]);
                        delay = std::stod(words[4]);

                        yaw   = degToRad(fixAngle(yaw));
                    }

                    entry.push_back(Goal(x, y, z, roll, pitch, yaw, delay));
                    if (repeat_number) repeated_goals_count++;
                }
                else {
                    std::cerr << "It's wrong map-file!" << std::endl;
                    return false;
                }
            } // else
        } // while (!map.eof())

        if (entry.size()) {
            if (repeat_number > 0)
                repeat(entry);
            
            // Add the finishing goal in the table
            Goal last = entry[entry.size()-1];
            entry.push_back(Goal(last.x(), last.y(), 0.1, 0.0, 0.0, 0.0));

            m_goalsTable.push_back(entry);
        }
        map.close();
    } // try
    catch (std::ifstream::failure exp) {
        std::cerr << "Could not read/close map-file!" << std::endl;
        return false;
    }
    return true;
} // readGoals


void PathsCreator::createPaths(bool splinesMode) {
    if (splinesMode) {
        for (auto entry: m_goalsTable)
            entry = createSpline(std::move(entry));
    }
    else {
        for (size_t i = 0; i < m_goalsTable.size(); ++i) {
            std::vector<Goal> tmp;
            std::vector<Goal> entry;

            for (size_t j = 0; j < m_goalsTable[i].size() - 1; ++j) {
                tmp = interpolate(m_goalsTable[i][j], m_goalsTable[i][j+1]);
                entry.insert(entry.end(), tmp.begin(), tmp.end());
            }

            entry.push_back(m_goalsTable[i][m_goalsTable[i].size()-1]);
            m_goalsTable[i] = std::move(entry);
        }
    }
}


std::vector<Goal> PathsCreator::getPath(const std::string &frame) const {
    for (size_t i = 0; i < m_frames.size(); ++i)
        if (m_frames[i] == frame) 
            return m_goalsTable[i];
        else 
            std::cerr << "Unknown frame: " << frame << std::endl;
 
    return std::vector<Goal>();
}


PathsCreator::~PathsCreator() {}
