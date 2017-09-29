#include <tf/transform_listener.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include "paths_creator.h"


PathsCreator::PathsCreator(
        const  std::string &worldFrame,
        const  std::vector<std::string> &frames,
        const  std::string &mapPath,
        double distanceBetweenDots,
        bool   splinesMode)
        : m_worldFrame      (worldFrame)
        , m_frames          (frames)
{
    if (readGoals(mapPath))
        interpolate(distanceBetweenDots, splinesMode);
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
    enum {
        COMMAND_COMPONENTS_AMOUNT  = 2,
        PARAMETERS_AMOUNT          = 5
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
                entry.push_back(Goal(last.x(), last.y(), 0.1, 0.0, 0.0, 0.0, 0.0));

                m_goalsTable.push_back(entry);
                entry.clear();
            }
            else {
                std::istringstream iss(line);
                std::vector<std::string> words {std::istream_iterator<std::string>{iss},
                                                std::istream_iterator<std::string>{}};

                if (words.size() == COMMAND_COMPONENTS_AMOUNT) {
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
                else if (words.size() == PARAMETERS_AMOUNT) {
                    double x = 0.0, y = 0.0, z = 0.0;
                    double roll = 0.0, pitch = 0.0, yaw = 0.0;
                    double delay = 0.0;
                    bool   isAnchor = true;

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

                    entry.push_back(Goal(x, y, z, roll, pitch, yaw, delay, isAnchor));
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
            entry.push_back(Goal(last.x(), last.y(), 0.1, 0.0, 0.0, 0.0, 0.0));

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


void PathsCreator::interpolate(double distanceBetweenDots, bool splinesMode) {
    if (!splinesMode) {
        for (size_t i = 0; i < m_goalsTable.size(); ++i) {
            std::vector<Goal> entry;

            for (size_t j = 0; j < m_goalsTable.size() - 1; ++j) {
                double x        = m_goalsTable[i][j].x();
                double y        = m_goalsTable[i][j].y();
                double z        = m_goalsTable[i][j].z();
                double roll     = m_goalsTable[i][j].roll();
                double pitch    = m_goalsTable[i][j].pitch();
                double yaw      = m_goalsTable[i][j].yaw();
                double delay    = m_goalsTable[i][j].delay();
                bool   isAnchor = m_goalsTable[i][j].isAnchor();

                double delta_x = m_goalsTable[i][j+1].x() - x;
                double delta_y = m_goalsTable[i][j+1].y() - y;
                double delta_z = m_goalsTable[i][j+1].z() - z;

                uint intermediateDotsAmount = std::max(1.0, std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2) + std::pow(delta_z, 2)) / distanceBetweenDots);

                double current_x = x + delta_x / intermediateDotsAmount;
                double current_y = y + delta_y / intermediateDotsAmount;
                double current_z = z + delta_z / intermediateDotsAmount;

                entry.push_back(Goal(x, y, z, roll, pitch, yaw, delay, isAnchor));
                delay = 0.0;

                for (size_t k = 0; k < intermediateDotsAmount; ++k) {
                    entry.push_back(Goal(current_x, current_y, current_z, roll, pitch, yaw, delay));
                    current_x += delta_x / intermediateDotsAmount;
                    current_y += delta_y / intermediateDotsAmount;
                    current_z += delta_z / intermediateDotsAmount;
                }
            } // for (size_t j = 0; j < m_goalsTable.size() - 1; ++j)

            m_goalsTable[i] = entry;
        } // for (size_t i = 0; i < m_goalsTable.size(); ++i)
    } // if (!splinesMode)
    else {
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
