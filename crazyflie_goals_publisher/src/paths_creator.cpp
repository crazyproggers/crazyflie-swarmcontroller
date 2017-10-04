#include <tf/transform_listener.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include "paths_creator.h"

PathsCreator::PathsCreator(
        const  std::string &worldFrame,
        const  std::vector<std::string> &frames,
        const  std::string &mapPath,
        bool   splinesMode)
        : m_worldFrame      (worldFrame)
        , m_frames          (frames)
{
    if (readGoals(mapPath))
        interpolate(splinesMode);
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
                entry.push_back(Goal(last.x(), last.y(), 0.1, 0.0, 0.0, 0.0, 0.0));

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


void PathsCreator::interpolate(bool splinesMode) {
    if (!splinesMode) {
        for (size_t i = 0; i < m_goalsTable.size(); ++i) {
            std::vector<Goal> entry;
            double distanceBetweenDots = 0.01;

            for (size_t j = 0; j < m_goalsTable[i].size() - 1; ++j) {
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
        double t[5], Ax[5], Ay[5], Az[5], Bx[5], By[5], Bz[5];
        std::vector<Goal> entry;
        double step = 0.01;

        for (size_t l = 1; l < m_goalsTable.size(); ++l){
            for (size_t i = 1; i < m_goalsTable[l].size(); ++i){

                double roll     = m_goalsTable[l][i].roll();
                double pitch    = m_goalsTable[l][i].pitch();
                double yaw      = m_goalsTable[l][i].yaw();
                double delay    = 0.0;

                for (int k = 0; k < 4; ++k) t[k] = k;

                for (double T = 1; T < 2; T += step) {

                    for (size_t j = 1; j < 4; ++j) {
                        Ax[j-1] = (t[j] - T) / (t[j] - t[j-1]) * m_goalsTable[l][j+i-1].x() +
                                (T - t[j-1]) / (t[j] - t[j-1]) * m_goalsTable[l][j+i].x();

                        Ay[j-1] = (t[j] - T) / (t[j] - t[j-1]) * m_goalsTable[l][j+i-1].y() +
                                (T - t[j-1]) / (t[j] - t[j-1]) * m_goalsTable[l][j+i].y();

                        Az[j-1] = (t[j] - T) / (t[j] - t[j-1]) * m_goalsTable[l][j+i-1].z() +
                                (T - t[j-1]) / (t[j] - t[j-1]) * m_goalsTable[l][j+i].z();
                    }

                    for (size_t j = 0; j < 2; ++j) {
                        Bx[j] = (t[j+2] - T) / (t[j+2] - t[j])  * Ax[j] +
                                  (T - t[j]) / (t[j+2] - t[j])  * Ax[j+1];

                        By[j] = (t[j+2] - T) / (t[j+2] - t[j])  * Ay[j] +
                                  (T - t[j]) / (t[j+2] - t[j])  * Ay[j+1];

                        Bz[j] = (t[j+2] - T) / (t[j+2] - t[j])  * Az[j] +
                                  (T - t[j]) / (t[j+2] - t[j])  * Az[j+1];
                     }


                    double Cx = ((t[2] - T) / (t[2] - t[1]) * Bx[0] +
                             (T - t[0]) / (t[2] - t[1]) * Bx[1]) / 2;

                    double Cy = ((t[2] - T) / (t[2] - t[1]) * By[0] +
                             (T - t[0]) / (t[2] - t[1]) * By[1]) / 2;

                    double Cz = ((t[2] - T) / (t[2] - t[1]) * Bz[0] +
                             (T - t[0]) / (t[2] - t[1]) * Bz[1]) / 2;

                    entry.push_back(Goal(Cx, Cy, Cz, roll, pitch, yaw, delay));
                }
            }// for (size_t j = 1; j < m_goalsTable[l].size(); ++j)
            m_goalsTable[l] = entry;
        }// for (size_t l = 1; l < m_goalsTable.size(); ++l)
    }// else
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
