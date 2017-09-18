#include <fstream>
#include <iostream>
#include <thread>
#include "goals_publisher.h"


constexpr double degToRad(double deg) {
    return deg / 180.0 * M_PI;
}

inline double fixCoordinate(double value, size_t num) {
    bool isAngle = ((num >= 3) && (num <= 5))? true : false;
    bool isPosition = (num < 3)? true : false;

    if (isAngle) {
        if (value > 180)
            value -= 360;
        else if (value < -180)
            value += 360;
    }
    else if (isPosition) {
        if (value > 2)
            value = 2;
        if (value < 0)
            value = 0;
    }

    return value;
}


std::vector<std::vector<Goal>> getGoals(
    const std::string &map_path,
    double distanceBetweenDots = 0.01) {

    std::ifstream map;
    map.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    std::vector<std::vector<Goal>> goals_table;

    try {
        map.open(map_path.c_str(), std::ios_base::in);
        const uint PARAMETERS_AMOUNT = 7;

        while (!map.eof()) {
            std::vector<std::vector<double>> anchor;
            std::vector<double> intermediateDots;

            uint anchorPointsAmount;
            map >> anchorPointsAmount;

            // Get anchor points from map-file
            for (size_t i = 0; i < anchorPointsAmount; ++i) {
                std::vector<double> tmp;
                double anchorPointsValue;
                
                for (size_t j = 0; j < PARAMETERS_AMOUNT; ++j) {
                    map >> anchorPointsValue;
                    tmp.push_back(anchorPointsValue);
                }
                anchor.push_back(tmp);
            }

            for (size_t i = 0; i < anchor.size() - 1; ++i) {
                double delta_x = anchor[i][0] - anchor[i+1][0];
                double delta_y = anchor[i][1] - anchor[i+1][1];
                double delta_z = anchor[i][2] - anchor[i+1][2];
                double delta_yaw = abs(anchor[i][5] - anchor[i+1][5]);

                double intermediateDotsAmount = std::sqrt(std::pow(delta_x, 2) 
                                                          + std::pow(delta_y, 2) 
                                                          + std::pow(delta_z, 2))
                                                          / distanceBetweenDots;
                
                double distanceBetweenCordinates[PARAMETERS_AMOUNT];
                double qurentPosition[PARAMETERS_AMOUNT];

                for (size_t j = 0; j < anchor[0].size(); ++j) {
                    distanceBetweenCordinates[j] = anchor[i+1][j] - anchor[i][j];
                    if (intermediateDotsAmount)
                        qurentPosition[j] = anchor[i][j] + distanceBetweenCordinates[j] / intermediateDotsAmount;
                    anchor[i][j] = fixCoordinate(anchor[i][j], j);
                    intermediateDots.push_back(anchor[i][j]);
                }

                for (size_t k = 0; k < intermediateDotsAmount; ++k) {
                    for (size_t j = 0; j < PARAMETERS_AMOUNT - 1; ++j) {
                        qurentPosition[j] = fixCoordinate(qurentPosition[j], j);
                        intermediateDots.push_back(qurentPosition[j]);
                        qurentPosition[j] += distanceBetweenCordinates[j] / intermediateDotsAmount;

                    }
                    intermediateDots.push_back(0.0); //delay between intermediate dots
                }
            } // for (size_t i = 0; i < anchor.size() - 1; ++i)

            for (size_t i = 0; i < anchor[0].size(); ++i) {
               anchor[anchor.size()-1][i] = fixCoordinate(anchor[anchor.size()-1][i], i);
               intermediateDots.push_back(anchor[anchor.size()-1][i]);
            }
            
            std::vector<Goal> entry;
            for (size_t i = 0; i < intermediateDots.size(); i += PARAMETERS_AMOUNT) {
                double &x     = intermediateDots[i];
                double &y     = intermediateDots[i+1];
                double &z     = intermediateDots[i+2];
                double &roll  = intermediateDots[i+3];
                double &pitch = intermediateDots[i+4];
                double &yaw   = intermediateDots[i+5];
                double &delay = intermediateDots[i+6];
                entry.push_back(Goal(x, y, z, degToRad(roll), degToRad(pitch), degToRad(yaw), delay));
            }
            goals_table.push_back(entry);
       } // while (!map.eof()) 
       map.close();
   } // try
   catch (std::ifstream::failure exp) {
        std::cerr << "Could not read/close map-file!" << std::endl;
   }

   return goals_table;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "goals_publisher");

    // Read parameters
    ros::NodeHandle n("~");
    std::string worldFrame;
    n.param<std::string>("worldFrame", worldFrame, "/world");

    std::string frames_str;
    n.getParam("frames", frames_str);

    // Split frames_str by whitespace
    std::istringstream iss(frames_str);
    std::vector<std::string> frame {std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};

    std::string map_path;
    n.getParam("map", map_path);

    // Read goals from map-file and to interpolate they
    double distanceBetweenDots;
    n.getParam("distanceBetweenDots", distanceBetweenDots);

    std::vector<std::vector<Goal>> goals = std::move(getGoals(map_path, distanceBetweenDots));
    if (!goals.size()) return -1;

    int rate;
    n.getParam("rate", rate);

    bool waitForAllAtAnchor;
    n.getParam("waitForAllAtAnchor", waitForAllAtAnchor);

    GoalsPublisher *goalsPublisher[frame.size()];
    std::thread   *thr[frame.size()];
    for (size_t i = 0; i < frame.size(); ++i) {
        goalsPublisher[i] = new GoalsPublisher(worldFrame, frame[i], goals[i], rate, waitForAllAtAnchor, frame.size());
        thr[i] = new std::thread(&GoalsPublisher::run, goalsPublisher[i]);
    }

    for (size_t i = 0; i < frame.size(); ++i) {
        thr[i]->join();
        delete thr[i];
        delete goalsPublisher[i];
    }

    return 0;
}
