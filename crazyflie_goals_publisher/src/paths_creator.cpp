#include <tf/transform_listener.h>
#include <fstream>

#include "paths_creator.h"
#include "interpolations.h"


PathsCreator::PathsCreator(
        const  std::string &pathToMap,
        const  std::string &worldFrame,
        bool   splinesMode)
        : m_worldFrame      (worldFrame)
        , m_splinesMode     (splinesMode)
        , m_canGenPaths     (false)
{
    if (readTable(pathToMap))
        m_canGenPaths = true;
}


PathsCreator::~PathsCreator() {
    m_paths.clear();
}


bool PathsCreator::readTable(const std::string &pathToMap) {
    // #########################################################
    // ################# SOME LAMBDA FUNCTIONS #################
    // #########################################################
    size_t repeat_number         = 0;
    size_t repeated_goals_amount = 0;

    auto repeat = [&](std::list<Goal> &path) mutable {
        std::list<Goal> tmp(std::prev(path.end(), repeated_goals_amount), path.end());

        for (; repeat_number > 0; --repeat_number)
            path.insert(path.end(), tmp.begin(), tmp.end());
        repeated_goals_amount = 0;
    };

    auto fixAngle = [](double degree) -> double {
        if (degree > 180.0)
            degree -= 360.0;
        else if (degree < -180.0)
            degree += 360.0;

        return degree;
    };

    auto degToRad = [](double degree) -> double {
        return degree / 180.0 * M_PI;
    };

    // #########################################################
    // ############## READ AND PARSE THE MAP-FILE ##############
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
            if (repeat_number)
                repeat(path);

            m_paths.push_back(path);
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

                     if (repeat_number)
                        repeat(path);
                     repeat_number = arg;
                 }
                 else {
                    ROS_ERROR("Wrong command: %s", command.c_str());
                    return false;
                 }
            }
            else if (words.size() == AMOUNT::PARAMETERS) {
                double x        = std::stod(words[0]);
                double y        = std::stod(words[1]);
                double z        = std::stod(words[2]);
                double yaw      = std::stod(words[3]);
                double delay    = std::stod(words[4]);
                double roll     = 0.0;
                double pitch    = 0.0;
                bool   isAnchor = true;

                yaw = degToRad(fixAngle(yaw));

                path.emplace_back(x, y, z, roll, pitch, yaw, delay, isAnchor);
                if (repeat_number) ++repeated_goals_amount;
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
        m_paths.push_back(path);
    }

    return true;
}


bool PathsCreator::canGenPaths() const noexcept {
    return m_canGenPaths;
}


std::list<Goal> PathsCreator::genPath(const std::string &frame) {
    if (!m_canGenPaths) return std::list<Goal>();

    // #########################################################
    // ########## FIND THE STARTING POINT OF ROBOT #############
    // #########################################################
    tf::TransformListener listener;
    tf::StampedTransform  startPoint;

    listener.waitForTransform(m_worldFrame, frame, ros::Time(0), ros::Duration(5.0));
    try {
        listener.lookupTransform(m_worldFrame, frame, ros::Time(0), startPoint);
    }
    catch (tf::TransformException &exc) {
        ROS_ERROR("%s%s", frame.c_str(), ": could not get current position!");
        ROS_ERROR("An exception was caught: %s", exc.what());
        return std::list<Goal>();
    }

    double x0 = startPoint.getOrigin().x();
    double y0 = startPoint.getOrigin().y();
    double z0 = startPoint.getOrigin().z() + 0.5;

    // #########################################################
    // ######## FIND THE NEAREST PATH TO STARTING POINT ########
    // #########################################################
    auto dist = [](double x0, double y0, double z0, double x, double y, double z) -> double {
        return std::sqrt(std::pow(x - x0, 2) + std::pow(y - y0, 2) + std::pow(z - z0, 2));
    };

    double minDist = std::numeric_limits<double>::max();
    size_t pathNum = 0;

    for (size_t i = 0; i < m_paths.size(); ++i) {
        Goal firstGoal = m_paths[i].front();
        double d = dist(x0, y0, z0, firstGoal.x(), firstGoal.y(), firstGoal.z());

        if (d < minDist) {
            minDist = d;
            pathNum = i;
        }
    }

    std::list<Goal> selectedPath = std::move(m_paths[pathNum]);
    m_paths.erase(std::next(m_paths.begin(), pathNum));

    // #########################################################
    // ############## INTREPOLATE SELECTED PATH ################
    // #########################################################
    if (m_splinesMode)
        selectedPath = createSpline(std::move(selectedPath));
    else {
        std::list<Goal> tmp;
        std::list<Goal> path;

        auto finish = std::prev(selectedPath.end());

        for (auto it = selectedPath.begin(); it != finish; ++it) {
            Goal curr = *it;
            Goal next = *(std::next(it));

            tmp = interpolate(curr, next);
            path.splice(path.end(), tmp);
        }

        path.push_back(selectedPath.back());
        selectedPath = std::move(path);
    }

    // #########################################################
    // ################ ADD FIRST AND LAST GOAL ################
    // #########################################################
    selectedPath.emplace_front(x0, y0, z0, 0.0, 0.0, 0.0);

    Goal   last   = selectedPath.back();
    double last_z = startPoint.getOrigin().z() + 0.2;
    selectedPath.emplace_back(last.x(), last.y(), last_z, 0.0, 0.0, 0.0, 0.5);

    return selectedPath;
}
