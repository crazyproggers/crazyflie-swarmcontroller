#include <tf/transform_listener.h>
#include <cmath>
#include <vector>
#include "interpolations.h"


std::vector<Goal> interpolate(const Goal &goal1, const Goal &goal2) {
    std::vector<Goal> intermediateGoals;

    double distanceBetweenDots = 0.01;
    double sqDifX = std::pow(goal2.x() - goal1.x(), 2);
    double sqDifY = std::pow(goal2.y() - goal1.y(), 2);
    double sqDifZ = std::pow(goal2.z() - goal1.z(), 2);

    uint intermediateDotsAmount = std::max(1.0, std::sqrt(sqDifX + sqDifY + sqDifZ) / distanceBetweenDots);

    intermediateGoals.push_back(goal1);
    for (size_t k = 1; k <= intermediateDotsAmount; ++k) {

        double current_x = goal1.x() + k * (goal2.x() - goal1.x()) / intermediateDotsAmount;
        double current_y = goal1.y() + k * (goal2.y() - goal1.y()) / intermediateDotsAmount;
        double current_z = goal1.z() + k * (goal2.z() - goal1.z()) / intermediateDotsAmount;

        intermediateGoals.push_back(Goal(current_x, current_y, current_z, 0.0, 0.0, 0.0, 0.0));
    }

    return intermediateGoals;
}


std::vector<Goal> createSpline(std::vector<Goal> goals, double step) {

    std::vector<Goal> intermediateGoals;
    double roll = goals[0].roll();
    double pitch = goals[0].roll();
    double yaw = goals[0].roll();
    double delay = 0.0;
    goals.insert(goals.begin(), goals[0]);
    goals.insert(goals.begin(), goals[0]);
    goals.push_back(goals[goals.size()-1]);

    double Ax[5], Ay[5], Az[5], Bx[5], By[5], Bz[5];
    double t[4] = { 0.0, 1.0, 2.0, 3.0 };

    for (size_t i = 1; i < goals.size() - 2; ++i) {

        for (double T = 1; T < 2; T += step) {

            for (size_t j = 1; j < 4; ++j) {
                Ax[j-1] = (t[j] - T) / (t[j] - t[j-1]) * goals[j+i-1].x() +
                        (T - t[j-1]) / (t[j] - t[j-1]) * goals[j+i].x();

                Ay[j-1] = (t[j] - T) / (t[j] - t[j-1]) * goals[j+i-1].y() +
                        (T - t[j-1]) / (t[j] - t[j-1]) * goals[j+i].y();

                Az[j-1] = (t[j] - T) / (t[j] - t[j-1]) * goals[j+i-1].z() +
                        (T - t[j-1]) / (t[j] - t[j-1]) * goals[j+i].z();
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

            intermediateGoals.push_back(Goal(Cx, Cy, Cz, roll, pitch, yaw, delay));
        }
    }

    return intermediateGoals;
}

