#include <cmath>
#include "interpolations.h"


std::vector<Goal> interpolate(const Goal &goal1, const Goal &goal2, double distance) {
    std::vector<Goal> intermediateGoals;

    double sqDifX = std::pow(goal2.x() - goal1.x(), 2);
    double sqDifY = std::pow(goal2.y() - goal1.y(), 2);
    double sqDifZ = std::pow(goal2.z() - goal1.z(), 2);

    uint intermediateGoalsAmount = std::max(1.0, std::sqrt(sqDifX + sqDifY + sqDifZ) / distance);

    intermediateGoals.push_back(goal1);
    for (uint k = 1; k <= intermediateGoalsAmount; ++k) {

        double current_x = goal1.x() + k * (goal2.x() - goal1.x()) / intermediateGoalsAmount;
        double current_y = goal1.y() + k * (goal2.y() - goal1.y()) / intermediateGoalsAmount;
        double current_z = goal1.z() + k * (goal2.z() - goal1.z()) / intermediateGoalsAmount;

        intermediateGoals.push_back(Goal(current_x, current_y, current_z, 0.0, 0.0, 0.0));
    }

    return intermediateGoals;
}


std::vector<Goal> createSpline(std::vector<Goal> goals, double step) {
    std::vector<Goal> intermediateGoals;

    goals.insert(goals.begin(), goals[0]);
    goals.insert(goals.begin(), goals[0]);
    goals.push_back(goals[goals.size()-1]);

    double Ax[3], Ay[3], Az[3], Bx[2], By[2], Bz[2];
    double t[4] = { 0.0, 1.0, 2.0, 3.0 };

    for (size_t i = 1; i < goals.size() - 2; ++i) {

        for (double T = 1; T < 2; T += step) {

            for (size_t j = 0; j < 3; ++j) {
                Ax[j] = (t[j+1] - T) / (t[j+1] - t[j]) * goals[j+i].x() +
                        (T - t[j]) / (t[j+1] - t[j]) * goals[j+i+1].x();

                Ay[j] = (t[j+1] - T) / (t[j+1] - t[j]) * goals[j+i].y() +
                        (T - t[j]) / (t[j+1] - t[j]) * goals[j+i+1].y();

                Az[j] = (t[j+1] - T) / (t[j+1] - t[j]) * goals[j+i].z() +
                        (T - t[j]) / (t[j+1] - t[j]) * goals[j+i+1].z();
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

            intermediateGoals.push_back(Goal(Cx, Cy, Cz, 0.0, 0.0, 0.0));
        }
    }

    return intermediateGoals;
}

