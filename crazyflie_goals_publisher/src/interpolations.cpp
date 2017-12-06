#include <cmath>
#include <array>
#include "interpolations.h"


std::list<Goal> interpolate(const Goal &goal1, const Goal &goal2, double distance) {
    std::list<Goal> intermediateGoals;

    double sqDifX = std::pow(goal2.x() - goal1.x(), 2);
    double sqDifY = std::pow(goal2.y() - goal1.y(), 2);
    double sqDifZ = std::pow(goal2.z() - goal1.z(), 2);

    size_t intermediateGoalsAmount = std::max(1.0, std::sqrt(sqDifX + sqDifY + sqDifZ) / distance);

    intermediateGoals.push_back(goal1);
    for (size_t k = 1; k < intermediateGoalsAmount; ++k) {

        double current_x = goal1.x() + k * (goal2.x() - goal1.x()) / intermediateGoalsAmount;
        double current_y = goal1.y() + k * (goal2.y() - goal1.y()) / intermediateGoalsAmount;
        double current_z = goal1.z() + k * (goal2.z() - goal1.z()) / intermediateGoalsAmount;

        intermediateGoals.push_back(Goal(current_x, current_y, current_z, 0.0, 0.0, 0.0));
    }

    return intermediateGoals;
}


std::list<Goal> createSpline(std::list<Goal> goals, double step) {
    std::list<Goal> intermediateGoals;

    goals.push_front(goals.front());
    goals.push_front(goals.front());
    goals.push_back(goals.back());

    std::array<double, 3> Ax, Ay, Az;
    std::array<double, 2> Bx, By, Bz;
    std::array<double, 4> t = { 0.0, 1.0, 2.0, 3.0 };

    auto finish = std::prev(goals.end(), t.size());

    for (auto goal = goals.begin(); goal != finish; ++goal) {
        for (double T = 1; T < 2; T += step) {

            auto firstGoal  = goal;
            auto secondGoal = goal;

            for (size_t j = 0; j < Ax.size(); ++j) {
                ++secondGoal;

                Ax[j] = (t[j+1] - T) / (t[j+1] - t[j]) * firstGoal->x() +
                        (T - t[j]) / (t[j+1] - t[j]) * secondGoal->x();

                Ay[j] = (t[j+1] - T) / (t[j+1] - t[j]) * firstGoal->y() +
                        (T - t[j]) / (t[j+1] - t[j]) * secondGoal->y();

                Az[j] = (t[j+1] - T) / (t[j+1] - t[j]) * firstGoal->z() +
                        (T - t[j]) / (t[j+1] - t[j]) * secondGoal->z();

                ++firstGoal;
            }

            for (size_t j = 0; j < Bx.size(); ++j) {
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

