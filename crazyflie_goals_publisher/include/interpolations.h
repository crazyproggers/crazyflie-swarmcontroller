#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <vector>
#include "goal.h"

// Linear interpolation between two goals
std::vector<Goal> interpolate(const Goal &goal1, const Goal &goal2, double distance = 0.01);

/*
 *  Catmull-Rom interpolation
 *  https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline
 */
std::vector<Goal> createSpline(std::vector<Goal> goals, double step = 0.01);

#endif // INTERPOLATION_H
