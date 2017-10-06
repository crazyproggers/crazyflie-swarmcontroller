#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <tf/transform_listener.h>
#include <vector>
#include "goal.h"

std::vector<Goal> interpolate(const Goal &goal1, const Goal &goal2, double distanceBetweenDots = 0.01);

std::vector<Goal> createSpline(std::vector<Goal> goals, double step = 0.01);

#endif // INTERPOLATION_H
