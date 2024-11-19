#pragma once

#include <iostream>
#include "../sldd/Points.h"
#include <vector>
#include <cmath>

class PathMatcher{
public:
    static PathPoint MatchToPath(const std::vector<PathPoint>& reference_line, const double x, const double y);

    static std::pair<double, double> GetPathFrenetCoordinate(const std::vector<PathPoint>&reference_line, const double x, const double y);

    static PathPoint MatchToPath(const std::vector<PathPoint>& reference_line, const double s);

    static PathPoint FindProjectionPoint(const PathPoint& p0, const PathPoint& p1, const double x, const double y);
};

double func_distance_square(const PathPoint& point, const double x, const double y);

PathPoint InterpolateUsingLinearApproximation(const PathPoint& p0, const PathPoint& p1, const double s);
TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint& p0, const TrajectoryPoint&p1, const double time);

