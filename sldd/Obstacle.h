#pragma once

#include <vector>
#include "Points.h"
#include <string>
#include <algorithm>
#include "../Header/path_matcher.h"
#include "../sldd/Box2d.h"
#include <iostream>

class Obstacle
{
private:
    /* data */
    std::string id_;
    std::vector<Vec2d> vertices_;

    bool isVirtual_;
    Trajectory trajectory_;
    PerceptionObstacles perception_obstacles_;

public:
    Obstacle(PerceptionObstacles& obstacles);
    void setTrajectory();
    std::string getId() const {return id_;};

    Trajectory getTrajectory() const {return trajectory_;};

    std::vector<Vec2d> getAllVertices() const {return vertices_;};
    bool IsVirtual() const {return isVirtual_;};
    bool HasTrajectory() const {return !(trajectory_.trajectory_points.empty());};
    TrajectoryPoint GetPointAtTime(const double relative_time) const;
    PerceptionObstacles Getobstacles() const {return perception_obstacles_;};
    BBox2D GetBoundingBox(const TrajectoryPoint& point) const;
};
