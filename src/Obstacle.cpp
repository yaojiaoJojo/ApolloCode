
#include "../sldd/Obstacle.h"

Obstacle::Obstacle(PerceptionObstacles& obstacles)
{
    id_ = obstacles.id;
    vertices_ = obstacles.corner_points;
    isVirtual_ = obstacles.isVirtual;
    perception_obstacles_= obstacles;
    setTrajectory();
}

void Obstacle::setTrajectory()
{
    double confidence_max = std::numeric_limits<double>::max();
    std::vector<TrajectoryPoint> traj_point;
    for(auto& it:perception_obstacles_.msg)
    {
        if(it.probablity > confidence_max)
        {
            confidence_max = it.probablity;
            traj_point = it.trajectory_points;
        }
    }
    trajectory_.probablity = confidence_max;
    trajectory_.trajectory_points = traj_point;
}

TrajectoryPoint Obstacle::GetPointAtTime(const double relative_time) const
{
    const std::vector<TrajectoryPoint>& points = trajectory_.trajectory_points;
    if(points.size() < 2)
    {
        TrajectoryPoint pt;
        pt.path_point.x = perception_obstacles_.position.first;
        pt.path_point.y = perception_obstacles_.position.second;
        pt.path_point.s = 0.0;
        pt.path_point.theta = perception_obstacles_.theta;
        pt.path_point.kappa = 0.0;
        pt.path_point.dkappa = 0.0;
        pt.path_point.ddkappa = 0.0;
        pt.v = 0.0;
        pt.a = 0.0;
        pt.relative_time = 0.0;
        return pt;
    }
    else
    {
        auto comp = [](const TrajectoryPoint p, const double time) {return p.relative_time < time;};
        std::vector<TrajectoryPoint>::const_iterator it_lower = std::lower_bound(points.begin(), points.end(), relative_time, comp);
        if(it_lower == points.end())
        {
            return *points.rbegin();
        }
        else if(it_lower == points.begin())
        {
            return *points.begin();
        }
        else
        {
            return InterpolateUsingLinearApproximation(*(it_lower -1), *it_lower, relative_time);
        }
    }
}

BBox2D Obstacle::GetBoundingBox(const TrajectoryPoint& point) const
{
    Vec2d center;
    center.x = point.path_point.x;
    center.y = point.path_point.y;

    Box2d box(center, point.path_point.theta, Getobstacles().length, Getobstacles().width);
    return box.GetBox2d();
}


