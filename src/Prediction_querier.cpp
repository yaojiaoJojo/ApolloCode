#include "../Header/Prediction_querier.h"



Prediction_querier::Prediction_querier(const std::vector<const Obstacle*>& obstacles, 
                                        const std::vector<PathPoint>& ptr_reference_line)
:ptr_reference_line_(ptr_reference_line)
{
    id_obstacle_map_.clear();
    for(const auto& obs:obstacles)
    {
        PredictionQuerier id_obs_map;
        id_obs_map.id = obs->getId();
        id_obs_map.obstacle = obs;
        
        id_obstacle_map_.emplace_back(id_obs_map);
        obstacles_.push_back(obs);
    }

}
std::vector<PredictionQuerier>::const_iterator Prediction_querier::find(const std::string& obstacle_id) const
{
    for(auto it = id_obstacle_map_.begin(); it < id_obstacle_map_.end(); it++)
    {
        if(obstacle_id == it->id)
        {
            return it;
        }
        
    }
    return id_obstacle_map_.end();
}

std::vector<const Obstacle*> Prediction_querier::GetObstacles() const
{
    return obstacles_;
}

double Prediction_querier::ProjectVelocityAlongReferenceLine(const std::string& obstacle_id, const double s, const double t) const
{
    auto obstacle_pos = find(obstacle_id);
    if(obstacle_pos == id_obstacle_map_.end())
    {
        std::cout << "obstacle is not in list" << std::endl;
        return 0.0;
    }
    const auto trajectroy = obstacle_pos->obstacle->getTrajectory();
    uint64_t num_trajectory_point = trajectroy.trajectory_points.size();
    if(num_trajectory_point < 2)
    {
        return 0.0;
    }
    if(t < trajectroy.trajectory_points.at(0).relative_time || t > trajectroy.trajectory_points.at(num_trajectory_point-1).relative_time)
    {
        return 0.0;
    }
    auto comp = [](const TrajectoryPoint& p, double t) {return p.relative_time < t;};
    auto matched_it = std::lower_bound(trajectroy.trajectory_points.begin(), trajectroy.trajectory_points.end(), t, comp);

    double v = matched_it->v;
    double theta = matched_it->path_point.theta;
    double v_x = v*std::cos(theta);
    double v_y = v*std::sin(theta);

    PathPoint obstacle_point_on_ref_line = PathMatcher::MatchToPath(ptr_reference_line_, s);

    double ref_theta = obstacle_point_on_ref_line.theta;
    return std::cos(ref_theta)*v_x + std::sin(ref_theta)*v_y;
}