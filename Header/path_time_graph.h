#pragma once


#include "../sldd/Points.h"
#include "../sldd/Obstacle.h"
#include "../Header/ReferenceLineInfo.h"
#include <string>
#include <array>
#include <vector>

class PathTimeGraph
{
public:
    PathTimeGraph(const std::vector<const Obstacle*>& obstacles,
                  const std::vector<PathPoint>& discretized_ref_points,
                  const ReferenceLineInfo* ptr_reference_line_info,
                  const double s_start, const double s_end, const double t_start, const double t_end,
                  const std::array<double, 3>& init_d);
    void setupObstacles(const std::vector<const Obstacle*>& obstacles, const std::vector<PathPoint>& discretized_ref_points);
    
    
    
    std::vector<STPoint> GetObstacleSurroundingPoints(const std::string& obstacle_id, const double s_dist, const double t_min_density) const;

private:
    void SetStaticObstacle(const Obstacle* obstacle, const std::vector<PathPoint>& discretized_ref_points);
    void SetDynamicObstacle(const Obstacle* obstacle, const std::vector<PathPoint>& discretized_ref_points);


    SL_boundary ComputeObstacleBoundary(const std::vector<Vec2d>& vertices, const std::vector<PathPoint>& discretized_ref_points) const;

    std::vector<STBoundary> path_time_obstacle_map_;
    std::vector<SL_boundary> path_time_obstacles_;
    std::pair<double,double> path_range_;
    std::pair<double,double> time_range_;
    const ReferenceLineInfo *ptr_reference_line_info_;
    // STPoint SetPathTimePoint(const std::string& obstacle_id, const double s, const double t) const;
    std::vector<STBoundary>::iterator find(std::vector<STBoundary>& path_time_obs_map, std::string id);
    std::vector<STBoundary>::const_iterator find(const std::string& obstacle_id) const;
};
