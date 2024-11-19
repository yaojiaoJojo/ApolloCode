#include "../Header/path_time_graph.h"
#include "limits"
#include "../Header/path_matcher.h"
#include "../sldd/Obstacle.h"

#include <iostream>
using namespace std;

#include <cmath>


PathTimeGraph::PathTimeGraph(const std::vector<const Obstacle*>& obstacles,
                  const std::vector<PathPoint>& discretized_ref_points,
                  const ReferenceLineInfo* ptr_reference_line_info,
                  const double s_start, const double s_end, const double t_start, const double t_end,
                  const std::array<double, 3>& init_d)
{
    ptr_reference_line_info_ = ptr_reference_line_info;
    path_range_.first = s_start;
    path_range_.second = s_end;
    time_range_.first = t_start;
    time_range_.second = t_end;
    PathTimeGraph::setupObstacles(obstacles, discretized_ref_points);
}


SL_boundary PathTimeGraph::ComputeObstacleBoundary(const std::vector<Vec2d>& vertices, const std::vector<PathPoint>& discretized_ref_points) const
{
    double start_s = std::numeric_limits<double>::max();
    double end_s = std::numeric_limits<double>::lowest();

    double start_l = std::numeric_limits<double>::max();
    double end_l = std::numeric_limits<double>::lowest();

    for(const Vec2d& point:vertices)
    {
        std::pair<double,double> sl_point = PathMatcher::GetPathFrenetCoordinate(discretized_ref_points, point.x, point.y);
        start_s = std::fmin(sl_point.first, start_s);
        end_s = std::fmax(sl_point.first, end_s);

        start_l = std::fmin(sl_point.second, start_l);
        end_l = std::fmax(sl_point.second, end_l);
    }

    SL_boundary sl_boundary;
    sl_boundary.start_s = start_s;
    sl_boundary.end_s = end_s;
    sl_boundary.start_l = start_l;
    sl_boundary.end_l = end_l;
    return sl_boundary;
}
/*
STPoint PathTimeGraph::SetPathTimePoint(const std::string& obstacle_id, const double s, const double t) const
{

}
*/
void PathTimeGraph::SetStaticObstacle(const Obstacle* obstacle, const std::vector<PathPoint>& discretized_ref_points)
{
    // obstacle->getAllVertices();
    const std::string obstacle_id = obstacle->getId();
    SL_boundary sl_boundary = ComputeObstacleBoundary(obstacle->getAllVertices(), discretized_ref_points);

    double left_width = FLAGS_default_reference_line_width*0.5;
    double right_width = FLAGS_default_reference_line_width*0.5;

    if(sl_boundary.start_s > path_range_.second || sl_boundary.end_s < path_range_.first ||
        sl_boundary.start_l > left_width || sl_boundary.end_l < -right_width)
        {
            std::cout << "Obstacle_id " << obstacle->getId() << " is out of range" << std::endl;
            return;
        }

    STBoundary path_time_obs;
    path_time_obs.id = obstacle->getId();
    path_time_obs.bottom_left_point.first = 0.0;
    path_time_obs.bottom_left_point.second = sl_boundary.start_s;
    path_time_obs.bottom_right_point.first = FLAGS_trajectory_time_length;
    path_time_obs.bottom_right_point.second = sl_boundary.start_s;

    path_time_obs.upper_left_point.first = 0.0;
    path_time_obs.upper_left_point.second = sl_boundary.end_s;
    path_time_obs.upper_right_point.first = FLAGS_trajectory_time_length;
    path_time_obs.upper_right_point.second = sl_boundary.end_s;
    path_time_obstacle_map_.push_back(path_time_obs);

}



void PathTimeGraph::SetDynamicObstacle(const Obstacle* obstacle, const std::vector<PathPoint>& discretized_ref_points)
{
    double relative_time = time_range_.first;
    while (relative_time < time_range_.second)
    {
        /* code */
        const TrajectoryPoint tps = obstacle->GetPointAtTime(relative_time);

        Box2d box({tps.path_point.x, tps.path_point.y}, obstacle->Getobstacles().theta, obstacle->Getobstacles().length, obstacle->Getobstacles().width);
        
        SL_boundary slboundary = PathTimeGraph::ComputeObstacleBoundary(box.GetAllCorners(), discretized_ref_points);
        double left_width = FLAGS_default_reference_line_width * 0.5;
        double right_width = FLAGS_default_reference_line_width * 0.5;
        
        
        
        std::vector<STBoundary>::iterator IsFindID = find(path_time_obstacle_map_, obstacle->getId());

        if(slboundary.start_s > path_range_.second || 
           slboundary.end_s < path_range_.first || 
           slboundary.start_l > left_width || 
           slboundary.end_l < -right_width)
           {
                
                if(IsFindID != path_time_obstacle_map_.end())
                {
                    break;
                }
                relative_time += FLAGS_trajectory_time_resolution;
                continue;
           }
           if(IsFindID == path_time_obstacle_map_.end())
           {
                STBoundary path_time_obs;
                path_time_obs.id = obstacle->getId();
                path_time_obs.bottom_left_point = {relative_time, slboundary.start_s};
                path_time_obs.upper_left_point = {relative_time, slboundary.end_s};
                path_time_obs.bottom_right_point = {0, 0};
                path_time_obs.upper_right_point = {0, 0};

                path_time_obstacle_map_.emplace_back(path_time_obs);
           }

        std::vector<STBoundary>::iterator it = find(path_time_obstacle_map_, obstacle->getId());
        it->bottom_right_point = {relative_time,slboundary.start_s};
        it->upper_right_point = {relative_time, slboundary.end_s};

        relative_time += FLAGS_trajectory_time_resolution;

    }
    
    

}

void PathTimeGraph::setupObstacles(const std::vector<const Obstacle*>& obstacles, const std::vector<PathPoint>& discretized_ref_points)
{
    // path_time_obstacle_map_.clear();
    for(const Obstacle* obs:obstacles)
    {
        if(obs->IsVirtual())
        {
            continue;
        }
        if(!obs->HasTrajectory())
        {
            SetStaticObstacle(obs, discretized_ref_points);
        }
        else
        {
            SetDynamicObstacle(obs, discretized_ref_points);
        }
    }
}

std::vector<STBoundary>::iterator PathTimeGraph::find(std::vector<STBoundary>& path_time_obs_map, std::string id)
{
    std::vector<STBoundary>::iterator it = path_time_obs_map.begin();

    for(it; it < path_time_obs_map.end(); it++)
    {
        if( (*it).id == id )
        {
            return it;
        }
    }
    it = path_time_obs_map.end();
    
    return it;
   
}
std::vector<STBoundary>::const_iterator PathTimeGraph::find(const std::string& obstacle_id) const
{
    for(auto index = path_time_obstacle_map_.begin(); index < path_time_obstacle_map_.end(); index++)
    {
        if(index->id == obstacle_id)
        {
            return index;
        }
    }

    return path_time_obstacle_map_.end();
}

std::vector<STPoint> PathTimeGraph::GetObstacleSurroundingPoints(const std::string& obstacle_id, const double s_dist, const double t_min_density) const
{
    vector<STPoint> pt_pairs;
    if(t_min_density < 0.0)
    {
        std::cout << "t_min_density is less than zero!!!" << std::endl;
        return pt_pairs;
    }

    auto obs_id_pos = find(obstacle_id);

    if(obs_id_pos == path_time_obstacle_map_.end())
    {
        return pt_pairs;
    }
    
    double s0 = 0.0;        //left_down point
    double s1 = 0.0;         //right_down point

    double t0 = 0.0;
    double t1 = 0.0;

    if(s_dist > 0)      //overtake state
    {
        s0 = obs_id_pos->upper_left_point.second;
        t0 = obs_id_pos->upper_left_point.first;

        s1 = obs_id_pos->upper_right_point.second;
        t1 = obs_id_pos->upper_right_point.first;
    }
    else
    {
        s0 = obs_id_pos->bottom_left_point.second;
        t0 = obs_id_pos->bottom_left_point.first;

        s1 = obs_id_pos->bottom_right_point.second;
        t1 = obs_id_pos->bottom_right_point.first;
    }

    double time_gap = t1 - t0;
    if(time_gap < 0.0)
    {
        std::cout << "time gap is less than zero!!!" << std::endl;
        return pt_pairs;
    }
    time_gap = std::abs(time_gap);
    uint64_t num_sections = static_cast<uint64_t>(time_gap)/(t_min_density + 1);
    double t_internal = time_gap/static_cast<double>(num_sections);
    for(int i = 0; i <= num_sections; i++)
    {
        double t = t_internal*static_cast<double>(i) + t0;
        double s = lerp(s0, t0, s1, t1, t) + s_dist;
        STPoint stt;
        stt.s = s;
        stt.t = t;
        pt_pairs.emplace_back(stt);
    }
    return pt_pairs;
}
