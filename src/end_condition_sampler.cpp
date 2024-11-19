#include "../Header/end_condition_sampler.h"
// #include "feasible_region.cpp"

using State = std::array<double, 3>;
using Condition = std::pair<State, double>;


end_condition_sampler::end_condition_sampler(const std::array<double, 3>& init_s, const std::array<double, 3>& init_d,
                        PathTimeGraph ptr_path_time_graph,
                        Prediction_querier ptr_prediction_querier)
    : init_s_(init_s),
      init_d_(init_d),
      feasible_region_(init_s),
      ptr_path_time_graph_(ptr_path_time_graph),
      ptr_prediction_querier_(ptr_prediction_querier){}



std::vector<Condition> end_condition_sampler::SampleLatEndConditions() const
{
    std::vector<Condition> end_d_conditions;
    std::array<double, 3> end_d_candidates = {0.0, -0.5, 0.5};
    std::array<double, 4> end_s_candidates = {10.0, 20.0, 40.0, 80.0};
    for(auto& s:end_s_candidates)
    {
        for(auto& d:end_d_candidates)
        {
            State end_d_state = {d, 0.0, 0.0};
            Condition end_d_condition;
            end_d_condition.first = end_d_state;
            end_d_condition.second = s;
            end_d_conditions.emplace_back(end_d_condition);
        }
    }
    return end_d_conditions;
}


std::vector<Condition> end_condition_sampler::SampleLonEndConditionsForCruising(const double ref_cruise_speed) const
{
    std::vector<Condition> end_s_conditions;
    if(ref_cruise_speed <= 1)
    {
        std::cout << "reference speed is too low!" << std::endl;
        return end_s_conditions;
    }
    static const uint8_t num_of_time_samples = 9;
    std::array<double,num_of_time_samples> time_samples;
    time_samples[0] = FLAGS_polyomial_minimal_param;
    for(int i = 1; i < num_of_time_samples; i++)
    {
        time_samples[i] = static_cast<double>(i)*FLAGS_trajectory_time_length/(num_of_time_samples - 1);
    }

    for(auto& time:time_samples)
    {
        double v_upper = std::min(feasible_region_.VUpper(time), ref_cruise_speed);
        double v_lower = feasible_region_.VLower(time);

        State lower_end_s = {0.0, v_lower, 0};
        State upper_end_s = {0.0, v_upper, 0};
        end_s_conditions.emplace_back(lower_end_s, time);
        end_s_conditions.emplace_back(upper_end_s, time);


        double v_range = v_upper - v_lower;
        uint64_t num_of_mid_points = std::min(static_cast<uint64_t>(FLAGS_num_velocity_sample-2) ,static_cast<uint64_t>(v_range/FLAGS_min_velocity_sample_gap));
        if(num_of_mid_points > 0)
        {
            double velocity_seg = v_range/static_cast<double>(num_of_mid_points + 1);
            for(int j = 0; j < num_of_mid_points; j++)
            {
                State end_s = {0.0, (v_lower + velocity_seg*static_cast<double>(j)), 0.0};
                end_s_conditions.emplace_back(end_s, time);
            }
        }
    }

    return end_s_conditions;
}

std::vector<Condition> end_condition_sampler::SampleLonEndConditionForStopping(const double ref_stop_point) const
{
    std::vector<Condition> end_s_conditions;
    uint8_t const size_t_num_of_time_samples = 9;
    std::array<double, size_t_num_of_time_samples> time_samples;
    time_samples[0] = FLAGS_polyomial_minimal_param;
    for(int i = 0; i < size_t_num_of_time_samples; i++)
    {
        double ratio = static_cast<double>(i)/(size_t_num_of_time_samples - 1);
        time_samples[i] = ratio*FLAGS_trajectory_time_length;
    }
    for(auto& time:time_samples)
    {
        State end_s = {std::max(init_s_[0], ref_stop_point),0.0,0.0};
        end_s_conditions.emplace_back(end_s, time);
    }
    return end_s_conditions;
    
}

std::vector<Condition> end_condition_sampler::SampleLonEndConditionForPathTimePoints() const
{
    std::vector<Condition> end_s_conditions;
    std::vector<SamplePoint> sample_points = QueryPathTimeObstacleSamplePoints();
    for(auto& sample_point:sample_points)
    {
        if(sample_point.path_time_point.t < FLAGS_polynomial_minimal_param)
        {
            continue;
        }
        double s = sample_point.path_time_point.s;
        double v = sample_point.ref_v;
        double t = sample_point.path_time_point.t;
        if(s < feasible_region_.SLower(t) || s > feasible_region_.SUpper(t))
        {
            continue;
        }
        
        State end_state = {s, v, 0};
        Condition end_condition;
        end_condition.first = end_state;
        end_condition.second = t;
        end_s_conditions.emplace_back(end_condition);

    }
    return end_s_conditions;
}

std::vector<SamplePoint> end_condition_sampler::QueryPathTimeObstacleSamplePoints() const
{
    VehicleConfig vehicle_config;
    vehicle_config.x = 0;
    vehicle_config.y = 0;
    vehicle_config.theta = 0;
    vehicle_config.is_valid = 1;
    vehicle_config.Back_edge_to_center = 2;
    vehicle_config.Front_edge_to_center = 2;
    std::vector<SamplePoint> sample_points;
    for(auto& path_time_obstacles:ptr_prediction_querier_.GetObstacles())
    {
        std::string obstacle_id = path_time_obstacles->getId();
        QueryFollowPathTimePoints(vehicle_config, obstacle_id, &sample_points);
        QueryOvertakePathTimePoints(vehicle_config, obstacle_id, &sample_points);
    }
    return sample_points;
}


void end_condition_sampler::QueryFollowPathTimePoints(const VehicleConfig& vehicle_config, const std::string& obstacle_id,
                                    std::vector<SamplePoint>* sample_points) const
{
    std::vector<STPoint> follow_path_time_points = ptr_path_time_graph_.GetObstacleSurroundingPoints(obstacle_id, -FLAGS_numerical_epsilon, FLAGS_time_min_density);
    for(const auto& path_time_point:follow_path_time_points)
    {
        double v = ptr_prediction_querier_.ProjectVelocityAlongReferenceLine(obstacle_id, path_time_point.s, path_time_point.t);
        double s_upper = path_time_point.s - vehicle_config.Front_edge_to_center;
        double s_lower = s_upper - FLAGS_default_lon_buffer;
        double s_gap = FLAGS_default_lon_buffer/(FLAGS_num_sample_follow_per_timestamp - 1);
        for(int i = 0; i < FLAGS_num_sample_follow_per_timestamp; i++)
        {
            double s = s_lower + static_cast<double>(i)*s_gap;
            SamplePoint sample_point;
            sample_point.path_time_point.s = s;
            sample_point.path_time_point.t = path_time_point.t;
            sample_point.ref_v = v;
            sample_points->emplace_back(sample_point);
        }
    }

}
    
void end_condition_sampler::QueryOvertakePathTimePoints(const VehicleConfig& vehicle_config, const std::string& obstacle_id,
                                        std::vector<SamplePoint>* sample_points) const
{
    std::vector<STPoint> overtake_path_time_points = ptr_path_time_graph_.GetObstacleSurroundingPoints(obstacle_id, FLAGS_numerical_epsilon, FLAGS_time_min_density);
    for(auto& path_time_point:overtake_path_time_points)
    {
        double v = ptr_prediction_querier_.ProjectVelocityAlongReferenceLine(obstacle_id, path_time_point.s, path_time_point.t);
        SamplePoint sample_point;
        sample_point.path_time_point.s = path_time_point.s + FLAGS_default_lon_buffer;
        sample_point.path_time_point.t = path_time_point.t;
        sample_point.ref_v = v;
        sample_points->emplace_back(sample_point);
    }
   
}