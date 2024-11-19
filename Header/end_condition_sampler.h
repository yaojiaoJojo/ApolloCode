#pragma once
#include "../sldd/Points.h"
#include "../Header/feasible_region.h"
#include "../Header/path_time_graph.h"
#include <vector>
#include <string>
#include <array>
#include "../Header/Prediction_querier.h"




class end_condition_sampler
{
private:
    /* data */
    std::array<double, 3> init_s_;
    std::array<double, 3> init_d_;
    FeasibleRegion feasible_region_;
    PathTimeGraph ptr_path_time_graph_;
    Prediction_querier ptr_prediction_querier_;


    std::vector<SamplePoint> QueryPathTimeObstacleSamplePoints() const;

    void QueryFollowPathTimePoints(const VehicleConfig& vehicle_config, const std::string& obstacle_id,
                                    std::vector<SamplePoint>* sample_points) const;
    
    void QueryOvertakePathTimePoints(const VehicleConfig& vehicle_config, const std::string& obstacle_id,
                                        std::vector<SamplePoint>* sample_points) const;

    

public:
    end_condition_sampler(const std::array<double, 3>& init_s, const std::array<double, 3>& init_d,
                        PathTimeGraph ptr_path_time_graph,
                        Prediction_querier ptr_prediction_querier);

    std::vector<std::pair<std::array<double, 3>, double>> SampleLatEndConditions() const;

    std::vector<std::pair<std::array<double, 3>, double>> SampleLonEndConditionsForCruising(const double ref_cruise_speed) const;

    std::vector<std::pair<std::array<double, 3>, double>> SampleLonEndConditionForStopping(const double ref_stop_point) const;

    std::vector<std::pair<std::array<double, 3>, double>> SampleLonEndConditionForPathTimePoints() const;
   
};

