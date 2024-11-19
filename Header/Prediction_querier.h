#pragma once

#include <string>
#include <vector>
#include "../sldd/Obstacle.h"
#include "../Header/path_matcher.h"

struct PredictionQuerier
{
    /* data */
    std::string id;
    const Obstacle* obstacle;
};

class Prediction_querier
{
private:
    /* data */
    std::vector<PredictionQuerier> id_obstacle_map_;
    std::vector<const Obstacle*> obstacles_;
    
    std::vector<PathPoint> ptr_reference_line_;

    std::vector<PredictionQuerier>::const_iterator find(const std::string& obstacle_id) const;
    
public:
    Prediction_querier(const std::vector<const Obstacle*>& obstacles, const std::vector<PathPoint>& ptr_reference_line);
    
    std::vector<const Obstacle*> GetObstacles() const;

    double ProjectVelocityAlongReferenceLine(const std::string& obstacle_id, const double s, const double t) const;

    ~Prediction_querier() = default;
};



