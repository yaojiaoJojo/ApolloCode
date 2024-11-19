#include "../sldd/Points.h"
#include <cmath>
#include <vector>

std::vector<PathPoint> ToDiscretizedReferenceLine(const std::vector<ReferencePoint> &ref_points)
{
    std::vector<PathPoint> path_points;
    for(const auto& ref_point:ref_points)
    {
        double s = 0;
        PathPoint path_point;
        path_point.x = ref_point.rx;
        path_point.y = ref_point.ry;
        path_point.theta = ref_point.rtheta;

        path_point.kappa = ref_point.rkappa;
        path_point.dkappa = ref_point.rdkappa;
        if(!path_points.empty())
        {
            double dx = path_point.x - path_points.back().x;
            double dy = path_point.y = path_points.back().y;

            s += sqrt(dx*dx + dy*dy);
        }
        path_point.s = s;
        path_points.push_back(path_point);
    }
    return path_points;
}