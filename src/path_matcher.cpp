#include "../Header/path_matcher.h"
#include <algorithm>
using namespace std;

PathPoint PathMatcher::MatchToPath(const std::vector<PathPoint>& reference_line, const double x, const double y)
{
    PathPoint point;
    if(reference_line.empty())
    {
        point.x = 0;
        point.y = 0;
        point.z = 0;
        point.kappa = 0;
        point.dkappa = 0;
        point.ddkappa = 0;
        point.s = 0;
        point.lane_id = 0;
        point.x_derivative = 0;
        point.y_derivative = 0;
        return point;
    }

    double distance_min = func_distance_square(reference_line.front(), x, y);
    int index_min = 0;
    for(int i = 1; i < reference_line.size(); i++)
    {
        double distance_temp = func_distance_square(reference_line.at(i), x, y);
        if(distance_temp < distance_min)
        {
            distance_min = distance_temp;
            index_min = i;
        }
    }

    int index_start = (index_min == 0)? index_min : index_min-1;
    int index_end = (index_min + 1 == reference_line.size()) ? index_min : index_min + 1;

    if(index_end == index_start)
    {
        return reference_line.at(index_start);
    }
    
    return FindProjectionPoint(reference_line.at(index_start), reference_line.at(index_end), x, y);
}

PathPoint PathMatcher::MatchToPath(const std::vector<PathPoint>& reference_line, const double s)
{
    auto comp = [](const PathPoint& p, const double s) {return p.s < s;};
    auto it_lower = std::lower_bound(reference_line.begin(), reference_line.end(), s, comp);
    if(it_lower == reference_line.begin())
    {
        return reference_line.front();
    }
    else if(it_lower == reference_line.end())
    {
        return reference_line.back();
    }
    else
    {
        return InterpolateUsingLinearApproximation(*(it_lower -1), *it_lower, s);
    }

}

std::pair<double, double> PathMatcher::GetPathFrenetCoordinate(const std::vector<PathPoint>&reference_line, const double x, const double y)
{
    auto matched_path_point = PathMatcher::MatchToPath(reference_line, x, y);
    double rtheta = matched_path_point.theta;
    double rx = matched_path_point.x;
    double ry = matched_path_point.y;
    double dx = x - rx;
    double dy = y - ry;
    double side = cos(rtheta)*dy - sin(rtheta)*dx;
    std::pair<double,double> relative_coordinate;
    relative_coordinate.first = matched_path_point.s;
    relative_coordinate.second = copysign(sqrt(dx*dx + dy*dy),side);
    return relative_coordinate;
}

double func_distance_square(const PathPoint& point, const double x, const double y)
{
    double dx = point.x - x;
    double dy = point.y - y;

    return dx*dx + dy*dy;

}

PathPoint PathMatcher::FindProjectionPoint(const PathPoint& p0, const PathPoint& p1, const double x, const double y)
{
    double v0x = x - p0.x;
    double v0y = y - p0.y;

    double v1x = p1.x - p0.x;
    double v1y = p1.y - p0.y;

    double v1_norm = sqrt(v1x*v1x + v1y*v1y);
    double dot = v0x*v1x + v0y*v1y;

    double delta_s = dot/v1_norm;

    double s = p0.s + delta_s;

    return InterpolateUsingLinearApproximation(p0, p1, s);



    
}

PathPoint InterpolateUsingLinearApproximation(const PathPoint& p0, const PathPoint& p1, const double s)
{
    double s0 = p0.s;
    double s1 = p1.s;

    PathPoint path_point;

    double weight = (s - s0)/(s1 - s0);

    double x = (1 - weight)*p0.x + weight*p1.x;
    double y = (1 - weight)*p0.y + weight*p1.y;

    double theta = (1 - weight)*p0.theta + weight*p1.theta;

    double kappa = (1 - weight)*p0.kappa + weight*p1.kappa;

    double dkappa = (1 - weight)*p0.dkappa + weight * p1.dkappa;

    path_point.x = x;
    path_point.y = y;
    path_point.theta = theta;
    path_point.kappa = kappa;
    path_point.dkappa = dkappa;
    path_point.s = s;

    return path_point;
}

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint& tp0, const TrajectoryPoint& tp1, const double time)
{
    PathPoint pp0 = tp0.path_point;
    PathPoint pp1 = tp1.path_point;

    double t0 = tp0.relative_time;
    double t1 = tp0.relative_time;
    TrajectoryPoint tp;
    tp.relative_time = time;
    tp.a = lerp(tp0.a, t0, tp1.a, t1, time);
    tp.v = lerp(tp0.v, t0, tp1.v, t1, time);
    tp.steer = lerp(tp0.steer, t0, tp1.steer, t1, time);
    tp.path_point.x = lerp(pp0.x, t0, pp1.x, t1, time);
    tp.path_point.y = lerp(pp0.y, t0, pp1.y, t1, time);
    tp.path_point.theta = lerp(pp0.theta, t0, pp1.theta, t1, time);
    tp.path_point.kappa = lerp(pp0.kappa, t0, pp1.kappa, t1, time);
    tp.path_point.dkappa = lerp(pp0.dkappa, t0, pp1.dkappa, t1, time);
    tp.path_point.ddkappa = lerp(pp0.ddkappa, t0, pp1.ddkappa, t1, time);
    tp.path_point.s = lerp(pp0.s, t0, pp1.s, t1, time);
    return tp;
}

