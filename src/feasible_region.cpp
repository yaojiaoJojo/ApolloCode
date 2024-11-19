#include "../Header/feasible_region.h"
#include <math.h>

FeasibleRegion::FeasibleRegion(const std::array<double, 3>& init_s)
{
    init_s_ = init_s;
    double v = init_s_[1];
    if(v < 0.0)
    {
        std::cout << "speed lower than zero!!!" << std::endl;
        return;
    }

    const double max_deceleration = -FLAFS_longitudinal_acceleration_lower_bound;
    t_at_zero_speed_ = v/max_deceleration;
    s_at_zero_speed_ = init_s_[0] + v*v/(2*max_deceleration);
}




double FeasibleRegion::SUpper(const double t) const
{
    if(t < 0.0)
    {
        std::cout << "time less than zero!!!" << std::endl;
        return -9999;
    }
   return init_s_[0] + init_s_[1]*t + 0.5*t*t*FLAGS_longitudinal_acceleration_upper_bound;
}

double FeasibleRegion::SLower(const double t) const
{
    if(t < t_at_zero_speed_)
    {
        return init_s_[0] + init_s_[1]*t + 0.5*t*t*FLAFS_longitudinal_acceleration_lower_bound;
    }
    return s_at_zero_speed_;
}

double FeasibleRegion::VUpper(const double t) const
{
    return init_s_[1] + FLAGS_longitudinal_acceleration_upper_bound*t;
}

double FeasibleRegion::VLower(const double t) const
{
    return t > t_at_zero_speed_ ? 0 : init_s_[1] + FLAFS_longitudinal_acceleration_lower_bound*t;
}


double FeasibleRegion::TLower(const double s) const
{
    if(s - init_s_[0] < 0)
    {
        std::cout << "delta s less than zero!!!" << std::endl;
        return -9999;
    }
    double delta_s = s - init_s_[0];
    double v = init_s_[1];
    double a = FLAGS_longitudinal_acceleration_upper_bound;
    double t_lower = (sqrt(v*v + 2*a*delta_s) - v)/a;
    return t_lower;
}