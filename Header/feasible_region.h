#pragma once

#include <algorithm>
#include <array>
#include <iostream>
#include "../sldd/Points.h"

class FeasibleRegion
{
private:
    /* data */
    std::array<double, 3> init_s_;
    double t_at_zero_speed_;
    double s_at_zero_speed_;
public:

    FeasibleRegion(const std::array<double, 3>& init_s);

    double SUpper(const double t) const;
    double SLower(const double t) const;
    double VUpper(const double t) const;
    double VLower(const double t) const;
    double TLower(const double s) const;

};

