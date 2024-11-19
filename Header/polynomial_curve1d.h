#pragma once
#include <math.h>
#include <array>
#include <iostream>

class quartic_polynomial_curve1d
{
private:
    /* data */
    std::array<double, 5> coef4_;
    // std::array<double, 3> start_state_{{0.0, 0.0, 0.0}};
    // std::array<double, 3> end_condition_{{0.0, 0.0, 0.0}};
public:
    void ComputeCoefficients(const double x0, const double dx0, const double ddx0, 
                                const double dx1, const double ddx1, const double p);
    std::array<double, 5> SetCoef4(){return coef4_;}
};

class quintic_polynomial_curve1d
{
private:
    std::array<double, 6> coef5_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    // std::array<double, 3> start_state_{{0.0, 0.0, 0.0}};
    // std::array<double, 3> end_condition_{{0.0, 0.0, 0.0}};
public:
    void computeCoefficients(const double x0, const double dx0, const double ddx0,
                                const double x1, const double dx1, const double ddx1, const double p);
    std::array<double, 6> SetCoef4(){return coef5_;}
};


