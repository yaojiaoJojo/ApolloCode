#include "../Header/cartesian_frenet_conversion.h"
#include <cmath>

double NormalizeAngle(double angle)
{
    double a = fmod(angle + M_PI, 2*M_PI);
    if(a < 0.0)
    {
        a += 2*M_PI;
    }

    return a - M_PI;
}

void CartesianFrenetConverter::cartesian_to_frenet(const double rs, const double rx, const double ry, 
    const double rtheta, const double rkappa, const double rdkappa, const double x, 
    const double y, const double v, const double a, const double theta, const double kappa,
    std::array<double, 3>* const ptr_s_condition,std::array<double, 3>* const ptr_d_condition)
    {
        const double dx = x - rx;
        const double dy = y - ry;

        const double cos_theta_r = std::cos(rtheta);
        const double sin_theta_r = std::sin(rtheta);

        const double cross_rd_nd = cos_theta_r*dy - sin_theta_r*dx;
        
        ptr_d_condition->at(0) = copysign(sqrt(dx*dx + dy*dy),cross_rd_nd);
        

        const double delta_theta = theta - rtheta;
        const double tan_delat_theta = tan(delta_theta);
        const double cos_delta_theta = cos(delta_theta);

        const double one_minus_kappa_r_d = 1 - ptr_d_condition->at(0)*rkappa;

        ptr_d_condition->at(1) = one_minus_kappa_r_d*tan_delat_theta;

        const double kappa_r_d_prime = rdkappa*ptr_d_condition->at(0) + ptr_d_condition->at(1)*rkappa;

        ptr_d_condition->at(2) = -kappa_r_d_prime*tan(delta_theta) + one_minus_kappa_r_d*(one_minus_kappa_r_d*kappa/cos_delta_theta - rkappa)/cos_delta_theta;

        ptr_s_condition->at(0) = rs;

        ptr_s_condition->at(1) = v*cos_delta_theta/one_minus_kappa_r_d;

        const double delta_theta_prime = one_minus_kappa_r_d/cos_delta_theta*kappa - rkappa;

        ptr_s_condition->at(2) = (a*cos_delta_theta - (ptr_s_condition->at(1)*ptr_s_condition->at(1))*(ptr_d_condition->at(1)*delta_theta_prime - kappa_r_d_prime))/one_minus_kappa_r_d;
        
    }

void CartesianFrenetConverter::cartesian_to_frenet(const double rs, const double rx, const double ry,
    const double rtheta, const double x, const double y,double *ptr_s, double *ptr_d)
    {
        const double dx = x - rx;
        const double dy = y - ry;

        const double cos_theta_r = cos(rtheta);
        const double sin_theta_r = sin(rtheta);

        const double cross_rd_nd = cos_theta_r*dy - sin_theta_r*dx;

        *ptr_d = copysign(sqrt(dx*dx + dy*dy),cross_rd_nd);

        *ptr_s = rs;

    }

void CartesianFrenetConverter::frenet_to_cartesian(const double rs, const double rx,const double ry,
    const double rtheta, const double rkappa, const double rdkappa,
    const std::array<double, 3>& s_condition,const std::array<double, 3>& d_condition,
    double* const ptr_x, double* const ptr_y, double* const ptr_theta,double* const ptr_kappa, 
    double* const ptr_v, double* const ptr_a)
    {
        const double sin_theta_r = sin(rtheta);
        const double cos_theta_r = cos(rtheta);

        *ptr_x = rx - d_condition.at(0)*sin_theta_r;
        *ptr_y = ry + d_condition.at(0)*cos_theta_r;

        const double one_minus_kappa_r_d = 1 - rkappa*d_condition.at(0);
        const double tan_delta_theta = d_condition.at(1)/one_minus_kappa_r_d;

        *ptr_theta = NormalizeAngle(atan2(d_condition.at(1),one_minus_kappa_r_d) + rtheta);

        const double d_dot = d_condition.at(1) * s_condition.at(1);
        *ptr_v = sqrt(s_condition.at(1)*one_minus_kappa_r_d*s_condition.at(1)*one_minus_kappa_r_d + d_dot*d_dot);
        
        const double cos_delta_theta = cos(*ptr_theta - rtheta);
        const double kappa_r_d_prime = rdkappa*d_condition.at(0) + rkappa*d_condition.at(1);
        
        *ptr_kappa = (((d_condition.at(2) + kappa_r_d_prime * tan_delta_theta) *
                 cos_delta_theta * cos_delta_theta) /
                    (one_minus_kappa_r_d) +
                rkappa) *
               cos_delta_theta / (one_minus_kappa_r_d);
        const double delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * (*ptr_kappa) - rkappa;


        *ptr_a = s_condition.at(2) * one_minus_kappa_r_d / cos_delta_theta + 
                s_condition.at(1) * s_condition.at(1) / cos_delta_theta *
               (d_condition.at(1) * delta_theta_prime - kappa_r_d_prime);

    }
    