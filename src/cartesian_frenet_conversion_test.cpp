#include <iostream>

#include "../Header/cartesian_frenet_conversion.h"
#include "math.h"
// #include "Discretized.cpp"
#include "../Header/path_matcher.h"
#include "../Header/path_time_graph.h"
#include "../Header/end_condition_sampler.h"
#include "../Header/polynomial_curve1d.h"
#include <algorithm>

using namespace std;




int main()
{
    cout << "hello, apollo lattice" << endl;

/* 测试 cartesian to Frenet 
    double rs = 10.0;
    double rx = 0.0;
    double ry = 0.0;
    double rtheta = M_PI / 4.0;
    double rkappa = 0.1;
    double rdkappa = 0.01;
    double x = -1.0;
    double y = 1.0;
    double v = 2.0;
    double a = 0.0;
    double theta = M_PI / 3.0;
    double kappa = 0.11;

    rs = 121.47234487952613;
    rx = -30.671551199574598;
    ry = 37.84892645771959;
    rtheta = 0.7121457559992703;
    rkappa = 1/50.0;
    rdkappa = 0;
    x = -30.8;
    y = 38;
    v = 10.0;
    a = 3;
    theta = 0.3*M_PI;
    kappa = 1/(50 + 0.25*3.75);
    // s_conditions = {121.47234488, 9.77746725, 3.3785997};
    // d_conditions = {0.19829854, 0.23356364, -0.00100663};

    std::array<double, 3> s_conditions;
    std::array<double, 3> d_conditions;

  CartesianFrenetConverter::cartesian_to_frenet(
      rs, rx, ry, rtheta, rkappa, rdkappa, x, y, v, a, theta, kappa,
      &s_conditions, &d_conditions);

      cout << "s_condition[0] = " << s_conditions.at(0) << " s_condition[1] = " << s_conditions.at(1)
      << " s_condition[2] = " << s_conditions.at(2) << endl;
      cout << "d_condition[0] = " << d_conditions.at(0) << " d_condition[1] = " << d_conditions.at(1)
      << " d_condition[2] = " << d_conditions.at(2) << endl;
      cout << endl;
*/

/*测试 Frenet To Cartesian 
  double x_out;
  double y_out;
  double theta_out;
  double kappa_out;
  double v_out;
  double a_out;

  CartesianFrenetConverter::frenet_to_cartesian(
      rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, &x_out,
      &y_out, &theta_out, &kappa_out, &v_out, &a_out);

    cout << "x_out = " << x_out << " y_out = " << y_out << " theta_out = " << theta_out << endl;
    cout << "kappa_out = " << kappa_out << " v_out = " << v_out << " a_out = " << a_out << endl;
    cout << endl;
*/

    vector<PathPoint> referent_line;
    float R = 20;
    float x = 10;
    float y = 0;
    double start_arc = M_PI;
    double end_arc = 0.5*M_PI;
    double planning_x = -4.5;
    double planning_y = 14.14;

    PathPoint match_point;
    cout << "M_PI = " << M_PI << endl;

    for(double angle = start_arc; angle > end_arc; angle = angle - 0.15)
    {
        PathPoint path_point;
        double x_point = x + R*cos(angle);
        double y_point  = y + R*sin(angle);
        double theta_point = atan2(y_point, x_point);
        double kappa_point = 1/R;
        double dkappa_point = 0;
        double ddkappa_point = 0;

        double s = R*(M_PI - angle);

        path_point.x = x_point;
        path_point.y = y_point;
        path_point.theta = theta_point;
        path_point.kappa = kappa_point;
        path_point.dkappa = dkappa_point;
        path_point.ddkappa = ddkappa_point;
        path_point.s = s;
        referent_line.push_back(path_point);
    }

    cout << "current PathPoint is ( " << planning_x << ", " << planning_y << " )" << endl;

    match_point = PathMatcher::MatchToPath(referent_line,planning_x,planning_y);

    cout << "The Match Point is (" << match_point.x << ", " << match_point.y << " )" << ";" << "s = " << match_point.s << endl;

    cout << "The Match Point theta = " << match_point.theta << "; " << "kappa = " << match_point.kappa << endl;

    cout << "The Match Point dkappa = " << match_point.dkappa << "; ddkappa = " << match_point.ddkappa << endl;
    
    std::vector<int> test_lower_bound;
    test_lower_bound.push_back(1);
    test_lower_bound.push_back(2);
    test_lower_bound.push_back(3);
    test_lower_bound.push_back(7);
    test_lower_bound.push_back(9);
    test_lower_bound.push_back(10);

    

    std::vector<int>::const_iterator it = test_lower_bound.begin();

    cout << "test_lower_bound = " << *(it+1) << endl;
    
    // int target = 5;
    
    // auto lower_value = std::lower_bound(test_lower_bound.begin(), test_lower_bound.end(),target);

    // std::cout << "lower_value = " << *(lower_value-1) << std::endl;
    // std::cout << std::endl;

    quartic_polynomial_curve1d coef;
    std::array<double, 5> coef_ret;
    coef.ComputeCoefficients(3,1,0,0.5,0.05,3);
    coef_ret = coef.SetCoef4();
    std::cout << "coef[3] = " << coef_ret.at(3) << std::endl;
  

    return 0;
}