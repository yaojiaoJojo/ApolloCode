#include "../Header/polynomial_curve1d.h"

void quartic_polynomial_curve1d::ComputeCoefficients(const double x0, const double dx0, const double ddx0, 
                                                        const double dx1,const double ddx1, const double p)
{
    if(p < 0)
    {
        std::cout << "time less than zero!!!" << std::endl;
        return;
    }
    coef4_[0] = x0;
    coef4_[1] = dx0;
    coef4_[2] = 0.5*ddx0;
    const double b0 = dx1 - ddx0*p - dx0;
    const double b1 = ddx1 - ddx0;
    coef4_[3] = (3*b0 - b1*p)/(3*p*p);
    coef4_[4] = (-2*b0 + b1*p)/(4*p*p*p);
}


void quintic_polynomial_curve1d::computeCoefficients(const double x0, const double dx0, const double ddx0,
                                const double x1, const double dx1, const double ddx1, const double p)
    {
        if(p < 0)
        {
            std::cout << "time is less than zero!!!" << std::endl;
            return;
        }

        coef5_[0] = x0;
        coef5_[1] = dx0;
        coef5_[2] = 0.5*ddx0;
        double p2 = p*p;
        double p3 = p2*p;
        const double c0 = (x1 - 0.5*p2*ddx0 - dx0*p - x0)/p3;
        const double c1 = (dx1 - ddx0*p - dx0)/p2;
        const double c2 = (ddx1 -ddx0)/p;
        coef5_[3] = 0.5*(20*c0 - 8.0*c1 + c2);
        coef5_[4] = (-15*c0 + 7*c1 - c2)/p;
        coef5_[5] = (6*c0 - 3*c1 + 0.5*c2)/p2;
    }