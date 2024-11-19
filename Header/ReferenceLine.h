#pragma once

#include "../sldd/Points.h"
#include <vector>


class ReferenceLine
{
public:
    ReferenceLine(std::vector<SpeedLimit>& speedlimit, double start_s, double end_s);
    
    double GetSpeedLimitFromS(const double s) const;

private:
    std::vector<SpeedLimit> speed_limit_;
    double start_s_;
    double end_s_;
};
