#include "../Header/ReferenceLine.h"


ReferenceLine::ReferenceLine(std::vector<SpeedLimit>& speedlimit, double start_s,double end_s)
{
    if(speedlimit.empty())
    {
        return;
    }
    for(const auto& spdlmt:speedlimit)
    {
        speed_limit_.push_back(spdlmt);
    }
    start_s_ = start_s;
    end_s_ = end_s;
}

double ReferenceLine::GetSpeedLimitFromS(const double s) const
{
    double speed_limit = FLAGS_planning_upper_speed_limit;
    return speed_limit;
}