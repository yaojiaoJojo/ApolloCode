#include "../Header/ReferenceLineInfo.h"


ReferenceLineInfo::ReferenceLineInfo(const ReferenceLine& reference_line):reference_line_(reference_line){}

void ReferenceLineInfo::SetCruseSpeed(double speed)
{
    cruse_speed_ = speed;
    base_cruse_speed_ = speed;
}