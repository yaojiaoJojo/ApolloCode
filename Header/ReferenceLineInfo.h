#pragma once

#include "../sldd/Points.h"
#include "../Header/ReferenceLine.h"

class ReferenceLineInfo
{
private:
    ReferenceLine reference_line_;
    PlanningTarget planning_target_;
    double cruse_speed_;
    double base_cruse_speed_;
public:
    ReferenceLineInfo(const ReferenceLine& reference_line);
    void SetCruseSpeed(double speed);
    const PlanningTarget& planning_target() const {return planning_target_;}
   
};

