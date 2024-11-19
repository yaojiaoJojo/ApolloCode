#pragma once

#include <vector>
#include "../sldd/Points.h"
#include <algorithm>
#include <limits>

using namespace std;


class Box2d
{
private:
    /* data */
    std::vector<Vec2d> corner_;
    Vec2d center_;
    double length_;
    double width_;
    double theta_;
    double half_length_;
    double half_width_;
    double cos_theta_;
    double sin_theta_;
    double max_x_ = std::numeric_limits<double>::lowest();
    double min_x_ = std::numeric_limits<double>::max();
    double max_y_ = std::numeric_limits<double>::lowest();
    double min_y_ = std::numeric_limits<double>::max();
public:
    Box2d(const Vec2d &center, const double theta, const double length, const double width);
    BBox2D GetBox2d() const;
    void initcorner();
    std::vector<Vec2d> GetAllCorners() const;
};
