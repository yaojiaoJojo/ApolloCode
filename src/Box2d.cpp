
#include "../sldd/Box2d.h"
#include <math.h>

Box2d::Box2d(const Vec2d &center, const double theta, const double length, const double width)
{
    center_ = center;
    theta_ = theta;
    length_ = length;
    width_ = width;
    half_width_ = width * 0.5;
    half_length_ = length *0.5;
    cos_theta_ = cos(theta);
    sin_theta_ = sin(theta);
    initcorner();
    
}

BBox2D Box2d::GetBox2d() const
{
    BBox2D box;
    box.xmax = max_x_;
    box.xmin = min_x_;
    box.ymax = max_y_;
    box.ymin = min_y_;
    return box;
}
void Box2d::initcorner()
{
    const double dx1 = half_length_*cos_theta_;
    const double dy1 = half_length_*sin_theta_;

    const double dx2 = half_width_*sin_theta_;
    const double dy2 = half_width_*cos_theta_;
    const Vec2d right_up = {center_.x + dx1 + dx2, center_.y + dy1 - dy2};
    const Vec2d left_up = {center_.x + dx1 - dx2, center_.y + dy1 + dy2};
    const Vec2d left_down = {center_.x - dx1 - dx2, center_.y - dy1 + dy2};
    const Vec2d right_down = {center_.x - dx1 - dx2, center_.y - dy1 - dy2};


    corner_.clear();

    corner_.emplace_back(right_up); //right_up
    corner_.emplace_back(left_up); // left_up
    corner_.emplace_back(left_down); //left_down
    corner_.emplace_back(right_down); //right_down

    for(auto& it:corner_)
    {
        max_x_ = fmax(it.x, max_x_);
        min_x_ = fmin(it.x, min_x_);
        max_y_ = fmax(it.y, max_y_);
        min_y_ = fmin(it.y, min_y_);
    }
}

std::vector<Vec2d> Box2d::GetAllCorners() const 
{
    return corner_;
}



