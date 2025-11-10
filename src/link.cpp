#include "link.hpp"
#include <eigen3/Eigen/src/Core/util/ForwardDeclarations.h>

Link::Link(double length, double angle) : length_(length), angle_(angle){};

double Link::getLength() const { return length_; }

void Link::setAngle(double angle) { angle_ = angle; }

double Link::getAngle() const { return angle_; };

Eigen::Vector2d Link::getLinkRelativePose() const
{
    return Eigen::Vector2d(getLength() * std::cos(getAngle()),
                           getLength() * std::sin(getAngle()));
}

Eigen::Vector2d
Link::getLinkAbsolutePose(const Eigen::Vector2d& frame_translation,
                          double frame_rotation) const
{
    Eigen::Matrix2d rotation;
    rotation << std::cos(frame_rotation), -std::sin(frame_rotation),
        std::sin(frame_rotation), std::cos(frame_rotation);
    return frame_translation + rotation * getLinkRelativePose();
}