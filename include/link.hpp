#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/src/Core/Matrix.h>

class Link
{
  public:
    Link(double length, double angle = 0.0);

    double getLength() const;
    void setAngle(double angle);
    double getAngle() const;
    Eigen::Vector2d getLinkRelativePose() const;
    Eigen::Vector2d
    getLinkAbsolutePose(const Eigen::Vector2d& frame_translation,
                        double frame_rotation) const;

  private:
    double length_;
    double angle_;
};