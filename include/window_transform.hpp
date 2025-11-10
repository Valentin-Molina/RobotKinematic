#pragma once

#include <SFML/System/Vector2.hpp>
#include <eigen3/Eigen/Core>

class WindowTransform
{
  public:
    WindowTransform(const uint window_width, const uint window_height,
                    const double robot_to_window_rotation_angle,
                    const double robot_to_window_scale);
    ~WindowTransform() = default;

    sf::Vector2f transformToWindow(const Eigen::Vector2d& vector) const;
    Eigen::Vector2d transformToRobot(const sf::Vector2f& vector) const;

  private:
    Eigen::Matrix2d robot_to_window_rotation_;
    const Eigen::Vector2d robot_to_window_translation_;
    const double robot_to_window_scale_;
};