#include "window_transform.hpp"

WindowTransform::WindowTransform(const uint window_width,
                                 const uint window_height,
                                 const double robot_to_window_rotation_angle,
                                 const double robot_to_window_scale)
    : robot_to_window_rotation_(),
      robot_to_window_translation_(window_width / 2.0, window_height / 2.0),
      robot_to_window_scale_(robot_to_window_scale)
{
    robot_to_window_rotation_ << std::cos(robot_to_window_rotation_angle),
        -std::sin(robot_to_window_rotation_angle),
        std::sin(robot_to_window_rotation_angle),
        std::cos(robot_to_window_rotation_angle);
}

sf::Vector2f
WindowTransform::transformToWindow(const Eigen::Vector2d& vector) const
{
    const auto transform_vector =
        robot_to_window_scale_ * robot_to_window_rotation_ * vector +
        robot_to_window_translation_;
    return sf::Vector2f(transform_vector.x(), transform_vector.y());
}

Eigen::Vector2d
WindowTransform::transformToRobot(const sf::Vector2f& vector) const
{
    const Eigen::Vector2d window_position(vector.x, vector.y);
    return (1.0 / robot_to_window_scale_) *
           robot_to_window_rotation_.transpose() *
           (window_position - robot_to_window_translation_);
}
