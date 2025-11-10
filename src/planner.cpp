#include "planner.hpp"

#include <cstddef>
#include <iostream>
#include <ostream>
#include <stdexcept>

Planner::Planner(Robot& robot, const IKSolver& ik_solver)
    : robot_(robot), ik_solver_(ik_solver),
      desired_joint_positions_(robot.getJointCount(), 0.0), time_(0.0)
{
    desired_joint_positions_[0] = M_PI;
    desired_joint_positions_[1] = M_PI;
    desired_joint_positions_[2] = M_PI;
    has_been_updated            = false;
}

void Planner::update()
{
    const double articular_vel = M_PI / 2.0;
    const double delta_t       = 0.01;
    for (size_t joint_index = 0; joint_index < robot_.getJointCount();
         joint_index++) {
        const double current_angle = robot_.getJointAngle(joint_index);
        const double desired_joint_angle =
            desired_joint_positions_[joint_index];
        const double delta = std::abs(desired_joint_angle - current_angle);
        const double direction =
            desired_joint_angle - current_angle < 0 ? -1.0 : 1.0;
        const double max_movement = delta_t * articular_vel;
        robot_.updateJointAngle(joint_index,
                                current_angle +
                                    direction * std::min(delta, max_movement));
    }
    time_ += delta_t;
}

void Planner::updateCartesianTarget(Eigen::Vector2d cartesian_coordinates)
{
    std::cout << "New cartesian coordinates received: " << std::endl
              << cartesian_coordinates << std::endl;
    const auto joint_coordinates =
        ik_solver_.computeInverseKinematic(robot_, cartesian_coordinates);
    if (!joint_coordinates.has_value()) {
        std::cerr
            << "Unable to compute joint coordinates to reach desired position"
            << std::endl
            << cartesian_coordinates << std::endl;
        return;
    }
    if (joint_coordinates.value().size() != robot_.getJointCount()) {
        throw std::runtime_error(
            "Ik solver outputs doesn't correspond to robot.");
    }
    desired_joint_positions_ = joint_coordinates.value();
}