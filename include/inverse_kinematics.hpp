#pragma once

#include <optional>
#include <vector>

#include <eigen3/Eigen/Core>

#include "robot.hpp"

class IKSolver
{
  public:
    virtual ~IKSolver() = default;

    virtual std::optional<std::vector<double>>
    computeInverseKinematic(const Robot& robot,
                            const Eigen::Vector2d& coordinates) const = 0;
};