#pragma once

#include "inverse_kinematics.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>

class CCDIKSolver : public IKSolver
{

  public:
    CCDIKSolver()          = default;
    virtual ~CCDIKSolver() = default;

    std::optional<std::vector<double>>
    computeInverseKinematic(const Robot& robot,
                            const Eigen::Vector2d& coordinates) const;

    static double computeSignedAngle(const Eigen::Vector2d& start,
                                     const Eigen::Vector2d& end);
};
