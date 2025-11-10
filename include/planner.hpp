#pragma once

#include <eigen3/Eigen/Core>
#include <vector>

#include "inverse_kinematics.hpp"
#include "robot.hpp"

class Planner
{
  public:
    Planner(Robot& robot, const IKSolver& ik_solver);
    ~Planner() = default;

    void update();
    void updateCartesianTarget(Eigen::Vector2d cartesian_coordinates);

  private:
    Robot& robot_;
    const IKSolver& ik_solver_;
    std::vector<double> desired_joint_positions_;

    double time_;
    bool has_been_updated;
};