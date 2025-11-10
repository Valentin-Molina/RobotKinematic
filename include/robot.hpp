#pragma once

#include <cstddef>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include "link.hpp"

class Robot
{
  public:
    Robot(Eigen::Vector2d base);
    Robot(const Robot& robot);

    size_t getJointCount() const;
    Eigen::Vector2d getBase() const;
    std::vector<Eigen::Vector2d> getJointPositions() const;
    Eigen::Vector2d getJointPosition(size_t joint_index) const;
    Eigen::Vector2d getEndEffectorPosition() const;
    void updateJointAngle(size_t link_index, double link_angle);
    double getJointAngle(size_t link_index) const;

  private:
    std::vector<Link> links_;
    Eigen::Vector2d base_;
};