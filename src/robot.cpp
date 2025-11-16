#include "link.hpp"
#include "robot.hpp"

#include <cmath>
#include <cstddef>
#include <eigen3/Eigen/src/Core/Matrix.h>

Robot::Robot(Eigen::Vector2d base) : base_(base)
{
    links_ = {Link(0.7), Link(0.5), Link(0.3), Link(0.3)};
};

Robot::Robot(const Robot& robot) : base_(robot.base_)
{
    links_.reserve(robot.getJointCount());
    for (size_t joint_index = 0; joint_index < robot.getJointCount();
         joint_index++) {
        links_.push_back(Link(robot.links_[joint_index].getLength(),
                              robot.links_[joint_index].getAngle()));
    }
}

size_t Robot::getJointCount() const { return links_.size(); }

std::vector<Eigen::Vector2d> Robot::getJointPositions() const
{
    std::vector<Eigen::Vector2d> vectors;
    double angle = 0.0;
    Eigen::Vector2d previous_link_absolute_position;
    previous_link_absolute_position << 0.0, 0.0;
    vectors.reserve(links_.size());
    for (const auto& link : links_) {
        const auto link_absolute_position =
            link.getLinkAbsolutePose(previous_link_absolute_position, angle);
        vectors.push_back(link_absolute_position);
        previous_link_absolute_position = link_absolute_position;
        angle += link.getAngle();
    }
    return vectors;
}

Eigen::Vector2d Robot::getJointPosition(size_t joint_index) const
{
    // FIXME: should be optimized
    return getJointPositions()[joint_index];
}

Eigen::Vector2d Robot::getEndEffectorPosition() const
{
    // FIXME: should be optimized
    return getJointPositions()[getJointCount() - 1];
}

Eigen::Vector2d Robot::getBase() const { return base_; }

void Robot::updateJointAngle(size_t link_index, double link_angle)
{
    links_[link_index].setAngle(link_angle);
}

double Robot::getJointAngle(size_t link_index) const
{
    return links_[link_index].getAngle();
}
