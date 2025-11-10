#include "ccd_inverse_kinematics.hpp"

#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <optional>

std::optional<std::vector<double>>
CCDIKSolver::computeInverseKinematic(const Robot& robot,
                                     const Eigen::Vector2d& target) const
{
    std::vector<double> joint_angles(robot.getJointCount(), 0.0);
    // FIXME: Take a copy as argument.
    Robot robot_cp(robot);
    for (int i = 0; i < 50; i++) {
        auto joint = robot_cp.getBase();
        for (size_t joint_index = 0; joint_index < robot.getJointCount();
             joint_index++) {
            const auto end_effector    = robot_cp.getEndEffectorPosition();
            const auto joint_to_target = target - joint;
            const auto joint_to_end_effector = end_effector - joint;
            const double angle =
                computeSignedAngle(joint_to_end_effector, joint_to_target);
            joint_angles[joint_index] =
                robot_cp.getJointAngle(joint_index) + angle;
            robot_cp.updateJointAngle(joint_index, joint_angles[joint_index]);
            joint = robot_cp.getJointPosition(joint_index);
        }
    }
    std::cout << "End of effector position reached: " << std::endl
              << robot_cp.getEndEffectorPosition() << std::endl;

    return joint_angles;
}

double CCDIKSolver::computeSignedAngle(const Eigen::Vector2d& start,
                                       const Eigen::Vector2d& end)
{
    return std::atan2(start.x() * end.y() - start.y() * end.x(),
                      start.dot(end));
}