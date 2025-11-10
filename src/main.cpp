#include "ccd_inverse_kinematics.hpp"
#include "renderer.hpp"
#include "window.hpp"

#include <cmath>
#include <eigen3/Eigen/Core>

#include "ccd_inverse_kinematics.hpp"
#include "planner.hpp"
#include "robot.hpp"
#include "window_transform.hpp"

int main()
{

    const uint window_width                     = 1920;
    const uint window_height                    = 1080;
    const double robot_to_window_rotation_angle = M_PI;
    const double robot_to_window_scale          = 200;
    WindowTransform window_transform(window_width, window_height,
                                     robot_to_window_rotation_angle,
                                     robot_to_window_scale);

    Robot robot(Eigen::Vector2d(0.0, 0.0));
    CCDIKSolver solver;
    Planner planner(robot, solver);
    Renderer renderer(robot, window_transform);
    Window window(window_width, window_height, renderer, window_transform,
                  planner);
    window.display();

    return 0;
}