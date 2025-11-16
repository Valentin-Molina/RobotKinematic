cmake_minimum_required(VERSION 3.27)
project(RobotKinematic CXX)

find_package(SFML 2.5 COMPONENTS system window graphics REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

include_directories(include)

set(sources
    src/main.cpp
    src/window.cpp
    src/link.cpp
    src/robot.cpp
    src/robot2.cpp
    src/renderer.cpp
    src/planner.cpp
    src/window_transform.cpp
    src/ccd_inverse_kinematics.cpp
)

add_executable(robk ${sources})

target_link_libraries(robk sfml-graphics)