#include "renderer.hpp"
#include "window_transform.hpp"

#include <cmath>

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/ConvexShape.hpp>
#include <SFML/Graphics/PrimitiveType.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/System/Vector2.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/src/Core/Matrix.h>

Renderer::Renderer(const Robot& robot, const WindowTransform& window_transform)
    : robot_(robot), window_transform_(window_transform)
{
}

void Renderer::render(sf::RenderWindow& window) const
{
    window.clear(sf::Color(30, 30, 46, 255));
    drawFrame(window);
    const auto base_position =
        window_transform_.transformToWindow(robot_.getBase());
    double angle            = 0.0;
    const double link_width = 2.0;

    sf::CircleShape base(20);
    auto color = sf::Color(200.0, 70, 50);
    base.setFillColor(color);
    base.setOrigin(20, 20);
    base.setPosition(base_position);
    window.draw(base);

    auto previous_joint_position = base_position;
    for (const auto& joint_position : robot_.getJointPositions()) {
        const auto joint_position_in_window =
            window_transform_.transformToWindow(joint_position);
        color.g += 40; // HACK: will not scale.
        drawLink(window, previous_joint_position, joint_position_in_window,
                 color);
        previous_joint_position = joint_position_in_window;
    }

    window.display();
}

void Renderer::drawRectangle(sf::RenderWindow& window,
                             const sf::Vector2f& start, const sf::Vector2f& end,
                             float width, sf::Color color) const
{
    const auto normal_axis = Renderer::computeNormalAxis(start, end);
    sf::ConvexShape rectangle;
    rectangle.setPointCount(4);
    rectangle.setPoint(0, start + normal_axis * (width / 2.0f));
    rectangle.setPoint(1, start - normal_axis * (width / 2.0f));
    rectangle.setPoint(2, end - normal_axis * (width / 2.0f));
    rectangle.setPoint(3, end + normal_axis * (width / 2.0f));
    rectangle.setFillColor(color);
    window.draw(rectangle);
}

void Renderer::drawFrame(sf::RenderWindow& window) const
{
    const sf::Vector2f center_in_window =
        window_transform_.transformToWindow(Eigen::Vector2d(0.0, 0.0));
    const sf::Vector2f axis_x_in_window =
        window_transform_.transformToWindow(Eigen::Vector2d(1.0, 0.0));
    const sf::Vector2f axis_y_in_window =
        window_transform_.transformToWindow(Eigen::Vector2d(0.0, 1.0));
    drawRectangle(window, center_in_window, axis_x_in_window, 2.0);
    sf::CircleShape axis_x_tip(
        3.f, 4); // HACK: Use a square instead of a triangle to avoid rotation.
    axis_x_tip.setOrigin(3.f, 3.f);
    axis_x_tip.setPosition(axis_x_in_window);
    window.draw(axis_x_tip);
    drawRectangle(window, center_in_window, axis_y_in_window, 2.0);
    sf::CircleShape axis_y_tip(
        3.f, 4); // HACK: Use a square instead of a triangle to avoid rotation.
    axis_y_tip.setOrigin(3.f, 3.f);
    axis_y_tip.setPosition(axis_y_in_window);
    window.draw(axis_y_tip);
}

void Renderer::drawLink(sf::RenderWindow& window,
                        const sf::Vector2f& previous_joint_position,
                        const sf::Vector2f& current_joint_position,
                        sf::Color color) const
{
    const float link_width   = 20.f;
    const float joint_radius = 20.f;

    drawRectangle(window, previous_joint_position, current_joint_position,
                  link_width, color);

    sf::CircleShape link_start(link_width / 2.f);
    link_start.setOrigin(link_width / 2.f, link_width / 2.f);
    link_start.setPosition(previous_joint_position);
    link_start.setFillColor(color);
    window.draw(link_start);

    sf::CircleShape joint(joint_radius);
    joint.setOrigin(joint_radius, joint_radius);
    joint.setPosition(current_joint_position);
    joint.setFillColor(color);
    window.draw(joint);
}

sf::Vector2f Renderer::computeNormalAxis(const sf::Vector2f& start,
                                         const sf::Vector2f& end)
{
    const auto vect  = end - start;
    const float norm = sqrt(vect.x * vect.x + vect.y * vect.y);
    return sf::Vector2f(-vect.y / norm, vect.x / norm);
}