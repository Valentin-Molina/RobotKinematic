#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Vector2.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include "robot.hpp"
#include "window_transform.hpp"

class Renderer
{
  public:
    Renderer(const Robot& robot, const WindowTransform& window_transform);

    void render(sf::RenderWindow& window) const;

  private:
    void drawRectangle(sf::RenderWindow& window, const sf::Vector2f& start,
                       const sf::Vector2f& end, float width,
                       sf::Color color = sf::Color(255, 255, 255, 255)) const;
    void drawFrame(sf::RenderWindow& window) const;
    void drawLink(sf::RenderWindow& window, const sf::Vector2f& previous_joint,
                  const sf::Vector2f& current_joint, sf::Color color) const;

    static sf::Vector2f computeNormalAxis(const sf::Vector2f& start,
                                          const sf::Vector2f& end);

    const Robot& robot_;
    const WindowTransform& window_transform_;
};