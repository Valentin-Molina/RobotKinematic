#pragma once

#include <SFML/Graphics.hpp>

#include "planner.hpp"
#include "renderer.hpp"
#include "window_transform.hpp"

class Window
{
  public:
    Window(unsigned int width, unsigned int heigth, const Renderer& renderer,
           const WindowTransform& window_transform, Planner& planner);
    ~Window() = default;

    void display();

  private:
    void handleEvent();

    sf::VideoMode vm_;
    sf::RenderWindow window_;
    sf::View view_;
    const Renderer& renderer_;
    const WindowTransform& window_transform_;
    Planner& planner_;

    bool mouse_pressed_;
};