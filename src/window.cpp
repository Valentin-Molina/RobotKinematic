#include "planner.hpp"

#include <iostream>
#include <thread>

#include <SFML/System/Vector2.hpp>
#include <eigen3/Eigen/Core>

#include "window.hpp"

Window::Window(unsigned int width, unsigned int height, const Renderer& render,
               const WindowTransform& window_transform, Planner& planner)
    : vm_(width, height), window_(vm_, "Test", sf::Style::Default),
      view_(sf::FloatRect(0, 0, width, height)), renderer_(render),
      window_transform_(window_transform), planner_(planner),
      mouse_pressed_(false)
{
    window_.setView(view_);
    window_.clear();
}

void Window::display()
{
    while (window_.isOpen()) {
        handleEvent();
        planner_.update();
        renderer_.render(window_);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void Window::handleEvent()
{
    sf::Event event;
    bool mouse_pressed = false;
    while (window_.pollEvent(event)) {

        if (event.type == sf::Event::Closed) {
            window_.close();
            return;
        }
        if (event.type == sf::Event::MouseButtonPressed) {
            mouse_pressed = true;
        }
        if (event.type == sf::Event::MouseButtonReleased) {
            mouse_pressed = false;
        }
        if (mouse_pressed && !mouse_pressed_) {
            const sf::Vector2f mouse_position(event.mouseButton.x,
                                              event.mouseButton.y);
            std::cout << "Mouse clicked at (" << event.mouseButton.x << ", "
                      << event.mouseButton.y << ")" << std::endl;
            planner_.updateCartesianTarget(
                window_transform_.transformToRobot(mouse_position));
        }
        mouse_pressed_ = mouse_pressed;
    }
}