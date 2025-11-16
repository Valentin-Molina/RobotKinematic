#include "robot2.hpp"
#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <vector>

Joint2::Joint2(double angle) : angle_(angle) {}

double Joint2::getAngle() const { return angle_; }

void Joint2::setAngle(double value) { angle_ = value; }

Link2::Link2(std::string name, double length, double width)
    : name_(name), length_(length), width_(width)
{
}

double Link2::getLength() const { return length_; }

double Link2::getWidth() const { return width_; }

std::string_view Link2::getName() const { return name_; }

Robot2::Builder::Builder(const Eigen::Vector2d& base_position)
    : base_position_(base_position)
{
}

void Robot2::Builder::addLink(Link2 link)
{
    links_.push_back(link);
    // Add entry for new link a adjacency matrix.
    for (auto& row : adjacency_matrix_) {
        row.push_back(link_not_adjacent);
    }
    adjacency_matrix_.push_back(
        std::vector(adjacency_matrix_.size() + 1, link_not_adjacent));
}

void Robot2::Builder::addJoint(Joint2 joint2, std::string link_a_name,
                               std::string link_b_name)
{
    size_t index_a = getLinkIndexByName(link_a_name);
    size_t index_b = getLinkIndexByName(link_b_name);
    joints_.push_back(joint2);
    adjacency_matrix_[index_a][index_b] = joints_.size() - 1;
    adjacency_matrix_[index_b][index_a] = joints_.size() - 1;
}

size_t
Robot2::Builder::getLinkIndexByName(const std::string_view link_name) const
{
    for (size_t index = 0; index < links_.size(); index++) {
        if (link_name == links_[index].getName()) {
            return index;
        }
    }
    std::stringstream error;
    error << "Link " << link_name << " unknown.";
    throw std::runtime_error(error.str());
}

Robot2 Robot2::Builder::build()
{
    return Robot2(std::move(base_position_), std::move(links_),
                  std::move(joints_), std::move(adjacency_matrix_));
}

Robot2::Robot2(Eigen::Vector2d&& base_position, std::vector<Link2>&& links,
               std::vector<Joint2>&& Joint2s,
               std::vector<std::vector<size_t>>&& adjacency_matrix)
{
}