#include "node.h"

using namespace Eigen;

Node::Node(Vector3d loc)
      :  m_loc(loc) {
}

Vector3d Node::get_loc() {
    return m_loc;
}

Vector3d Node::get_loc_mid() {
    return m_loc_mid;
}

void Node::set_loc(Vector3d loc) {
    m_loc = loc;
}

Vector3d Node::get_vel() {
    return m_vel;
}

Vector3d Node::get_vel_mid() {
    return m_vel_mid;
}

void Node::set_vel(Vector3d vel) {
    m_vel = vel;
}

void Node::add_mass(double m) {
    m_mass += m;
}

double Node::get_mass() {
    return m_mass;
}

void Node::add_force(Eigen::Vector3d f) {
    m_force += f;
}

/**
 * @brief Node::update - update the node locations
 * @param seconds - time
 * @param mid - boolean on whether or not we are calculating the midpoint locations for the midpoint method
 */
void Node::update(double seconds, bool mid) {
    Vector3d a = m_force / m_mass;
    if (mid) {
        // Update velocity and location
        m_vel_mid = m_vel + a * (seconds / 2.0);
        m_loc_mid = m_loc + m_vel_mid * (seconds / 2.0);
    } else {
        // Update velocity and location
        m_vel = m_vel_mid + a * seconds;
        m_loc = m_loc_mid + m_vel * seconds;
    }

    m_force = _gravity;
}

