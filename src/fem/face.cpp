#include "face.h"

Face::Face(std::shared_ptr<Node> n_1, std::shared_ptr<Node> n_2, std::shared_ptr<Node> n_3) :
    n_1(n_1),
    n_2(n_2),
    n_3(n_3)
{
    // Calculate face area
    Vector3d a = n_2->get_loc() - n_1->get_loc();
    Vector3d b = n_3->get_loc() - n_1->get_loc();

    m_area = 0.5 * (a.cross(b)).norm();

    // Calculate face normal
    m_normal = a.cross(b);
}

std::vector<std::shared_ptr<Node>> Face::get_vertices() {
    return std::vector<std::shared_ptr<Node>>({n_1, n_2, n_3});
}

std::shared_ptr<Node> Face::get_n_1() {
    return n_1;
}

std::shared_ptr<Node> Face::get_n_2() {
    return n_2;
}

std::shared_ptr<Node> Face::get_n_3() {
    return n_3;
}

double Face::get_area() {
    return m_area;
}

Vector3d Face::get_normal() {
    return m_normal;
}

// Comparator for faces
bool Face::eq(std::shared_ptr<Face> rhs) {
    std::vector<std::shared_ptr<Node>> l_v = get_vertices();
    std::vector<std::shared_ptr<Node>> r_v = rhs->get_vertices();
    std::set<std::shared_ptr<Node>> nodes;

    for (std::shared_ptr<Node> n : l_v) {
        nodes.insert(n);
    }

    for (std::shared_ptr<Node> n : r_v) {
        nodes.insert(n);
    }

    if (nodes.size() == 3) {
        return true;
    } else {
        return false;
    }
}
