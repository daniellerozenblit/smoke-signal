#include "plane.h"

Plane::Plane(Vector3d point, Vector3d normal) :
    m_point(point),
    m_normal(normal) {
}

void Plane::collision(std::shared_ptr<Node> n, Matrix4d model) {
    Vector3d x = to_world(n->get_loc(), model);

    if ((x - m_point).dot(m_normal) < 0.0) {
        // Project vertex out of the collider
        while ((x - m_point).dot(m_normal) < 0.0) {
            x = x + m_normal * 0.001;
        }
        n->set_loc(to_object(x, model));

        // Decompose velocity into a normal component and tangential component
        Vector3d v = n->get_vel();
        Vector3d normal_component = m_normal.dot(v) * v;
        Vector3d tangential_component = v - normal_component;

        // Scale the normal and tangential components
        n->set_vel(tangential_component * 1e-1 + normal_component * (1 - 1e-2));
    }
}

void Plane::set_normal(Vector3d normal) {
    m_normal = normal;
}
