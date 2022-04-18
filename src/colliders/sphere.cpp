#include "sphere.h"

Sphere::Sphere(Vector3d point, double radius) :
    m_point(point),
    m_radius(radius) {
}

void Sphere::collision(std::shared_ptr<Node> n, Matrix4d model) {
    Vector3d x = to_world(n->get_loc(), model);

    if ((x - m_point).norm() < m_radius) {
        // Project vertex out of the collider
        while ((x - m_point).norm() < m_radius) {
            x = x + (x - m_point) * 0.001;
        }
        n->set_loc(to_object(x, model));

        // Decompose velocity into a normal component and tangential component
        Vector3d v = n->get_vel();
        Vector3d normal_component = (x - m_point).dot(v) * v;
        Vector3d tangential_component = v - normal_component;

        // Scale the normal and tangential components
        n->set_vel(tangential_component * 1e-1 + normal_component * (1 - 1e-2));
    }
}
