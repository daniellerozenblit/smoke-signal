#ifndef SPHERE_H
#define SPHERE_H

#include "collider.h"

class Sphere : public Collider {
    public:
        Sphere(Vector3d point, double radius);
        void collision(std::shared_ptr<Node>, Matrix4d m_model);
    private:
        Vector3d m_point;
        double m_radius;
};

#endif // SPHERE_H
