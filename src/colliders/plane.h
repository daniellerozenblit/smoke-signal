#ifndef PLANE_H
#define PLANE_H

#include "collider.h"

class Plane : public Collider {
    public:
        Plane(Vector3d point, Vector3d normal);
        void collision(std::shared_ptr<Node>, Matrix4d m_model);
        void set_normal(Vector3d normal);
    private:
        Vector3d m_point;
        Vector3d m_normal;
};

#endif // PLANE_H
