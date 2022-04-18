#ifndef COLLIDER_H
#define COLLIDER_H

#include <Eigen/Dense>
#include "../fem/node.h"
#include <memory>

using namespace Eigen;

class Collider
{
public:
    Collider();
    virtual void collision(std::shared_ptr<Node> n, Matrix4d m_model) {};
protected:
    Vector3d to_world(Vector3d v, Matrix4d model);
    Vector3d to_object(Vector3d v, Matrix4d model);
};

#endif // COLLIDER_H
