#include "collider.h"

Collider::Collider()
{

}

Vector3d Collider::to_world(Vector3d v, Matrix4d model) {
    Vector4d v_4 = Vector4d(v[0], v[1], v[2], 1.0);
    Vector4d world = model * v_4;

    return Vector3d(world[0], world[1], world[2]);
}

Vector3d Collider::to_object(Vector3d v, Matrix4d model) {
    Vector4d v_4 = Vector4d(v[0], v[1], v[2], 1.0);
    Vector4d object = model.inverse() * v_4;

    return Vector3d(object[0], object[1], object[2]);
}
