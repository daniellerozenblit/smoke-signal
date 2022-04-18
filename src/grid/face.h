#ifndef FACE_H
#define FACE_H

#include <Eigen/StdVector>
#include <memory>

using namespace Eigen;

struct Face
{
    Vector3f index;

    Vector3d vel;
    Vector3d nextVel;

    Vector3d vort;
    Vector3d nextVort;

};

#endif // FACE_H
