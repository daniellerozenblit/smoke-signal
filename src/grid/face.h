#ifndef FACE_H
#define FACE_H

#include <Eigen/StdVector>
#include <memory>

struct Face
{
    Eigen::Vector3f index;

    Eigen::Vector3d vel;
    Eigen::Vector3d nextVel;

    Eigen::Vector3d vort;
    Eigen::Vector3d nextVort;

};

#endif // FACE_H
