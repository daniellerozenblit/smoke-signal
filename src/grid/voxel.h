#ifndef VOXEL_H
#define VOXEL_H

#include <Eigen/StdVector>
#include <memory>

#include "face.h"

using namespace Eigen;

struct Voxel
{
    std::vector<std::shared_ptr<Face>> faces;
    Vector3i index;

    Vector3d density;
    Vector3d nextDensity;

    Vector3d temp;
    Vector3d nextTemp;

    Vector3d centerVel;
    Vector3d nextCenterVel;

    Vector3d centerVort;
    Vector3d nextCenterVort;

    double volume;
};
#endif // VOXEL_H
