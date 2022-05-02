#ifndef VOXEL_H
#define VOXEL_H

#include <Eigen/StdVector>
#include <memory>

#include "voxelFace.h"

using namespace Eigen;

struct Voxel
{
    std::vector<std::shared_ptr<VoxelFace>> faces;
    Vector3i index;

    float density;
    Vector3d nextDensity;

    Vector3d temp;
    Vector3d nextTemp;

    Vector3d centerVel;
    Vector3d nextCenterVel;

    Vector3d centerVort;
    Vector3d nextCenterVort;

    Vector3d center;
    Vector3d nextCenter;

    double volume;
};
#endif // VOXEL_H
