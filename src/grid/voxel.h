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

<<<<<<< HEAD
    double density;
    double nextDensity;
=======
    float density;
    float nextDensity;
>>>>>>> 2d2187edd999fa4704660890c90cef0a344ba3e1

    double temp;
    double nextTemp;

    Vector3d centerVel;
    Vector3d nextCenterVel;

    Vector3d centerVort;
    Vector3d nextCenterVort;

    Vector3d center;
    Vector3d nextCenter;

    Vector3d force;
<<<<<<< HEAD
    Vector3d nextForce;
=======
>>>>>>> 2d2187edd999fa4704660890c90cef0a344ba3e1

    double volume;
};
#endif // VOXEL_H
