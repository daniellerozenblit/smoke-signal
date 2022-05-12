#ifndef VOXEL_H
#define VOXEL_H

#include <Eigen/StdVector>
#include <memory>

#include "voxelFace.h"

using namespace Eigen;

struct Voxel {
    std::vector<std::shared_ptr<VoxelFace>> faces;
    Vector3i index;

    double density;
    double nextDensity;

    double pressure;
    double nextPressure;

    double temp;
    double nextTemp;

    Eigen:Vector3d centerVel;
    Vector3d nextCenterVel;

    Vector3d vort;
//    Vector3d nextVort;

    Vector3d center;
    Vector3d nextCenter;

    Vector3d force;
//    Vector3d nextForce;

    double volume;
};
#endif // VOXEL_H
