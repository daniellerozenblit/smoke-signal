#ifndef VOXELFACE_H
#define VOXELFACE_H

#include <Eigen/StdVector>
#include <memory>

using namespace Eigen;

struct VoxelFace
{
    Vector3f index;

    double vel;
    double nextVel;

    Vector3d velocity;
    Vector3d nextVelocity;
};

#endif // VOXELFACE_H
