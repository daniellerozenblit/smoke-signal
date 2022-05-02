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
};

#endif // VOXELFACE_H
