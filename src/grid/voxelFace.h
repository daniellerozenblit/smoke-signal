#ifndef VOXELFACE_H
#define VOXELFACE_H

#include <Eigen/StdVector>
#include <memory>

using namespace Eigen;

struct VoxelFace
{
    Vector3f index;

    Vector3d vel;
    Vector3d nextVel;

    Vector3d vort;
    Vector3d nextVort;

};

#endif // VOXELFACE_H
