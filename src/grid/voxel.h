#ifndef VOXEL_H
#define VOXEL_H

#include <Eigen/StdVector>
#include <memory>

#include "face.h"

struct Voxel
{
    std::vector<std::shared_ptr<Face>> faces;
    Eigen::Vector3i index;

    Eigen::Vector3d density;
    Eigen::Vector3d nextDensity;

    Eigen::Vector3d temp;
    Eigen::Vector3d nextTemp;

    Eigen::Vector3d centerVel;
    Eigen::Vector3d nextCenterVel;

    Eigen::Vector3d centerVort;
    Eigen::Vector3d nextCenterVort;

    double volume;
};
#endif // VOXEL_H
