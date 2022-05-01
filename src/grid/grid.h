#ifndef GRID_H
#define GRID_H

#include "unordered_map"
#include <memory>
#include "voxel.h"
#include "Eigen/Sparse"
#include "voxelFace.h"


class Grid
{
    public:
    std::vector<std::vector<std::vector<std::shared_ptr<Voxel>>>> grid;

    void initGrid();

};

#endif // GRID_H
