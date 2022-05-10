#ifndef GRID_H
#define GRID_H

#include "unordered_map"
#include <memory>
#include "voxel.h"
#include "Eigen/Sparse"
#include "voxelFace.h"


class Grid {
    public:
    Grid();
    ~Grid();

    std::vector<std::vector<std::vector<std::shared_ptr<Voxel>>>> grid;
    std::vector<std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>>> faces;

    void initGrid();
    void init();
    void initFaces();
    void render(std::string number);

};

#endif // GRID_H
