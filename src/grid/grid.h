#ifndef GRID_H
#define GRID_H

#include "unordered_map"
#include <memory>
#include "voxel.h"

struct Grid
{
    std::vector<std::vector<std::vector<std::shared_ptr<Voxel>>>> grid;

};

#endif // GRID_H
