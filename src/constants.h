#ifndef CONSTANTS_H
#define CONSTANTS_H

const int gridSize = 16;
const int voxelSize = 1;
const double timestep = 0.02;

constexpr int INDEX(int i, int j, int k) {
    return i + gridSize * j + gridSize * gridSize * k;
}


#endif // CONSTANTS_H
