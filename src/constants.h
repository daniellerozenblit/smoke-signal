#ifndef CONSTANTS_H
#define CONSTANTS_H

// Visualizer
int MAXDENSITYSPHERES = 30;

// Grid
const int gridSize = 16;
const int voxelSize = 1;

constexpr int INDEX(int i, int j, int k) {
    return i + gridSize * j + gridSize * gridSize * k;
}

// Simulation
const double timestep = 0.02;
const double epsilon = 1e-4;

const double alpha = 9.8;
const double beta = 15.0;
const double Tambient = 25.0; //degrees farenheit

enum INTERP_TYPE
{
    VELOCITY,
    DENSITY,
    TEMPERATURE
};


#endif // CONSTANTS_H
