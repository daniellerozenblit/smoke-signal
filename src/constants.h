#ifndef CONSTANTS_H
#define CONSTANTS_H

// Visualizer
const int MAXDENSITYSPHERES = 15;

// Grid
const int gridSize = 10;
const int cubeSize = gridSize * gridSize * gridSize;
const double voxelSize = 1.0 / gridSize;
const float PI = 3.1415;
const double emitSeconds = 1.0;

constexpr int INDEX(int i, int j, int k) {
    return i + gridSize * j + gridSize * gridSize * k;
}

// Simulation
const double timestep = .085;
const double epsilon = 1e-12;

const double alpha = 9.8;
const double beta = 15.0;
const double Tambient = 50.0; //degrees farenheit

enum INTERP_TYPE
{
    VELOCITY,
    DENSITY,
    TEMPERATURE
};


#endif // CONSTANTS_H
