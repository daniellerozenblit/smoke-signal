#ifndef CONSTANTS_H
#define CONSTANTS_H

// Visualizer
const int MAXDENSITYSPHERES = 15;

// Grid

const int gridSize = 5;
const int cubeSize = gridSize * gridSize * gridSize;
const double voxelSize = 1.0 / gridSize;
const double emitSeconds = 0.6;

constexpr int INDEX(int i, int j, int k) {
    return i + gridSize * j + gridSize * gridSize * k;
}

// Simulation
const double timestep = .05;
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
