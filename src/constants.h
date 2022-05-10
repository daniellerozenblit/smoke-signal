#ifndef CONSTANTS_H
#define CONSTANTS_H

// Visualizer
const int MAXDENSITYSPHERES = 15;

// Grid

const int SIZE_X = 5;
const int SIZE_Y = 10;
const int SIZE_Z = 5;
const int SIZE_CUBE = SIZE_X * SIZE_Y * SIZE_Z;
const double voxelSize = 0.5;
const float PI = 3.1415;
const double emitSeconds = 2.0;

constexpr int INDEX(int i, int j, int k) {
    return i + SIZE_X * j + SIZE_X * SIZE_Y * k;
}

// Simulation

const double timestep = 0.05;
const double epsilon = 1.0e-3;

const double alpha = 9.8;
const double beta = 15.0;
const double Tambient = 50.0; //degrees farenheit

const bool ADVECT_TEMP = false;
const bool ADVECT_DENSITY = true;

enum INTERP_TYPE
{
    VELOCITY,
    DENSITY,
    TEMPERATURE
};


#endif // CONSTANTS_H
