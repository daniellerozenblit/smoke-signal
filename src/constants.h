#ifndef CONSTANTS_H
#define CONSTANTS_H


///NEW

const int MILLI_SECS_PER_FRAME = 10;
const double TIMESTEP =0.085;
const double VOXEL_SIZE = 0.5;
const double FLUID_DENSE = 2;
const double AIR_DENSE = 1;
const double BOUND_CONSTANT = (fluidDensity * theCellSize) / dt;
const double ALPHA = 0.08; // Gravity's effect on the smoke particles.
const double BETA = 0.97; // Buoyancy's effect due to temperature difference.
const double T_AMBIENT = 0.0; // Ambient temperature.
const double EPSILON = 0.1;






//// Visualizer
const int MAXDENSITYSPHERES = 15;

//// Grid
const int SIZE_X = 16;
const int SIZE_Y = 16;
const int SIZE_Z = 16;
const int SIZE_CUBE = SIZE_X * SIZE_Y * SIZE_Z;
//const double voxelSize = 0.5;
//const double emitSeconds = 2.0;

//constexpr int INDEX(int i, int j, int k) {
//    return i + SIZE_X * j + SIZE_X * SIZE_Y * k;
//}

//// Simulation

//const double timestep = 0.05;
//const double VORT_EPSILON = 1.0e-2;
//const double epsilon = 1.0e-7;

//const double alpha = 0.08;
//const double beta = 0.97;
//const double Tambient = 50.0; //degrees farenheit

//const bool ADVECT_TEMP = false;
//const bool ADVECT_DENSITY = true;

//enum INTERP_TYPE
//{
//    VELOCITY_X,
//    VELOCITY_Y,
//    VELOCITY_Z,
//    DENSITY,
//    TEMPERATURE
//};


#endif // CONSTANTS_H
