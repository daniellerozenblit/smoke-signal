#include "simulation.h"

#include <iostream>
#include "graphics/MeshLoader.h"
using namespace Eigen;


/// WILL CONTAIN ALL EQUATIONS NECESSARY FOR THE SMOKE SIMULATION

// FINITE VOLUME SPATIAL DISCRETIZATION-- numerically solve flud flow

// create voxels (do this at outset)

// define temperature, smoke density, external forces (center)
// define velocity at voxel faces

/*
if boundaries (other objects immersed in volume)

flag every single voxel which intersects with the object as being occupied,
set velocity to be same as immersed object, set temperature same as well

DENSITY ZERO in intersecting voxel... boundary voxel desity = closest unoccupied voxel

*/

//simulation advances by updating one grid from the other over dt
//TWO GRID INSTANCES
//


// EACH TIME STEP
//
