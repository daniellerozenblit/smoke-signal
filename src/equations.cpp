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
// update the velocity components of the fluid---
/// updateVelocities
    // add force fields to velocity grid (including user fields, buoyancy, confinement)
    /// incorporate/process user fields
    /// incorporate buoyancy (eqn 8)
    /// confinement (eqn 11)
    // ^^ multiply each force by time step and add to velocity (APPENDIX A)
// solve for advection term (in eqn 3)
/// updateAdvection (semi Langrangian scheme for advection in eqn 3)
/// builds new grid from precomputed
/// trace midpoints of each face through field
/// new vels interpolated--> transferred to face cells of origin
/// ** boundary (clip to furthest boundary point fig 2)
// cubic interpolator
/// see appendix
// mass conservation
/// conserve mass
/// poisson eqn for pressure (eqn 4) --> sparse linear system
/// ** free neumann boundary conditions at boundary (normal dp = 0)
// solve
/// conjugate gradient method, incomplete Choleski preconditioner
// swap grids
/// REPEAT 20 ITERATIONS

// advect temp and density (semi-Lagrangian sheme with voxel centers, interpolate as in velocity)

// output to render file?
// update wireframe?

/// ALL EQUATION HEADERS NEEDED FOR SIMULATION.CPP
/*

void update();

void init();
void updateVelocities();
void defForces();
void defVelocities();
void advect();
void createSparsePressure();
void solveSparsePressure();
void advectVelocity();
void advectPressure();

void cubicInterpolator();

// stuff for solver

std::vector<T> tripletList;
Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> ICCG;

Eigen::SparseMatrix<double, Eigen::RowMajor> A;
Eigen::VectorXd b;
Eigen::VectorXd x;
*/

/*\

init()
{
    defVelocities();
    ---
    for each grid cell
    {
        set temp to room temp
    }
}




*/




