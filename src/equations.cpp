#include "simulation.h"

#include <iostream>
#include "graphics/MeshLoader.h"
#include "constants.h"

#include "grid/voxelFace.h"
#include "grid/voxel.h"
#include "rendering/rendering.h"

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

void Simulation::updateVelocities() {
    for (int i = 0; i < gridSize; i++)
    {
        for (int j = 0; j < gridSize; j++)
        {
            for(int k = 0; k < gridSize; k++)
            {
                // buoyancy constants
                double alpha = 9.8;
                double beta = 15.0;
                double ambient_temp = 50.0;

                // add vertical buoyancy force to z axis where z = 1 is up
                grid[i][j][k]->force = Vector3d();
                grid[i][j][k]->force[0] = 0;
                grid[i][j][k]->force[1] = 0;
                grid[i][j][k]->force[2] = -1.0 * alpha * grid[i][j][k]->density + beta * (grid[i][j][k]->temp - ambient_temp);

                // user defined force fields


                // vorticity confinement force

            }
        }
    }
}


// solve for advection term (in eqn 3)
/// updateAdvection (semi Langrangian scheme for advection in eqn 3)
/// builds new grid from precomputed
/// trace midpoints of each face through field
/// new vels interpolated--> transferred to face cells of origin
/// ** boundary (clip to furthest boundary point fig 2)
void Simulation::advectVelocity()
{
//    OPENMP_FOR_COLLAPSE
//            FOR_EACH_FACE_X
//            {
//                Vec3 pos_u = m_grids->getCenter(i, j, k) - 0.5 * Vec3(VOXEL_SIZE, 0, 0);
//                Vec3 vel_u = m_grids->getVelocity(pos_u);
//                pos_u -= DT * vel_u;
//                m_grids->u(i, j, k) = m_grids->getVelocityX(pos_u);
//            }

//            OPENMP_FOR_COLLAPSE
//            FOR_EACH_FACE_Y
//            {
//                Vec3 pos_v = m_grids->getCenter(i, j, k) - 0.5 * Vec3(0, VOXEL_SIZE, 0);
//                Vec3 vel_v = m_grids->getVelocity(pos_v);
//                pos_v -= DT * vel_v;
//                m_grids->v(i, j, k) = m_grids->getVelocityY(pos_v);
//            }

//            OPENMP_FOR_COLLAPSE
//            FOR_EACH_FACE_Z
//            {
//                Vec3 pos_w = m_grids->getCenter(i, j, k) - 0.5 * Vec3(0, 0, VOXEL_SIZE);
//                Vec3 vel_w = m_grids->getVelocity(pos_w);
//                pos_w -= DT * vel_w;
//                m_grids->w(i, j, k) = m_grids->getVelocityZ(pos_w);
    for (int i = 0; i < gridSize; i++)
    {
        for (int j=0; j<gridSize; j++)
        {
            for(int k=0; k<gridSize; k++)
            {
                //iterate 3 times for x, y, and z faces
                for(int c=0; c<3; c++)
                {
                    Eigen::Vector3d pos;
                    Eigen::Vector3d vel;
                    Eigen::Vector3d dpos;

                    switch(c) {
                    case 0: //x
                        pos = Vector3d(i-(0.5*voxelSize),j,k);


                        break;
                    case 1: //y
                        pos = Vector3d(i,j-(0.5*voxelSize),k);

                        break;
                    case 2: //z
                        pos = Vector3d(i,j,k-(0.5*voxelSize));

                        break;
                    }
                    vel = faces[c][i][j][k]->vel;
                    dpos = timestep*vel;
                }
                //Matrix3f position =
            }
        }
    }

}
// cubic interpolator
void cubicInterpolator()
{
    /// a3 = dk + d_(k+1) - deltak;
    /// a2 = 3*deltak - 2*dk - d_(k+1)
    /// a1 = dk;
    /// a0 = fk;
    /// dk = (f_(k+1) - f_(k-1))/2;
    /// deltak = f_(k+1) - f_k;

    //f(t) = a3(t - tk)^3 + a2(t - tk)^2 + a1(t - tk) + a0;
//    double deltak = ;
}

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












