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

// smoke emission! pass in a list of indices of emitting voxels and it will set their density to 1 and upward velocity to 50
void Simulation::emitSmoke(std::vector<Eigen::Vector3i> indices) {
    for (auto voxel_index : indices) {
        grid[voxel_index[0]][voxel_index[1]][voxel_index[2]]->density = 1.0;
        grid[voxel_index[0]][voxel_index[1]][voxel_index[2]]->faces[4]->vel = 50.0;
    }
}

// EACH TIME STEP
/// updateVelocities
    // add force fields to velocity grid (user fields, buoyancy, confinement)
    // ^^ multiply each force by time step and add to velocity (APPENDIX A)

void Simulation::updateVelocities() {
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for(int k = 0; k < gridSize; k++) {
                // buoyancy constants
                double alpha = 9.8;
                double beta = 15.0;
                double ambient_temp = 50.0;

                // add vertical buoyancy force to z axis where z = 1 is up (eqn. 8)
                grid[i][j][k]->force = Vector3d();
                grid[i][j][k]->force[0] = 0;
                grid[i][j][k]->force[1] = 0;
                grid[i][j][k]->force[2] = -1.0 * alpha * grid[i][j][k]->density + beta * (grid[i][j][k]->temp - ambient_temp);

                // user defined force fields


                // vorticity confinement force (eqn. 11)

            }
        }
    }
}

void Simulation::confinementForce() {
    // Find the cell-centered velocities in each direction
    std::vector<float> avg_u(gridSize * gridSize * gridSize);
    std::vector<float> avg_v(gridSize * gridSize * gridSize);
    std::vector<float> avg_w(gridSize * gridSize * gridSize);

    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for(int k = 0; k < gridSize; k++) {
                avg_u[INDEX(i, j, k)] = (grid[i][j][k]->faces[0]->vel + grid[i][j][k]->faces[1]->vel) / 2.0f;
                avg_v[INDEX(i, j, k)] = (grid[i][j][k]->faces[2]->vel + grid[i][j][k]->faces[3]->vel) / 2.0f;
                avg_w[INDEX(i, j, k)] = (grid[i][j][k]->faces[4]->vel + grid[i][j][k]->faces[5]->vel) / 2.0f;
            }
        }
    }

    // Calculate the vorticities for each cell
    std::vector<Eigen::Vector3f> vorticity(gridSize * gridSize * gridSize);

    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for(int k = 0; k < gridSize; k++) {
                // Border Cases
                if (i == 0 || j == 0 || k == 0 || i == gridSize - 1 || j == gridSize - 1 || k == gridSize - 1) {
                    vorticity[INDEX(i, j, k)] = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
                    continue;
                }

                float vort_x = 0.5f * avg_w[INDEX(i, j + 1, k)] - avg_w[INDEX(i, j - 1, k)]
                        - avg_v[INDEX(i, j, k + 1)] + avg_v[INDEX(i, j, k - 1)] / voxelSize;

                float vort_y = 0.5f * avg_u[INDEX(i, j, k + 1)] - avg_u[INDEX(i, j, k - 1)]
                        - avg_w[INDEX(i + 1, j, k)] + avg_w[INDEX(i - 1, j, k)] / voxelSize;

                float vort_z = 0.5f * avg_v[INDEX(i + 1, j, k)] - avg_v[INDEX(i - 1, j, k)]
                        - avg_u[INDEX(i, j + 1, k)] + avg_u[INDEX(i, j - 1, k)] / voxelSize;

                vorticity[INDEX(i, j, k)] = Eigen::Vector3f(vort_x, vort_y, vort_z);
            }
        }
    }

    // Calculate the confinement force for each cell
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for(int k = 0; k < gridSize; k++) {
                // Border Cases
                if (i == 0 || j == 0 || k == 0 || i == gridSize - 1 || j == gridSize - 1 || k == gridSize - 1) {
                    continue;
                }
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
        for (int j=0; j < gridSize; j++)
        {
            for(int k=0; k < gridSize; k++)
            {
                //iterate 3 times for x, y, and z faces
                for(int c=0; c<3; c++)
                {
                    Eigen::Vector3d pos;
                    Eigen::Vector3d vel;
                    Eigen::Vector3d dpos;

                    switch(c) {
                    case 0: //x
                        pos = Vector3d(i-(0.5 * voxelSize),j,k);


                        break;
                    case 1: //y
                        pos = Vector3d(i,j-(0.5 * voxelSize),k);

                        break;
                    case 2: //z
                        pos = Vector3d(i,j,k-(0.5 * voxelSize));

                        break;
                    }
//                    vel = faces[c][i][j][k]->vel;
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
    // double deltak = ;
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












