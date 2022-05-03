#include "simulation.h"

#include <iostream>
#include "graphics/MeshLoader.h"
#include "constants.h"

#include "grid/voxelFace.h"
#include "grid/voxel.h"
#include "rendering/rendering.h"

using namespace Eigen;

///// WILL CONTAIN ALL EQUATIONS NECESSARY FOR THE SMOKE SIMULATION

//// FINITE VOLUME SPATIAL DISCRETIZATION-- numerically solve flud flow

//// create voxels (do this at outset)

//// define temperature, smoke density, external forces (center)
//// define velocity at voxel faces

///*
//if boundaries (other objects immersed in volume)

//flag every single voxel which intersects with the object as being occupied,
//set velocity to be same as immersed object, set temperature same as well

//DENSITY ZERO in intersecting voxel... boundary voxel desity = closest unoccupied voxel

//*/

//// smoke emission! pass in a list of indices of emitting voxels and it will set their density to 1 and upward velocity to 50
void Simulation::emitSmoke(std::vector<Eigen::Vector3i> indices) {
    for (auto voxel_index : indices) {
        grid->grid[voxel_index[0]][voxel_index[1]][voxel_index[2]]->density = 1.0;
        grid->grid[voxel_index[0]][voxel_index[1]][voxel_index[2]]->faces[4]->vel = 50.0;
    }
}

//// EACH TIME STEP
///// updateVelocities
//    // add force fields to velocity grid (user fields, buoyancy, confinement)
//    // ^^ multiply each force by time step and add to velocity (APPENDIX A)

void Simulation::updateVelocities() {
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for(int k = 0; k < gridSize; k++) {
                // buoyancy constants
                double alpha = 9.8;
                double beta = 15.0;
                double ambient_temp = 50.0;

                // add vertical buoyancy force to z axis where z = 1 is up (eqn. 8)
                grid->grid[i][j][k]->force = Vector3d();
                grid->grid[i][j][k]->force[0] = 0;
                grid->grid[i][j][k]->force[1] = 0;
                grid->grid[i][j][k]->force[2] = -1.0 * alpha * grid->grid[i][j][k]->density + beta * (grid->grid[i][j][k]->temp - ambient_temp);

                // user defined force fields


                // vorticity confinement force (eqn. 11)

            }
        }
    }
}

void Simulation::computeCellCenteredVel() {
    // Find the cell-centered velocities in each direction
    double avg_u;
    double avg_v;
    double avg_w;

    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for(int k = 0; k < gridSize; k++) {
                avg_u = (grid->grid[i][j][k]->faces[0]->vel + grid->grid[i][j][k]->faces[1]->vel) / 2.0;
                avg_v = (grid->grid[i][j][k]->faces[2]->vel + grid->grid[i][j][k]->faces[3]->vel) / 2.0;
                avg_w = (grid->grid[i][j][k]->faces[4]->vel + grid->grid[i][j][k]->faces[5]->vel) / 2.0;

                grid->grid[i][j][k]->centerVel = Vector3d(avg_u, avg_v, avg_w);
            }
        }
    }
}

void Simulation::confinementForce() {
    // Find the cell-centered velocities in each direction
    computeCellCenteredVel();

    // Calculate the vorticities for each cell
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for(int k = 0; k < gridSize; k++) {
                // Border Cases
                if (i == 0 || j == 0 || k == 0 || i == gridSize - 1 || j == gridSize - 1 || k == gridSize - 1) {
                    grid->grid[i][j][k]->vort = Eigen::Vector3d(0.0, 0.0, 0.0);
                    continue;
                }

                float vort_x = (grid->grid[i][j + 1][k]->centerVel[2] - grid->grid[i][j - 1][k]->centerVel[2]
                        - grid->grid[i][j][k + 1]->centerVel[1] + grid->grid[i][j][k - 1]->centerVel[1]) * 0.5 / voxelSize;

                float vort_y = (grid->grid[i][j][k + 1]->centerVel[0] - grid->grid[i][j][k - 1]->centerVel[0]
                        - grid->grid[i + 1][j][k]->centerVel[2] + grid->grid[i - 1][j][k]->centerVel[2]) * 0.5 / voxelSize;

                float vort_z = (grid->grid[i + 1][j][k]->centerVel[1] - grid->grid[i - 1][j][k]->centerVel[1]
                        - grid->grid[i][j + 1][k]->centerVel[0] + grid->grid[i][j - 1][k]->centerVel[0]) * 0.5 / voxelSize;

                grid->grid[i][j][k]->vort = Eigen::Vector3d(vort_x, vort_y, vort_z);
            }
        }
    }

    std::vector<Eigen::Vector3d> confinement(gridSize * gridSize * gridSize);
    // Calculate the confinement force for each cell
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for(int k = 0; k < gridSize; k++) {
                // Border Cases
                if (i == 0 || j == 0 || k == 0 || i == gridSize - 1 || j == gridSize - 1 || k == gridSize - 1) {
                    continue;
                }

                // Gradient of vorticity
                double g_x = (grid->grid[i + 1][j][k]->vort.norm() - grid->grid[i - 1][j][k]->vort.norm()) * 0.5 / voxelSize;
                double g_y = (grid->grid[i][j + 1][k]->vort.norm() - grid->grid[i][j - 1][k]->vort.norm()) * 0.5 / voxelSize;
                double g_z = (grid->grid[i][j][k + 1]->vort.norm() - grid->grid[i][j][k - 1]->vort.norm()) * 0.5 / voxelSize;

                // Normalized vorticity location vector
                Eigen::Vector3d N = Eigen::Vector3d(g_x, g_y, g_z).normalized();

                // Calculate the confinement force
                // TODO: add confinement force to voxel forces
                confinement[INDEX(i + 1, j, k)] = epsilon * voxelSize * grid->grid[i][j][k]->vort.cross(N);
            }
        }
    }
}


//// solve for advection term (in eqn 3)
///// updateAdvection (semi Langrangian scheme for advection in eqn 3)
///// builds new grid from precomputed
///// trace midpoints of each face through field
///// new vels interpolated--> transferred to face cells of origin
///// ** boundary (clip to furthest boundary point fig 2)
void Simulation::advectVelocity()
{
    for (int i = 0; i < gridSize + 1; i++) {
        for (int j = 0; j < gridSize+1; j++) {
            for (int k = 0; k<gridSize+1; k++) {
                Vector3d newVel;
                //iterate 3 times for x, y, and z faces
                for(int c=0; c<3; c++)
                {
                    Eigen::Vector3d pos;
                    Eigen::Vector3d vel;
                    Eigen::Vector3d dpos;

                    switch(c) {
                    case 0: //x
                        pos = Vector3d((i-0.5),j,k) * voxelSize;
                        vel = grid->grid[i][j][k]->centerVel;
                        break;
                    case 1: //y
                        pos = Vector3d(i,(j-0.5),k) * voxelSize;
                        vel = grid->grid[i][j][k]->centerVel;
                        break;
                    case 2: //z
                        pos = Vector3d(i,j,(k-0.5)) * voxelSize;
                        vel = grid->grid[i][j][k]->centerVel;

                        break;
                    }
                    pos -= timestep*vel;
                    newVel[c] = cubicInterpolator(pos, INTERP_TYPE::VELOCITY, c);
                    grid->faces[c][i][j][k]->vel = newVel[c];
                }
            }
        }
    }

}

// cubic interpolator
double Simulation::cubicInterpolator(Vector3d position, INTERP_TYPE var, int axis)
{
    // first set up the f to be interpolated... get coords and clamp within grid bounds
    Vector3d posClamped = Vector3d(clamp(position[0]), clamp(position[1]), clamp(position[2]));
    Vector3i indexCast;
    Vector3d percentage;
    for (int c = 0; c<3 ; c++)
    {
        indexCast[c] = (int)(posClamped[c]/voxelSize);
        percentage[c] = posClamped[c]/voxelSize - indexCast[c];
    }
    //collapse on each direction
    //compute indices for the collapse
    Vector4i collapseX = Vector4i{indexCast[0]-1, indexCast[0], indexCast[0]+1, indexCast[0]+2};
    Vector4i collapseY = Vector4i{indexCast[1]-1, indexCast[1], indexCast[1]+1, indexCast[1]+2};
    Vector4i collapseZ = Vector4i{indexCast[2]-1, indexCast[2], indexCast[2]+1, indexCast[2]+2};

    //nested collapse on each axis using the coordinates...
    //collapse z
    //collapse y
    //collapse x
    Vector4d Xcollapse;
    for(int i = 0; i < 4; i++)
    {
        Vector4d Ycollapse;

        for(int j = 0; j < 4; j++)
        {
            Vector4d Zcollapse;
            for(int k = 0; k < 4; k++)
            {
                switch (var)
                {
                    case INTERP_TYPE::DENSITY:
                        Zcollapse[k] = grid->grid[collapseX[i]][collapseY[j]][collapseZ[k]]->density;
                        break;
                    case INTERP_TYPE::TEMPERATURE:
                        Zcollapse[k] = grid->grid[collapseX[i]][collapseY[j]][collapseZ[k]]->temp;
                        break;
                    case INTERP_TYPE::VELOCITY:
                        Zcollapse[k] = grid->grid[collapseX[i]][collapseY[j]][collapseZ[k]]->centerVel[axis];
                        break;
                }
            }
            Ycollapse[j] = collapseAxis(Zcollapse, percentage[2]);
        }
        Xcollapse[i] = collapseAxis(Ycollapse, percentage[1]);
    }
    return collapseAxis(Xcollapse, percentage[0]);
}

double Simulation::collapseAxis(Vector4d input, double percentage)
{
    /// a3 = dk + d_(k+1) - deltak;
    /// a2 = 3*deltak - 2*dk - d_(k+1)
    /// a1 = dk;
    /// a0 = fk;
    /// dk = (f_(k+1) - f_(k-1))/2;
    /// deltak = f_(k+1) - f_k;
    /// f(t) = a3(t - tk)^3 + a2(t - tk)^2 + a1(t - tk) + a0;

    double deltak = input[2]-input[1];
    double dk = (input[2] - input[0])/2.0;
    double dk1 = (input[3] - input[1])/2.0;

    double a0 = input[1];
    double a1 = dk;
    double a2 = 3*deltak - 2*dk - dk1;
    double a3 = dk + dk1 - deltak;

    double collapse = a3*pow(percentage, 3) + a2*pow(percentage, 2) + a1*(percentage) + a0;
    return collapse;
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


double Simulation::clamp(double input)
{
    return (std::min(std::max(0.0, input), 1.0*gridSize*voxelSize));
}









