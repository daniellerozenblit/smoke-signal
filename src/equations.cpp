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



void Simulation::initGrid()
{
    std::vector<std::shared_ptr<VoxelFace>> Xfaces1d(gridSize+1);
    std::vector<std::vector<std::shared_ptr<VoxelFace>>> Xfaces2d(gridSize+1);
    std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>> Xfaces3d(gridSize+1);
    std::vector<std::shared_ptr<VoxelFace>> Yfaces1d(gridSize+1);
    std::vector<std::vector<std::shared_ptr<VoxelFace>>> Yfaces2d(gridSize+1);
    std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>> Yfaces3d(gridSize+1);
    std::vector<std::shared_ptr<VoxelFace>> Zfaces1d(gridSize+1);
    std::vector<std::vector<std::shared_ptr<VoxelFace>>> Zfaces2d(gridSize+1);
    std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>> Zfaces3d(gridSize+1);

    // triple for loop of nxnxn... push back vertices
    for (int i = 0; i < gridSize+1; i++)
    {
        for (int j=0; j<gridSize+1; j++)
        {
            for(int k=0; k<gridSize+1; k++)
            {
                //create new faces at all the indices... all are halfIndex ++
                Xfaces1d[k] = std::make_shared<VoxelFace>();
                Yfaces1d[k] = std::make_shared<VoxelFace>();
                Zfaces1d[k] = std::make_shared<VoxelFace>();
            }
            Xfaces2d[j] = Xfaces1d;
            Yfaces2d[j] = Yfaces1d;
            Zfaces2d[j] = Zfaces1d;
        }
        Xfaces3d[i] = Xfaces2d;
        Yfaces3d[i] = Yfaces2d;
        Zfaces3d[i] = Zfaces2d;
    }

    std::vector<std::shared_ptr<Voxel>> voxel1d(gridSize);
    std::vector<std::vector<std::shared_ptr<Voxel>>> voxel2d(gridSize);
    std::vector<std::vector<std::vector<std::shared_ptr<Voxel>>>> voxel3d(gridSize);
    std::vector<std::shared_ptr<VoxelFace>> voxelFace(6);


    // triple for loop of nxnxn... push back vertices
    for (int i = 0; i < gridSize; i++)
    {
        for (int j=0; j<gridSize; j++)
        {
            for(int k=0; k<gridSize; k++)
            {
                //create new faces at all the indices... all are halfIndex ++
                voxel1d[k] = std::make_shared<Voxel>();
                //fill voxelFace vector with appropriate faces
                //left and right
                voxelFace[0] = Xfaces3d[i][j][k];
                voxelFace[1] = Xfaces3d[i+1][j][k];
                //front and back
                voxelFace[2] = Yfaces3d[i][j][k];
                voxelFace[3] = Yfaces3d[i][j+1][k];
                //top and bottom
                voxelFace[4] = Zfaces3d[i][j][k];
                voxelFace[5] = Zfaces3d[i][j][k+1];

                voxel1d[i]->faces = voxelFace;
                voxel1d[i]->density = (i+j+k)/(1.0f*gridSize*3);
            }
            voxel2d[j] = voxel1d;
        }
        voxel3d[i] = voxel2d;
    }
    grid = voxel3d;
    std::vector<float> d1d(gridSize);
    std::vector<std::vector<float>> d2d(gridSize);
    std::vector<std::vector<std::vector<float>>> densities(gridSize);

    for (int i = 0; i < gridSize; i++)
    {
        for (int j=0; j<gridSize; j++)
        {
            for(int k=0; k<gridSize; k++)
            {
                d1d[k] = grid[i][j][k]->density;
            }
            d2d[j] = d1d;
        }
        densities[i] = d2d;
    }
    /// output a vector of vector of vector of float  create voxel shit and export
    /// include rendering header
    Rendering::write_vol("C:\\Users\\annaf\\course\\cs2240\\final\\smoke-signal\\src\\rendering\\densities.vol", densities);
}









