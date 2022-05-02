#include "grid.h"
#include "constants.h"
#include "rendering/rendering.h"

void Grid::init()
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
    std::vector<std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>>> faces3d(3);
    faces3d[0] = Xfaces3d;
    faces3d[1] = Yfaces3d;
    faces3d[2] = Zfaces3d;
    faces = faces3d;

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

                voxel1d[k]->faces = voxelFace;
                voxel1d[k]->center = Vector3d(i,j,k)*voxelSize;
                voxel1d[k]->density = (i+j+k)/(1.0f*gridSize*3);
            }
            voxel2d[j] = voxel1d;
        }
        voxel3d[i] = voxel2d;
    }
    grid = voxel3d;
}

void Grid::render()
{
    std::vector<float> d1d(gridSize);
    std::vector<std::vector<float>> d2d(gridSize);
    std::vector<std::vector<std::vector<float>>> densities(gridSize);

    for (int i = 0; i < gridSize; i++) {
        for (int j=0; j < gridSize; j++) {
            for(int k=0; k < gridSize; k++) {
                d1d[k] = grid[i][j][k]->density;
            }
            d2d[j] = d1d;
        }
        densities[i] = d2d;
    }
    /// output a vector of vector of vector of float create voxel shit and export
    /// include rendering header
    Rendering::write_vol("C:\\Users\\annaf\\course\\cs2240\\final\\smoke-signal\\src\\rendering\\densities.vol", densities);
}
