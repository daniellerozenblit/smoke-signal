#include "grid.h"
#include "constants.h"
#include "rendering/rendering.h"

void Grid::init() {
    initFaces();
    std::vector<std::vector<std::vector<std::shared_ptr<Voxel>>>> voxel3d(gridSize);

    // Assign faces to corresponding voxels
    for (int i = 0; i < gridSize; i++) {
        std::vector<std::vector<std::shared_ptr<Voxel>>> voxel2d(gridSize);

        for (int j=0; j < gridSize; j++) {
            std::vector<std::shared_ptr<Voxel>> voxel1d(gridSize);

            for(int k=0; k < gridSize; k++) {
                // Create new voxel and fill with appropriate faces
                voxel1d[k] = std::make_shared<Voxel>();
                std::vector<std::shared_ptr<VoxelFace>> voxelFace(6);

                // Left and right
                voxelFace[0] = faces[0][i][j][k];
                voxelFace[1] = faces[0][i+1][j][k];
                // Front and back
                voxelFace[2] = faces[1][i][j][k];
                voxelFace[3] = faces[1][i][j+1][k];
                // Top and bottom
                voxelFace[4] = faces[2][i][j][k];
                voxelFace[5] = faces[2][i][j][k+1];

                voxel1d[k]->faces = voxelFace;
                voxel1d[k]->center = Vector3d(i,j,k) * voxelSize;
                voxel1d[k]->density = (i+j+k)/(1.0f * gridSize * 3);
            }
            voxel2d[j] = voxel1d;
        }
        voxel3d[i] = voxel2d;
    }
    grid = voxel3d;
}

void Grid::initFaces() {
    std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>> x_faces(gridSize + 1);
    std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>> y_faces(gridSize + 1);
    std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>> z_faces(gridSize + 1);

    // Create new faces for each voxel
    for (int i = 0; i < gridSize + 1; i++) {
        std::vector<std::vector<std::shared_ptr<VoxelFace>>> x_faces2d(gridSize + 1);
        std::vector<std::vector<std::shared_ptr<VoxelFace>>> y_faces2d(gridSize + 1);
        std::vector<std::vector<std::shared_ptr<VoxelFace>>> z_faces2d(gridSize + 1);

        for (int j=0; j < gridSize + 1; j++) {
            std::vector<std::shared_ptr<VoxelFace>> x_faces1d(gridSize + 1);
            std::vector<std::shared_ptr<VoxelFace>> y_faces1d(gridSize + 1);
            std::vector<std::shared_ptr<VoxelFace>> z_faces1d(gridSize + 1);

            for(int k = 0; k < gridSize + 1; k++) {
                // Create x,y,z facing faces
                x_faces1d[k] = std::make_shared<VoxelFace>();
                y_faces1d[k] = std::make_shared<VoxelFace>();
                z_faces1d[k] = std::make_shared<VoxelFace>();
            }

            x_faces2d[j] = x_faces1d;
            y_faces2d[j] = y_faces1d;
            z_faces2d[j] = z_faces1d;
        }

        x_faces[i] = x_faces2d;
        y_faces[i] = y_faces2d;
        z_faces[i] = z_faces2d;
    }

    std::vector<std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>>> faces3d(3);
    faces3d[0] = x_faces;
    faces3d[1] = y_faces;
    faces3d[2] = z_faces;
    faces = faces3d;
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
