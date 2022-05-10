#include "grid.h"
#include "constants.h"
#include "rendering/rendering.h"


Grid::Grid()
{
    init();
}

void Grid::init() {
    initFaces();

    std::vector<std::vector<std::vector<std::shared_ptr<Voxel>>>> voxel3d(SIZE_X);

    // Assign faces to corresponding voxels
    for (int i = 0; i < SIZE_X; i++) {
        std::vector<std::vector<std::shared_ptr<Voxel>>> voxel2d(SIZE_Y);

        for (int j = 0; j < SIZE_Y; j++) {
            std::vector<std::shared_ptr<Voxel>> voxel1d(SIZE_Z);

            for (int k = 0; k < SIZE_Z; k++) {
                // Create new voxel and fill with appropriate faces
                voxel1d[k] = std::make_shared<Voxel>();
                std::vector<std::shared_ptr<VoxelFace>> voxelFace(6);

                // Left and right
                voxelFace[0] = faces[0][i][j][k];
                voxelFace[1] = faces[0][i + 1][j][k];
                // Front and back
                voxelFace[2] = faces[1][i][j][k];
                voxelFace[3] = faces[1][i][j + 1][k];
                // Top and bottom
                voxelFace[4] = faces[2][i][j][k];
                voxelFace[5] = faces[2][i][j][k + 1];

                voxel1d[k]->faces = voxelFace;
                voxel1d[k]->center = Vector3d(i,j,k) * voxelSize;
                voxel1d[k]->centerVel = Vector3d(0.0,0.0,0.0);
                voxel1d[k]->nextCenterVel = voxel1d[k]->centerVel;
                voxel1d[k]->vort = Vector3d(0.0,0.0,0.0);
                voxel1d[k]->force = Vector3d(0.0,0.0,0.0);
                voxel1d[k]->density = 0.0;
                voxel1d[k]->temp = Tambient;
            }
            voxel2d[j] = voxel1d;
        }
        voxel3d[i] = voxel2d;
    }
    grid = voxel3d;
}

void Grid::initFaces() {
    std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>> x_faces(SIZE_X + 1);
    std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>> y_faces(SIZE_X + 1);
    std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>> z_faces(SIZE_X + 1);

    // Create new faces for each voxel
    for (int i = 0; i < SIZE_X + 1; i++) {
        std::vector<std::vector<std::shared_ptr<VoxelFace>>> x_faces2d(SIZE_Y + 1);
        std::vector<std::vector<std::shared_ptr<VoxelFace>>> y_faces2d(SIZE_Y + 1);
        std::vector<std::vector<std::shared_ptr<VoxelFace>>> z_faces2d(SIZE_Y + 1);

        for (int j = 0; j < SIZE_Y + 1; j++) {
            std::vector<std::shared_ptr<VoxelFace>> x_faces1d(SIZE_Z + 1);
            std::vector<std::shared_ptr<VoxelFace>> y_faces1d(SIZE_Z + 1);
            std::vector<std::shared_ptr<VoxelFace>> z_faces1d(SIZE_Z + 1);

            for(int k = 0; k < SIZE_Z + 1; k++) {
                // Create x,y,z facing faces
                x_faces1d[k] = std::make_shared<VoxelFace>();
                y_faces1d[k] = std::make_shared<VoxelFace>();
                z_faces1d[k] = std::make_shared<VoxelFace>();

                x_faces1d[k]->vel = 0.0;
                y_faces1d[k]->vel = 0.0;
                z_faces1d[k]->vel = 0.0;
                x_faces1d[k]->nextVel = 0.0;
                y_faces1d[k]->nextVel = 0.0;
                z_faces1d[k]->nextVel = 0.0;
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

void Grid::render(std::string number)
{
    std::vector<float> d1d(SIZE_Z);
    std::vector<std::vector<float>> d2d(SIZE_Y);
    std::vector<std::vector<std::vector<float>>> densities(SIZE_X);

    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                d1d[k] = grid[i][j][k]->density;
            }
            d2d[j] = d1d;
        }
        densities[i] = d2d;
    }
    std::string filepathFolders = "C:\\Users\\annaf\\course\\cs2240\\final\\smoke-signal\\src\\rendering\\render1_gs10\\";
    std::string fileName = "SMOKE_";
    std::string fileType = ".vol";
    std::string filePath_FULL = filepathFolders+fileName+number+fileType;
    Rendering::write_vol(filePath_FULL, densities);
}

Grid::~Grid()
{

}
