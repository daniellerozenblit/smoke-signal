#ifndef SMOKE_H
#define SMOKE_H

#include "grid/macgrid.h"
#include <Eigen/Dense>
#include <memory>
#include "Eigen/Sparse"

#include "graphics/shape.h"
#include "graphics/sphere.h"
#include "graphics/cone.h"
#include "graphics/cube.h"
#include "graphics/MeshLoader.h"
#include <graphics/Shader.h>

#include "fem/mesh.h"
#include "grid/voxelFace.h"
#include "grid/voxel.h"
#include "grid/grid.h"
#include "rendering/rendering.h"
#include "constants.h"

#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>


using namespace Eigen;

class Smoke
{
public:
    Smoke();

    std::shared_ptr<MACgrid> grid;
    double time_run;

    void update();

    void emitSmoke();
    void advectVelocity();
    void calculateForces();
    void projectPressure();
    void solvePressure();
    void advectTemp();
    void advectDensity();

    // advection helpers
    Vector3d getVelocity(Vector3d pos);
    double interpolate(DATA_TYPE type, Vector3d pos);
    Vector3d getActualPos(DATA_TYPE type, Vector3d pos);
    double getVal(DATA_TYPE type, int i, int j, int k);

    double cubicInterpolator(double prev, double cur, double next, double nextnext, double percent);

    int m_numIterations=0;

    void draw(Shader *shader, Shader *m_normalsShader, Shader *m_normalsArrowShader);
    void toggleWire();
    void interaction(Vector3d dir);
    void tilt_ground(float dir);
    Shape m_sphere;
    Shape arrow;
    Shape stem;

    std::vector<std::vector<Shape>> densitySpheres;
    std::vector<Shape> stems;
    std::vector<Shape> arrows;
    void toggleARROWS();
    void toggleDENSITY();
    void toggleVOXELS();

    bool arrowsBOOL = false;
    bool densitiesBOOL = false;
    bool voxelsBOOL = true;
    double m_seconds = 0.0;

    float a,b,c = 0;
    Shape m_shape;
    Shape m_ground;
    std::vector<Shape> voxels;

    //Shape m_visulizer;
    void initGround();
    void initSphere(std::shared_ptr<Grid> grid);
    void initGridViz();

    std::shared_ptr<Mesh> m_tetmesh;
    std::vector<std::shared_ptr<Collider>> m_colliders;
    std::shared_ptr<Plane> m_ground_collider;

    void initGrid();
    void setfaces(std::vector<std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>>> facesin);

};

#endif // SMOKE_H
