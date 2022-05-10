#ifndef SIMULATION_H
#define SIMULATION_H

#include "graphics/shape.h"
#include "graphics/sphere.h"
#include "graphics/cone.h"
#include "graphics/cube.h"
#include "graphics/MeshLoader.h"
#include <graphics/Shader.h>

#include "fem/mesh.h"
#include "Eigen/Sparse"
#include "grid/voxelFace.h"
#include "grid/voxel.h"
#include "grid/grid.h"
#include "rendering/rendering.h"
#include "constants.h"

#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>


class Shader;
enum INTERP_TYPE;

class Simulation
{
public:
    Simulation();

    void init();

    void update(float seconds, int total_seconds);

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

private:
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

    std::shared_ptr<Grid> grid;

    void initGrid();
    void setfaces(std::vector<std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>>> facesin);

    void emitSmoke(std::vector<Eigen::Vector3i> indices);


    void update();

    // Equations
    void updateVelocities();
    void defForces();
    void defVelocities();
    void advect();
    void solvePressure();
    void advectVelocity();
    void advectPressure();
    void advectTemp();
    void advectDensity();
    double clamp(double input);
    void addForces();
    void computeCellCenteredVel();

    Vector4i clampIndex(Vector4i(index));
    int sign(double x);
    double zero(double x);
    Vector3d zero(Vector3d x);
    double clampUnit(double x);
    Vector3d clampUnit(Vector3d x);

    // Interpolation
    Vector3d getVel(Vector3d &pos);
    double getVelAxis(Vector3d &pos, int axis);
    double cubicInterpolator(Vector3d position, INTERP_TYPE var, int axis);
    double collapseAxis(Vector4d input, double percentage);


    double totalDensity();
};

#endif // SIMULATION_H
