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
#include "rendering/rendering.h"
#include "constants.h"

#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include "grid/macgrid.h"

class Shader;
enum DATA_TYPE;

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
    int m_numIterations = 0;
    double time_run;

    // Equations
    void update();
    void emitSmoke();
    void advectVelocity();
    void calculateForces();
    void projectPressure();
    void solvePressure();
    void advectTemp();
    void advectDensity();

    // Object Collisions
    bool miss_sphere_girl;
    double sphere_thiccness;
    Vector3d sphere_is_where;

    std::shared_ptr<MACgrid> grid;


    // Advection Helpers
    Vector3d getVelocity(Vector3d pos);
    double interpolate(DATA_TYPE type, Vector3d pos);
    Vector3d getActualPos(DATA_TYPE type, Vector3d pos);
    double getVal(DATA_TYPE type, int i, int j, int k);
    double cubicInterpolator(double prev, double cur, double next, double nextnext, double percent);

    std::shared_ptr<Mesh> m_tetmesh;
    std::vector<std::shared_ptr<Collider>> m_colliders;
    std::shared_ptr<Plane> m_ground_collider;

    float a,b,c = 0;

private:
    Shape m_shape;
    Shape m_ground;
    std::vector<Shape> voxels;

    //Shape m_visulizer;
    void initGround();
    void initSphere(std::shared_ptr<MACgrid> grid);
    void initGridViz();
};

#endif // SIMULATION_H
