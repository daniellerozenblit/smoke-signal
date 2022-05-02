#ifndef SIMULATION_H
#define SIMULATION_H

#include "graphics/shape.h"
#include "fem/mesh.h"
#include "Eigen/Sparse"
#include "grid/voxelFace.h"
#include "grid/voxel.h"
#include "grid/grid.h"

class Shader;

class Simulation
{
public:
    Simulation();

    void init();

    void update(float seconds);

    void draw(Shader *shader);

    void toggleWire();

    void interaction(Vector3d dir);
    void tilt_ground(float dir);
    std::shared_ptr<Grid> grid;


private:
    Shape m_shape;
    Shape m_ground;
    Shape m_sphere;
    void initGround();
    void initSphere();
    std::shared_ptr<Mesh> m_tetmesh;
    std::vector<std::shared_ptr<Collider>> m_colliders;
    std::shared_ptr<Plane> m_ground_collider;


    void update();

    void updateVelocities();
    void defForces();
    void defVelocities();
    void advect();
    void createSparsePressure();
    void solveSparsePressure();
    void advectVelocity();
    void advectPressure();

    void cubicInterpolator();

    // stuff for solver

    std::vector<Triplet<double>> tripletList;
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> ICCG;

    Eigen::SparseMatrix<double, Eigen::RowMajor> A;
    Eigen::VectorXd b;
    Eigen::VectorXd x;
};

#endif // SIMULATION_H
