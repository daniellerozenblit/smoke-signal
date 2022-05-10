#include "simulation.h"

using namespace Eigen;

Simulation::Simulation() {
    grid = std::make_shared<Grid>();
}

void Simulation::init()
{
    // Mesh translation
    Affine3d t = Affine3d(Translation3d(0, 2, 0));

    // Mesh rotation
    Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(0.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(0.0, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    t.rotate(q);

    Affine3f t_f = t.cast <float> ();
    m_shape.setModelMatrix(t_f);
    initGround();
    initGridViz();

    a = ((float)rand()/(float)RAND_MAX) - 0.5;
    b = ((float)rand()/(float)RAND_MAX) - 0.5;
    c = ((float)rand()/(float)RAND_MAX) - 0.5;

    grid->grid[1][1][1]->density = 1.0;
    //grid->faces[0][1][1][1]->vel = -0.5;
    grid->faces[1][1][1][1]->vel = -0.5;
//    grid->faces[2][1][1][1]->vel = 0.5;
//    grid->faces[0][2][1][1]->vel = 0.5;
//    grid->faces[1][1][2][1]->vel = 0.5;
//    grid->faces[2][1][1][2]->vel = 0.5;
}


void Simulation::update(float seconds, int total_seconds) {
    m_seconds += timestep;

    if (m_seconds < emitSeconds) {
        // emitSmoke({Vector3i(1,1,1)});
    }

    initSphere(grid);
    computeCellCenteredVel();
//    advectVelocity();
//    computeCellCenteredVel();
    addForces();
    updateVelocities();
    computeCellCenteredVel();
    solvePressure();
//    advectTemp();
    advectDensity();
}
