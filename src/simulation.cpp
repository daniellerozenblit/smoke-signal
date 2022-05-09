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

}


void Simulation::update(float seconds, int total_seconds) {
    m_seconds += timestep;

    std::cout << total_seconds << std::endl;
    if (m_seconds < emitSeconds) {
        emitSmoke({Vector3i(3,3,3)});
        emitSmoke({Vector3i(4,3,3)});
        emitSmoke({Vector3i(2,3,3)});
    }

    computeCellCenteredVel();
    advectVelocity();
    computeCellCenteredVel();
    addForces();
    updateVelocities();
    computeCellCenteredVel();
    solvePressure();
    computeCellCenteredVel();
//    advectTemp();
    advectDensity();
    initSphere(grid);
}
