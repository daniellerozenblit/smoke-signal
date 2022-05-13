#include "simulation.h"

using namespace Eigen;

Simulation::Simulation() :
    time_run(0.0), miss_sphere_girl(true), sphere_thiccness(6.0) {
    grid = std::make_shared<MACgrid>();
    sphere_is_where = Vector3d(5.5, 20.0, 5.5);
}


void Simulation::init() {
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
}


void Simulation::update() {
    std::cout << "update" << std::endl;

    if ( m_numIterations < 40) //m_numIterations % 5 == 0 &&
    {
        std::string ones = std::to_string(m_numIterations % 10);
        std::string tens = std::to_string((m_numIterations/10) % 10);
        std::string hundreds = std::to_string((m_numIterations/100) % 10);

        std::string number = hundreds + tens + ones;
        // grid->render(number);
    }

    if (VISUALIZE) {
        initSphere(grid);
    }

    if (time_run < 2) {
        emitSmoke();
    }
    advectVelocity();
    calculateForces();
    projectPressure();
    advectTemp();
    advectDensity();

    time_run += TIMESTEP;
    m_numIterations++;

}
