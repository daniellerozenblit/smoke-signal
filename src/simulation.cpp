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
}


void Simulation::update(float seconds, int total_seconds) {
    m_seconds += timestep;
//    if (m_seconds < emitSeconds) {
//        for (int i = 1; i<4; i++)
//        {
//            for (int k = 0; k<3; k++)
//            {
//                emitSmoke({Vector3i(i,0,k)});
//            }
//        }

//        std::cout << "emit" << std::endl;
//    }

    std::cout << m_seconds << std::endl;
//     std::cout << totalDensity() << std::endl;

    initSphere(grid);
    computeCellCenteredVel();
    addForces();
    updateVelocities();
    advectVelocity();
    solvePressure();
    advectDensityAndTemp();


    ///////////////// IM HERE



//    if ( m_numIterations < 40) //m_numIterations % 5 == 0 &&
//    {
//        std::string ones = std::to_string(m_numIterations % 10);
//        std::string tens = std::to_string((m_numIterations/10) % 10);
//        std::string hundreds = std::to_string((m_numIterations/100) % 10);

//        std::string number = hundreds + tens + ones;
//        grid->render(number);
//    }
    m_numIterations++;
}
