#ifndef SMOKE_H
#define SMOKE_H

#include "grid/macgrid.h"
#include <Eigen/Dense>

using namespace Eigen;

class Smoke
{
public:
    Smoke();

    std::shared_ptr<MACgrid> grid;

    void emitSmoke();
    void advectVelocity();
    void calculateForces();
    void projectPressure();
    void advectTemp();
    void advectDensity();

    // advection helpers
    Vector3d getVelocity(Vector3d pos);
    double interpolate(INTERP_TYPE type, Vector3d pos);
    Vector3d getActualPos(INTERP_TYPE type, Vector3d pos);
};

#endif // SMOKE_H
