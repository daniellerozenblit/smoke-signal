#ifndef SMOKE_H
#define SMOKE_H

#include "grid/macgrid.h"
#include <Eigen/Dense>
#include <memory>
#include "Eigen/Sparse"


using namespace Eigen;

class Smoke
{
public:
    Smoke();

    std::shared_ptr<MACgrid> grid;

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
    double interpolate(INTERP_TYPE type, Vector3d pos);
    Vector3d getActualPos(INTERP_TYPE type, Vector3d pos);
    double getVal(INTERP_TYPE type, int i, int j, int k);

    double cubicInterpolator(double prev, double cur, double next, double nextnext, double percent);
};

#endif // SMOKE_H
