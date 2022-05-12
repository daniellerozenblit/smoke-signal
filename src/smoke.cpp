#include "smoke.h"
#include "constants.h"

Smoke::Smoke()
{

}

void Smoke::emitSmoke() {
    // set v of i,j,k to 2
    // set d of i,j,k to 1
    // set t of i,j,k to 1
}

void Smoke::advectVelocity() {

}

void Smoke::calculateForces() {
    for(int i = 0; i < SIZE_X; i++) {
        for(int j = 0; j < SIZE_Y; j++) {
            for(int k = 0; k < SIZE_Z; k++) {
                // buoyancy
                double buoyancy_force = -1.0 * -0.08 * (grid_density[i][j][k] + grid_density[i][j][k] )
            }
        }

    }


}

void Smoke::advectDensity() {

}
