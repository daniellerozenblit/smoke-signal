#include "smoke.h"
#include "constants.h"
#include <memory>

using namespace Eigen;

Smoke::Smoke()
{
    grid = std::make_shared<MACgrid>();
}

void Smoke::emitSmoke() {
    grid->density[SIZE_X / 2][0][SIZE_Z / 2] = 1.0;
    grid->temperature[SIZE_X / 2][0][SIZE_Z / 2] = 1.0;
    grid->face_vel_y[SIZE_X / 2][0][SIZE_Z / 2] = 2.0;
}

void Smoke::advectVelocity() {

}

void Smoke::calculateForces() {
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                // buoyancy
                double buoyancy_force = -1.0 * -0.08 * ((grid->density[i][j][k] + grid->density[i][j-1][k]) / 2.0) + 0.97 * (((grid->temperature[i][j][k] + grid->temperature[i][j-1][k]) / 2.0) - T_AMBIENT);
                grid->face_vel_y[i][j][k] += TIMESTEP * buoyancy_force;

                // voriticity ?
            }
        }

    }


}

void Smoke::advectDensity() {
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                Vector3d cur_voxel_center_pos = Vector3d((i + 0.5) * VOXEL_SIZE, (j + 0.5) * VOXEL_SIZE, (k + 0.5) * VOXEL_SIZE);
                Vector3d cur_center_vel = getVelocity(cur_voxel_center_pos);
                Vector3d mid_point_pos = cur_voxel_center_pos - cur_center_vel * TIMESTEP / 2.0;
                Vector3d back_traced_pos = cur_voxel_center_pos - getVelocity(mid_point_pos);
                double new_density = getDensity(back_traced_pos);
                grid->next_density[i][j][k] = new_density;
            }
        }
    }

    grid->density = grid->next_density;
}

Vector3d Smoke::getVelocity(Vector3d pos) {
    Vector3d velocity(0.0, 0.0, 0.0);

    velocity[0] = interpolate(INTERP_TYPE::VELOCITY_X, pos);
    velocity[1] = interpolate(INTERP_TYPE::VELOCITY_Y, pos);
    velocity[2] = interpolate(INTERP_TYPE::VELOCITY_Z, pos);

    return velocity;
}

double Smoke::interpolate(INTERP_TYPE type, Vector3d pos) {
    Vector3d actual_pos = getActualPos(type, pos);
}

Vector3d Smoke::getActualPos(INTERP_TYPE type, Vector3d pos) {
    switch (type) {
        case INTERP_TYPE::VELOCITY_X:
            return Vector3d(std::min(std::max(0.0, pos[0]), VOXEL_SIZE * (SIZE_X + 1)), std::min(std::max(0.0, pos[1] - VOXEL_SIZE * 0.5), VOXEL_SIZE * SIZE_Y), std::min(std::max(0.0, pos[2] - VOXEL_SIZE * 0.5), VOXEL_SIZE * SIZE_Z);
            break;
    }
}
