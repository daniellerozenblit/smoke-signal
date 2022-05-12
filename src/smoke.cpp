#include "smoke.h"
#include "constants.h"
#include <memory>
#include <iostream>

using namespace Eigen;

Smoke::Smoke()
{
    grid = std::make_shared<MACgrid>();
}

void Smoke::update() {
    std::cout << "update" << std::endl;
//    for (int i = 0; i < SIZE_X; i++) {
//        for (int j = 0; j < SIZE_Y; j++) {
//            for (int k = 0; k < SIZE_Z; k++) {
//                if (getVelocity(Vector3d((i + 0.5) * VOXEL_SIZE, (j + 0.5) * VOXEL_SIZE, (k + 0.5) * VOXEL_SIZE)).norm() > 0) {
//                    std::cout << "CELL: " << i << ", " << j << ", " << k << " has velocity: " << getVelocity(Vector3d((i + 0.5) * VOXEL_SIZE, (j + 0.5) * VOXEL_SIZE, (k + 0.5) * VOXEL_SIZE)) << std::endl;
//                } if (getVal(DENSITY, i, j, k) > 0) {
//                    std::cout << "CELL: " << i << ", " << j << ", " << k << " has density: " << getVal(DENSITY, i, j, k) << std::endl;
//                }
//            }
//        }
//    }
    advectVelocity();
    calculateForces();
    advectDensity();
}

void Smoke::emitSmoke() {
    grid->density[SIZE_X / 2][0][SIZE_Z / 2] = 1.0;
    grid->temperature[SIZE_X / 2][0][SIZE_Z / 2] = 1.0;
    grid->face_vel_y[SIZE_X / 2][0][SIZE_Z / 2] = 2.0;
}

void Smoke::advectVelocity() {
    // each x face
    for(int i = 0; i < SIZE_X + 1; i++){
        for(int j = 0; j < SIZE_Y; j++){
            for(int k = 0; k < SIZE_Z; k++) {
                Vector3d cur_face_center_pos = Vector3d(i * VOXEL_SIZE, (j + 0.5) * VOXEL_SIZE, (k + 0.5) * VOXEL_SIZE);
                Vector3d mid_point_pos = cur_face_center_pos - getVelocity(cur_face_center_pos) * TIMESTEP / 2.0;
                grid->next_face_vel_x[i][j][k] = interpolate(VELOCITY_X, cur_face_center_pos - getVelocity(mid_point_pos) * TIMESTEP);
            }
        }
    }

    // each y face
    for(int i = 0; i < SIZE_X; i++){
        for(int j = 0; j < SIZE_Y + 1; j++){
            for(int k = 0; k < SIZE_Z; k++) {
                Vector3d cur_face_center_pos = Vector3d((i + 0.5) * VOXEL_SIZE, j * VOXEL_SIZE, (k + 0.5) * VOXEL_SIZE);
                Vector3d mid_point_pos = cur_face_center_pos - getVelocity(cur_face_center_pos) * TIMESTEP / 2.0;
                grid->next_face_vel_y[i][j][k] = interpolate(VELOCITY_Y, cur_face_center_pos - getVelocity(mid_point_pos) * TIMESTEP);
            }
        }
    }

    // each z face
    for(int i = 0; i < SIZE_X; i++){
        for(int j = 0; j < SIZE_Y; j++){
            for(int k = 0; k < SIZE_Z + 1; k++) {
                Vector3d cur_face_center_pos = Vector3d((i + 0.5) * VOXEL_SIZE, (j + 0.5) * VOXEL_SIZE, k * VOXEL_SIZE);
                Vector3d mid_point_pos = cur_face_center_pos - getVelocity(cur_face_center_pos) * TIMESTEP / 2.0;
                grid->next_face_vel_z[i][j][k] = interpolate(VELOCITY_Z, cur_face_center_pos - getVelocity(mid_point_pos) * TIMESTEP);
            }
        }
    }

    grid->face_vel_x = grid->next_face_vel_x;
    grid->face_vel_y = grid->next_face_vel_y;
    grid->face_vel_z = grid->next_face_vel_z;

}

void Smoke::calculateForces() {
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                // buoyancy
                double buoyancy_force = -1.0 * -0.08 * ((getVal(DENSITY, i, j, k) + getVal(DENSITY, i, j - 1, k)) / 2.0) + 0.97 * (((getVal(TEMPERATURE, i, j, k) + getVal(TEMPERATURE, i, j - 1, k)) / 2.0) - T_AMBIENT);
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

double Smoke::getDensity(Vector3d pos) {
    return interpolate(DENSITY, pos);
}

double Smoke::getVal(INTERP_TYPE type, int i, int j, int k) {
    switch (type) {
        case VELOCITY_X:
           if (i < 0 || i > SIZE_X) return 0.0;

           if (j < 0) j = 0;
           if (j > SIZE_Y - 1) j = SIZE_Y - 1;
           if (k < 0) k = 0;
           if (k > SIZE_Z - 1) k = SIZE_Z - 1;

           return grid->face_vel_x[i][j][k];
           break;
        case VELOCITY_Y:
           if (j < 0 || j > SIZE_Y) return 0.0;

           if (i < 0) i = 0;
           if (i > SIZE_X - 1) i = SIZE_X - 1;
           if (k < 0) k = 0;
           if (k > SIZE_Z - 1) k = SIZE_Z - 1;

           return grid->face_vel_y[i][j][k];
           break;
        case VELOCITY_Z:
           if (k < 0 || k > SIZE_Z) return 0.0;

           if (i < 0) i = 0;
           if (i > SIZE_X - 1) i = SIZE_X - 1;
           if (j < 0) j = 0;
           if (j > SIZE_Y - 1) j = SIZE_Y - 1;

           return grid->face_vel_z[i][j][k];
           break;
        case DENSITY:
           if (i < 0 || j < 0 || k < 0 ||
               i > SIZE_X - 1 ||
               j > SIZE_Y - 1 ||
               k > SIZE_Z - 1) return 0.0;

           return grid->density[i][j][k];
           break;
       case TEMPERATURE:
           if (i < 0 || j < 0 || k < 0 ||
               i > SIZE_X - 1 ||
               j > SIZE_Y - 1 ||
               k > SIZE_Z - 1) return 0.0;

           return grid->temperature[i][j][k];
           break;
    }
}

double Smoke::interpolate(INTERP_TYPE type, Vector3d pos) {
    Vector3d actual_pos = getActualPos(type, pos);

    int index_i = (int) (actual_pos[0] / VOXEL_SIZE);
    int index_j = (int) (actual_pos[1] / VOXEL_SIZE);
    int index_k = (int) (actual_pos[2] / VOXEL_SIZE);

    double percentage_x = (1.0 / VOXEL_SIZE) * (actual_pos[0] - index_i * VOXEL_SIZE);
    double percentage_y = (1.0 / VOXEL_SIZE) * (actual_pos[1] - index_j * VOXEL_SIZE);
    double percentage_z = (1.0 / VOXEL_SIZE) * (actual_pos[2] - index_k * VOXEL_SIZE);

    double collapsed_once[4][4];
    double collapsed_twice[4];
    double result;

    for (int i = -1; i <= 2; i++) {
        for (int j = -1; j <= 2; j++) {
            collapsed_once[i+1][j+1] = cubicInterpolator(getVal(type, index_i+i,index_j+j,index_k-1), getVal(type, index_i+i,index_j+j,index_k), getVal(type, index_i+i,index_j+j,index_k+1), getVal(type, index_i+i,index_j+j,index_k+2), percentage_z);
        }
    }

    for (int i = -1; i <= 2; i++) {
        collapsed_twice[i+1] = cubicInterpolator(collapsed_once[i+1][0], collapsed_once[i+1][1], collapsed_once[i+1][2], collapsed_once[i+1][3], percentage_y);
    }

    return cubicInterpolator(collapsed_twice[0], collapsed_twice[1], collapsed_twice[2], collapsed_twice[3], percentage_x);
}

Vector3d Smoke::getActualPos(INTERP_TYPE type, Vector3d pos) {
    switch (type) {
        case VELOCITY_X:
            return Vector3d(std::min(std::max(0.0, pos[0]), VOXEL_SIZE * (SIZE_X + 1)), std::min(std::max(0.0, pos[1] - VOXEL_SIZE * 0.5), VOXEL_SIZE * SIZE_Y), std::min(std::max(0.0, pos[2] - VOXEL_SIZE * 0.5), VOXEL_SIZE * SIZE_Z));
            break;
        case VELOCITY_Y:
            return Vector3d(std::min(std::max(0.0, pos[0] - VOXEL_SIZE * 0.5), VOXEL_SIZE * SIZE_X), std::min(std::max(0.0, pos[1]), VOXEL_SIZE * (SIZE_Y + 1)), std::min(std::max(0.0, pos[2] - VOXEL_SIZE * 0.5), VOXEL_SIZE * SIZE_Z));
            break;
        case VELOCITY_Z:
            return Vector3d(std::min(std::max(0.0, pos[0] - VOXEL_SIZE * 0.5), VOXEL_SIZE * SIZE_X), std::min(std::max(0.0, pos[1] - VOXEL_SIZE * 0.5), VOXEL_SIZE * SIZE_Y), std::min(std::max(0.0, pos[2]), VOXEL_SIZE * (SIZE_Z + 1)));
            break;
        default:
            return Vector3d(std::min(std::max(0.0, pos[0] - VOXEL_SIZE * 0.5), VOXEL_SIZE * SIZE_X), std::min(std::max(0.0, pos[1] - VOXEL_SIZE * 0.5), VOXEL_SIZE * SIZE_Y), std::min(std::max(0.0, pos[2] - VOXEL_SIZE * 0.5), VOXEL_SIZE * SIZE_Z));
            break;
    }
}

double Smoke::cubicInterpolator(double prev, double cur, double next, double nextnext, double percent) {
    double slope_1 = (next - prev) / 2.0;
    double slope_2 = (nextnext - cur) / 2.0;

    double dist = (next - cur) / 2.0;

    if (dist > 0.0) {
        slope_1 = std::max(slope_1, 0.0);
        slope_2 = std::max(slope_2, 0.0);
    } else if (dist < 0.0) {
        slope_1 = std::min(slope_1, 0.0);
        slope_2 = std::min(slope_2, 0.0);
    }

    return cur + slope_1 * percent + (3.0 * dist - 2.0 * slope_1 - slope_2) * (percent * percent) + (-2.0 * dist + slope_1 + slope_2) * (percent * percent * percent);
}
