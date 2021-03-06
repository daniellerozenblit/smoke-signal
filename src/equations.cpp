#include "simulation.h"
#include "constants.h"


using namespace Eigen;

void Simulation::emitSmoke() {
    for(int i = 3; i < 6; i++){
        for(int k = 3; k < 6; k++) {
            grid->density[i][0][k] = 1.0;
            grid->temperature[i][0][k] = T_AMBIENT;
            grid->face_vel_y[i][1][k] = 10.0;
        }
    }
}

void Simulation::advectVelocity() {
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

void Simulation::calculateForces() {
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                // buoyancy
                double buoyancy_force = -1.0 * -0.08 * ((grid->getVal(DENSITY, i, j, k) + grid->getVal(DENSITY, i, j - 1, k)) / 2.0) + 0.97 * (((grid->getVal(TEMPERATURE, i, j, k) + grid->getVal(TEMPERATURE, i, j - 1, k)) / 2.0) - T_AMBIENT);
                grid->face_vel_y[i][j][k] += TIMESTEP * buoyancy_force;
            }
        }

    }

    // voriticity
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                //center velocity
                grid->center_vel_x[i][j][k] = (grid->getVal(VELOCITY_X, i, j, k) + grid->getVal(VELOCITY_X, i+1, j, k)) / 2.0;
                grid->center_vel_y[i][j][k] = (grid->getVal(VELOCITY_Y, i, j, k) + grid->getVal(VELOCITY_Y, i, j+1, k)) / 2.0;
                grid->center_vel_z[i][j][k] = (grid->getVal(VELOCITY_Z, i, j, k) + grid->getVal(VELOCITY_Z, i, j, k+1)) / 2.0;
            }
        }
    }

    // calculate the vorticity based on the center velocities
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                grid->vorticity_x[i][j][k] = (grid->getVal(CENTER_VEL_Z, i, j+1, k) - grid->getVal(CENTER_VEL_Z, i, j-1, k) - grid->getVal(CENTER_VEL_Y, i, j, k+1) + grid->getVal(CENTER_VEL_Y, i, j, k-1)) / (2.0 * VOXEL_SIZE);
                grid->vorticity_y[i][j][k] = (grid->getVal(CENTER_VEL_X, i, j, k+1) - grid->getVal(CENTER_VEL_X, i, j, k-1) - grid->getVal(CENTER_VEL_Z, i+1, j, k) + grid->getVal(CENTER_VEL_Z, i-1, j, k)) / (2.0 * VOXEL_SIZE);
                grid->vorticity_z[i][j][k] = (grid->getVal(CENTER_VEL_Y, i+1, j, k) - grid->getVal(CENTER_VEL_Y, i-1, j, k) - grid->getVal(CENTER_VEL_X, i, j+1, k) + grid->getVal(CENTER_VEL_X, i, j-1, k)) / (2.0 * VOXEL_SIZE);

                //add these to a vector a take the norm to get total vorticity
                Vector3d vort = Vector3d(grid->vorticity_x[i][j][k], grid->vorticity_y[i][j][k], grid->vorticity_z[i][j][k]);
                grid->vorticity[i][j][k] = vort.norm();
            }
        }
    }

    // gradient of vorticity
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                grid->vorticity_grad_x[i][j][k] = (grid->getVal(VORTICITY, i+1, j, k) - grid->getVal(VORTICITY, i-1, j, k)) / (2 * VOXEL_SIZE);
                grid->vorticity_grad_y[i][j][k] = (grid->getVal(VORTICITY, i, j+1, k) - grid->getVal(VORTICITY, i, j-1, k)) / (2 * VOXEL_SIZE);
                grid->vorticity_grad_z[i][j][k] = (grid->getVal(VORTICITY, i, j, k+1) - grid->getVal(VORTICITY, i, j, k-1)) / (2 * VOXEL_SIZE);
            }
        }
    }

    //vorticity confinement force for each cell
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                Vector3d vorticity_grad = Vector3d(grid->vorticity_grad_x[i][j][k], grid->vorticity_grad_y[i][j][k], grid->vorticity_grad_z[i][j][k]);
                vorticity_grad.normalize();
                Vector3d vorticity = Vector3d(grid->vorticity_x[i][j][k],grid->vorticity_y[i][j][k],grid->vorticity_z[i][j][k]);
                Vector3d vcf = vorticity_grad.cross(vorticity)*VOXEL_SIZE*EPSILON;

                grid->vcf_x[i][j][k] = vcf[0];
                grid->vcf_y[i][j][k] = vcf[1];
                grid->vcf_z[i][j][k] = vcf[2];
            }
        }
    }

    // Add this to face velocities
    for (int i = 0; i < SIZE_X+1; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                grid->face_vel_x[i][j][k] += TIMESTEP * (grid->getVal(VCF_X, i-1, j, k) + grid->getVal(VCF_X, i, j, k)) / FLUID_DENSE * 2.0;
            }
        }
    }

    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y+1; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                grid->face_vel_y[i][j][k] += TIMESTEP * (grid->getVal(VCF_Y, i, j-1, k) + grid->getVal(VCF_Y, i, j, k)) / FLUID_DENSE * 2.0;
            }
        }
    }

    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z+1; k++) {
                grid->face_vel_z[i][j][k] += TIMESTEP * (grid->getVal(VCF_Z, i, j, k-1) + grid->getVal(VCF_Z, i, j, k)) / FLUID_DENSE * 2.0;
            }
        }
    }
}

void Simulation::projectPressure()
{

    Eigen::VectorXd b(SIZE_CUBE);
    b.setZero();
    Eigen::VectorXd p(SIZE_CUBE);
    p.setZero();

    //DIVERGENCE CALCULATIONS
    for(int i = 0; i<SIZE_X; i++)
    {
        for (int j = 0; j<SIZE_Y; j++)
        {
            for (int k = 0; k< SIZE_Z; k++)
            {
                double xm = grid->getVal(VELOCITY_X, i, j, k);
                double xp = grid->getVal(VELOCITY_X, i+1, j, k);
                double ym = grid->getVal(VELOCITY_Y, i, j, k);
                double yp = grid->getVal(VELOCITY_Y, i, j+1, k);
                double zm = grid->getVal(VELOCITY_Z, i, j, k);
                double zp = grid->getVal(VELOCITY_Z, i, j, k+1);
                if (i==0)
                {
                    xm = 0.0;
                } else if (i==SIZE_X-1)
                {
                    xp = 0.0;
                }
                if (j==0)
                {
                    ym = 0.0;
                } else if (j==SIZE_Y-1)
                {
                    yp = 0.0;
                }
                if (k==0)
                {
                    zm = 0.0;
                } else if (k==SIZE_Z-1)
                {
                    zp = 0.0;
                }
                grid->divergence[i][j][k] = -((xp - xm)+(yp - ym)+(zp - zm))/VOXEL_SIZE;
                b[INDEX(i,j,k)] = grid->divergence[i][j][k];
            }
        }
    }

    // solve pressures
    p = grid->solver.solve(b);

    // turn pressures from 1D back to 3D
    for(int i = 0; i<SIZE_X; i++)
    {
        for (int j = 0; j<SIZE_Y; j++)
        {
            for (int k = 0; k< SIZE_Z; k++)
            {
                grid->pressure[i][j][k] = p[INDEX(i,j,k)] * (AIR_DENSE * (VOXEL_SIZE * VOXEL_SIZE))/ TIMESTEP;
            }
        }
    }

    // check pressures and apply to velocities for each of the faces
    for(int i = 0; i<SIZE_X+1; i++)
    {
        for (int j = 0; j<SIZE_Y; j++)
        {
            for (int k = 0; k< SIZE_Z; k++)
            {
                double highPressure;
                double lowPressure;
                if(i-1>=0)
                {
                    lowPressure = grid->getVal(PRESSURE, i-1,j,k);
                }
                if(i<SIZE_X)
                {
                    highPressure = grid->getVal(PRESSURE, i,j,k);
                }
                if(i-1<0)
                {
                    lowPressure = highPressure - FLUID_DENSE*VOXEL_SIZE/TIMESTEP*grid->face_vel_x[i][j][k];
                }
                if(i>=SIZE_X)
                {
                    highPressure = lowPressure + FLUID_DENSE*VOXEL_SIZE/TIMESTEP*grid->face_vel_x[i][j][k];
                }

                grid->face_vel_x[i][j][k] -= TIMESTEP/AIR_DENSE * (highPressure - lowPressure)/VOXEL_SIZE;
            }
        }
    }

    for(int i = 0; i<SIZE_X; i++)
    {
        for (int j = 0; j<SIZE_Y+1; j++)
        {
            for (int k = 0; k< SIZE_Z; k++)
            {
                double highPressure;
                double lowPressure;
                if(j-1>=0)
                {
                    lowPressure = grid->getVal(PRESSURE, i,j-1,k);
                }
                if(j<SIZE_Y)
                {
                    highPressure = grid->getVal(PRESSURE, i,j,k);
                }
                if(j-1<0)
                {
                    lowPressure = highPressure - FLUID_DENSE*VOXEL_SIZE/TIMESTEP*grid->face_vel_y[i][j][k];
                }
                if(j>=SIZE_Y)
                {
                    highPressure = lowPressure + FLUID_DENSE*VOXEL_SIZE/TIMESTEP*grid->face_vel_y[i][j][k];
                }

                grid->face_vel_y[i][j][k] -= TIMESTEP/AIR_DENSE * (highPressure - lowPressure)/VOXEL_SIZE;
            }
        }
    }

    for(int i = 0; i<SIZE_X; i++)
    {
        for (int j = 0; j<SIZE_Y; j++)
        {
            for (int k = 0; k< SIZE_Z+1; k++)
            {
                double highPressure;
                double lowPressure;
                if(k-1>=0)
                {
                    lowPressure = grid->getVal(PRESSURE, i,j,k-1);
                }
                if(k<SIZE_Z)
                {
                    highPressure = grid->getVal(PRESSURE, i,j,k);
                }
                if(k-1<0)
                {
                    lowPressure = highPressure - FLUID_DENSE*VOXEL_SIZE/TIMESTEP*grid->face_vel_z[i][j][k];
                }
                if(k>=SIZE_Z)
                {
                    highPressure = lowPressure + FLUID_DENSE*VOXEL_SIZE/TIMESTEP*grid->face_vel_z[i][j][k];
                }

                grid->face_vel_z[i][j][k] -= TIMESTEP/AIR_DENSE * (highPressure - lowPressure)/VOXEL_SIZE;
            }
        }
    }
}


void Simulation::advectDensity() {
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                Vector3d cur_voxel_center_pos = Vector3d((i + 0.5) * VOXEL_SIZE, (j + 0.5) * VOXEL_SIZE, (k + 0.5) * VOXEL_SIZE);
                Vector3d cur_center_vel = getVelocity(cur_voxel_center_pos);
                Vector3d mid_point_pos = cur_voxel_center_pos - cur_center_vel * TIMESTEP / 2.0;
                Vector3d back_traced_pos = cur_voxel_center_pos - getVelocity(mid_point_pos);

                grid->next_density[i][j][k] = interpolate(DENSITY, back_traced_pos);
            }
        }
    }

    grid->density = grid->next_density;
}

void Simulation::advectTemp() {
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                Vector3d cur_voxel_center_pos = Vector3d((i + 0.5) * VOXEL_SIZE, (j + 0.5) * VOXEL_SIZE, (k + 0.5) * VOXEL_SIZE);
                Vector3d cur_center_vel = getVelocity(cur_voxel_center_pos);
                Vector3d mid_point_pos = cur_voxel_center_pos - cur_center_vel * TIMESTEP / 2.0;
                Vector3d back_traced_pos = cur_voxel_center_pos - getVelocity(mid_point_pos);

                grid->next_temperature[i][j][k] = interpolate(TEMPERATURE, back_traced_pos);
            }
        }
    }

    grid->temperature = grid->next_temperature;
}

Vector3d Simulation::getVelocity(Vector3d pos) {
    Vector3d velocity(0.0, 0.0, 0.0);

    if (!miss_sphere_girl) {
        velocity[0] = interpolate(VELOCITY_X, pos);
        velocity[1] = interpolate(VELOCITY_Y, pos);
        velocity[2] = interpolate(VELOCITY_Z, pos);
    } else {
        Vector3d center = VOXEL_SIZE * sphere_is_where;
        double dist = (pos - center).norm();
        if (dist < sphere_thiccness) {
            Vector3d v1 = Vector3d(interpolate(VELOCITY_X, pos), interpolate(VELOCITY_Y, pos), interpolate(VELOCITY_Z, pos));
            Vector3d v2 = v1.dot(pos - center) * (pos - center).normalized();
            return v1 - v2;
        } else {
            velocity[0] = interpolate(VELOCITY_X, pos);
            velocity[1] = interpolate(VELOCITY_Y, pos);
            velocity[2] = interpolate(VELOCITY_Z, pos);
        }
    }

    return velocity;
}

double Simulation::interpolate(DATA_TYPE type, Vector3d pos) {
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
            collapsed_once[i+1][j+1] = cubicInterpolator(grid->getVal(type, index_i+i,index_j+j,index_k-1), grid->getVal(type, index_i+i,index_j+j,index_k), grid->getVal(type, index_i+i,index_j+j,index_k+1), grid->getVal(type, index_i+i,index_j+j,index_k+2), percentage_z);
        }
    }

    for (int i = -1; i <= 2; i++) {
        collapsed_twice[i+1] = cubicInterpolator(collapsed_once[i+1][0], collapsed_once[i+1][1], collapsed_once[i+1][2], collapsed_once[i+1][3], percentage_y);
    }

    return cubicInterpolator(collapsed_twice[0], collapsed_twice[1], collapsed_twice[2], collapsed_twice[3], percentage_x);
}

Vector3d Simulation::getActualPos(DATA_TYPE type, Vector3d pos) {
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

double Simulation::cubicInterpolator(double prev, double cur, double next, double nextnext, double percent) {
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
