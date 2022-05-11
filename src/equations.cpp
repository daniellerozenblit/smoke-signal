#include "simulation.h"
#include "constants.h"


using namespace Eigen;

// flag every single voxel which intersects with the object as being occupied,
// set velocity to be same as immersed object, set temperature same as well
// DENSITY ZERO in intersecting voxel... boundary voxel desity = closest unoccupied voxel

//// smoke emission! pass in a list of indices of emitting voxels and it will set their density to 1 and upward velocity to 50
void Simulation::emitSmoke(std::vector<Eigen::Vector3i> indices) {
    for (auto voxel_index : indices) {
         grid->grid[voxel_index[0]][voxel_index[1]][voxel_index[2]]->density = 1.0;
         grid->faces[1][voxel_index[0]][voxel_index[1]][voxel_index[2]]->vel = 90.0;
    }
}

void Simulation::updateVelocities() {
    // Update face velocities with forces
    for (int i = 0; i < SIZE_X + 1; i++) {
        for (int j = 0; j < SIZE_Y + 1; j++) {
            for(int k = 0; k < SIZE_Z + 1; k++) {
                // Edge cases
                if (i == 0 || i == SIZE_X) {
                    grid->faces[0][i][j][k]->vel = 0.0;
                } else if (j < SIZE_Y  && k < SIZE_Z) {
                    grid->faces[0][i][j][k]->vel += (grid->grid[i][j][k]->force[0] + grid->grid[i - 1][j][k]->force[0]) * timestep / 2.0;
                }

                if (j == 0 || j == SIZE_Y) {
                    grid->faces[1][i][j][k]->vel = 0.0;
                } else if (i < SIZE_X  && k < SIZE_Z ) {
                    grid->faces[1][i][j][k]->vel += (grid->grid[i][j][k]->force[1] + grid->grid[i][j - 1][k]->force[1]) * timestep / 2.0;
                }

                if (k == 0 || k == SIZE_Z) {
                    grid->faces[2][i][j][k]->vel = 0.0;
                } else if (i < SIZE_X  && j < SIZE_Y ) {
                    grid->faces[2][i][j][k]->vel += (grid->grid[i][j][k]->force[2] + grid->grid[i][j][k - 1]->force[2]) * timestep / 2.0;
                }
            }
        }
    }
}

void Simulation::addForces() {
    // Calculate the vorticities for each cell
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for(int k = 0; k < SIZE_Z; k++) {
                // Border Cases
                if (i == 0 || j == 0 || k == 0 || i == SIZE_X - 1 || j == SIZE_Y - 1 || k == SIZE_Z - 1) {
                    grid->grid[i][j][k]->vort = Eigen::Vector3d(0.0, 0.0, 0.0);
                    continue;
                }

                float vort_x = (grid->grid[i][j + 1][k]->centerVel[2] - grid->grid[i][j - 1][k]->centerVel[2]
                        - grid->grid[i][j][k + 1]->centerVel[1] + grid->grid[i][j][k - 1]->centerVel[1]) / (2.0 * voxelSize);

                float vort_y = (grid->grid[i][j][k + 1]->centerVel[0] - grid->grid[i][j][k - 1]->centerVel[0]
                        - grid->grid[i + 1][j][k]->centerVel[2] + grid->grid[i - 1][j][k]->centerVel[2]) / (2.0 * voxelSize);

                float vort_z = (grid->grid[i + 1][j][k]->centerVel[1] - grid->grid[i - 1][j][k]->centerVel[1]
                        - grid->grid[i][j + 1][k]->centerVel[0] + grid->grid[i][j - 1][k]->centerVel[0]) / (2.0 * voxelSize);

                grid->grid[i][j][k]->vort = Eigen::Vector3d(vort_x, vort_y, vort_z);
            }
        }
    }

    // Calculate the confinement force for each cell
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for(int k = 0; k < SIZE_Z; k++) {
                 // Add vertical buoyancy force to z axis where z = 1 is up (eqn. 8)
                 grid->grid[i][j][k]->force = Vector3d();
                 grid->grid[i][j][k]->force[0] = 0.0;
                 grid->grid[i][j][k]->force[1] = -1.0 * alpha * grid->grid[i][j][k]->density + beta * (grid->grid[i][j][k]->temp - Tambient);
                 grid->grid[i][j][k]->force[2] = 0.0;

                // Border Cases
                if (i == 0 || j == 0 || k == 0 || i == SIZE_X - 1 || j == SIZE_Y - 1 || k == SIZE_Z - 1) {
                    continue;
                }

                // Gradient of vorticity
                double g_x = grid->grid[i + 1][j][k]->vort.norm() - grid->grid[i - 1][j][k]->vort.norm() / (2.0 * voxelSize);
                double g_y = grid->grid[i][j + 1][k]->vort.norm() - grid->grid[i][j - 1][k]->vort.norm() / (2.0 * voxelSize);
                double g_z = grid->grid[i][j][k + 1]->vort.norm() - grid->grid[i][j][k - 1]->vort.norm() / (2.0 * voxelSize);

                // Normalized vorticity location vector
                Eigen::Vector3d N = Eigen::Vector3d(g_x, g_y, g_z).normalized();

                // Add confinement force to voxel forces
                Eigen::Vector3d f_c = VORT_EPSILON * voxelSize * N.cross(grid->grid[i][j][k]->vort);
                grid->grid[i][j][k]->force += f_c;
            }
        }
    }
}

void Simulation::advectVelocity() {
    for (int i = 0; i < SIZE_X + 1; i++) {
        for (int j = 0; j < SIZE_Y + 1; j++) {
            for (int k = 0; k < SIZE_Z + 1; k++) {
                // Iterate 3 times for x, y, and z faces
                for (int c = 0; c < 3; c++) {
                    Eigen::Vector3d pos;
                    switch(c) {
                        case 0: //x
                            pos = Vector3d(i - 0.5, j, k) * voxelSize;
                            break;
                        case 1: //y
                            pos = Vector3d(i, j - 0.5, k) * voxelSize;
                            break;
                        case 2: //z
                            pos = Vector3d(i, j, k - 0.5) * voxelSize;
                            break;
                    }

                    Eigen::Vector3d m_pos =  pos - getVel(pos) * timestep / 2.0;
                    Eigen::Vector3d o_pos = pos - getVel(m_pos) * timestep;
                    double newVel = getVelAxis(o_pos, c);
                    grid->faces[c][i][j][k]->nextVel = newVel;
                }
            }
        }
    }

    for (int i = 0; i < SIZE_X + 1; i++) {
        for (int j = 0; j < SIZE_Y + 1; j++) {
            for (int k = 0; k < SIZE_Z + 1; k++) {
                for (int c = 0; c < 3; c++) {
                    grid->faces[c][i][j][k]->vel = grid->faces[c][i][j][k]->nextVel;
                }
            }
        }
    }
}

void Simulation::advectDensityAndTemp() {
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                Eigen::Vector3d pos = Vector3d(i, j, k) * voxelSize;
                Eigen::Vector3d m_pos =  pos - getVel(pos) * timestep / 2.0;
                Eigen::Vector3d o_pos = pos - getVel(m_pos) * timestep - Vector3d(0.5, 0.5, 0.5) * voxelSize;;

                if (ADVECT_DENSITY) {
                    double newDensity = cubicInterpolator(o_pos, INTERP_TYPE::DENSITY, -1);
                    grid->grid[i][j][k]->nextDensity = newDensity;
                }

                if (ADVECT_TEMP) {
                    double newTemp = cubicInterpolator(o_pos, INTERP_TYPE::TEMPERATURE, -1);
                    grid->grid[i][j][k]->nextTemp = newTemp;
                }
            }
        }
    }

    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                if (ADVECT_DENSITY) {
                    grid->grid[i][j][k]->density = grid->grid[i][j][k]->nextDensity;
                }

                if (ADVECT_TEMP) {
                    grid->grid[i][j][k]->temp = grid->grid[i][j][k]->nextTemp;
                }
            }
        }
    }
}

void Simulation::solvePressure() {
    std::vector<Triplet<double>> t;
    // TODO: try other solvers - SparseLDLT
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> solver;
    Eigen::SparseMatrix<double, Eigen::RowMajor> A(SIZE_CUBE, SIZE_CUBE);
    A.setZero();
    Eigen::VectorXd b(SIZE_CUBE);
    b.setZero();
    Eigen::VectorXd p(SIZE_CUBE);
    p.setZero();

    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {

                // Calculate b (divergence vector) based on the intermediate face velocities
                b[INDEX(i, j, k)] = 0.0;

                // Neighboring voxels
                double neighbors = 0.0;

                if (i > 0) {
                    b[INDEX(i, j, k)] -= grid->faces[0][i][j][k]->vel;
                    neighbors += 1.0;
                    t.push_back(Eigen::Triplet(INDEX(i, j, k), INDEX(i - 1, j, k), 1.0));
                }

                if (j > 0) {
                    b[INDEX(i, j, k)] -= grid->faces[1][i][j][k]->vel;
                    neighbors += 1.0;
                    t.push_back(Eigen::Triplet(INDEX(i, j, k), INDEX(i, j - 1, k), 1.0));
                }

                if (k > 0) {
                    b[INDEX(i, j, k)] -= grid->faces[2][i][j][k]->vel;
                    neighbors += 1.0;
                    t.push_back(Eigen::Triplet(INDEX(i, j, k), INDEX(i, j, k - 1), 1.0));
                }

                if (i < SIZE_X - 1) {
                    b[INDEX(i, j, k)] += grid->faces[0][i + 1][j][k]->vel;
                    neighbors += 1.0;
                    t.push_back(Eigen::Triplet(INDEX(i, j, k), INDEX(i + 1, j, k), 1.0));
                }

                if (j < SIZE_Y - 1) {
                    b[INDEX(i, j, k)] += grid->faces[1][i][j + 1][k]->vel;
                    neighbors += 1.0;
                    t.push_back(Eigen::Triplet(INDEX(i, j, k), INDEX(i, j + 1, k), 1.0));
                }

                if (k < SIZE_Z - 1) {
                    b[INDEX(i, j, k)] += grid->faces[2][i][j][k + 1]->vel;
                    neighbors += 1.0;
                    t.push_back(Eigen::Triplet(INDEX(i, j, k), INDEX(i, j, k + 1), 1.0));
                }

                // Diagonal
                t.push_back(Eigen::Triplet(INDEX(i, j, k), INDEX(i, j, k), -neighbors));
            }
        }
    }

    // Solve sparse linear system
    b *= 1.0 / voxelSize;
    A.setFromTriplets(t.begin(), t.end());
    solver.compute(A);
    p = solver.solve(b);

    // Adjust face velocities based on pressure
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                // TODO: do the timestep and voxelSize cancel out?
                if (i < SIZE_X - 1) {
                    grid->faces[0][i + 1][j][k]->vel -= (p[INDEX(i + 1, j, k)] - p[INDEX(i, j, k)]) * timestep / voxelSize;
                }

                if (j < SIZE_Y - 1) {
                    grid->faces[1][i][j + 1][k]->vel -= (p[INDEX(i, j + 1, k)] - p[INDEX(i, j, k)]) * timestep / voxelSize;
                }

                if (k < SIZE_Z - 1) {
                    grid->faces[2][i][j][k + 1]->vel -= (p[INDEX(i, j, k + 1)] - p[INDEX(i, j, k)]) * timestep / voxelSize;
                }
            }
        }
    }
}

void Simulation::computeCellCenteredVel() {
    // Find the cell-centered velocities in each direction
    double avg_u;
    double avg_v;
    double avg_w;

    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for(int k = 0; k < SIZE_Z; k++) {
                avg_u = (grid->grid[i][j][k]->faces[0]->vel + grid->grid[i][j][k]->faces[1]->vel) / 2.0;
                avg_v = (grid->grid[i][j][k]->faces[2]->vel + grid->grid[i][j][k]->faces[3]->vel) / 2.0;
                avg_w = (grid->grid[i][j][k]->faces[4]->vel + grid->grid[i][j][k]->faces[5]->vel) / 2.0;

                grid->grid[i][j][k]->centerVel = Vector3d(avg_u, avg_v, avg_w);
            }
        }
    }
}


double Simulation::cubicInterpolator(Vector3d position, INTERP_TYPE var, int axis) {
    // Get coords and clamp within grid bounds
    Vector3d posClamped = clampPos(position);
    Vector3i indexCast;
    Vector3d percentage;

    // Find the voxel index based on clamped position
    for (int c = 0; c < 3 ; c++) {
        indexCast[c] = (int) (posClamped[c] / voxelSize);
        percentage[c] = posClamped[c] / voxelSize - (double) indexCast[c];
        assert (percentage[c] < 1.0 && percentage[c] >= 0);
    }



    // Compute indices for the interpolation
    Vector4i x_indices = clampIndex(Vector4i{indexCast[0] - 1, indexCast[0], indexCast[0] + 1, indexCast[0] + 2}, 0);
    Vector4i y_indices = clampIndex(Vector4i{indexCast[1] - 1, indexCast[1], indexCast[1] + 1, indexCast[1] + 2}, 1);
    Vector4i z_indices = clampIndex(Vector4i{indexCast[2] - 1, indexCast[2], indexCast[2] + 1, indexCast[2] + 2}, 2);

    // Nested collapse on each axis using the coordinates
    Vector4d Xcollapse;
    for(int i = 0; i < 4; i++) {
        Vector4d Ycollapse;
        for(int j = 0; j < 4; j++) {
            Vector4d Zcollapse;
            for(int k = 0; k < 4; k++) {
                switch (var) {
                    case INTERP_TYPE::DENSITY:
                        Zcollapse[k] = grid->grid[x_indices[i]][y_indices[j]][z_indices[k]]->density;
                        break;
                    case INTERP_TYPE::TEMPERATURE:
                        Zcollapse[k] = grid->grid[x_indices[i]][y_indices[j]][z_indices[k]]->temp;
                        break;
                    case INTERP_TYPE::VELOCITY:
                        // TODO: do we need to add 1 for faces? yes
                        Zcollapse[k] = grid->faces[axis][x_indices[i] + 1][y_indices[j] + 1][z_indices[k] + 1]->vel;
                        break;
                }
            }
            Ycollapse[j] = collapseAxis(Zcollapse, percentage[2]);
        }
        Xcollapse[i] = collapseAxis(Ycollapse, percentage[1]);
    }
    return collapseAxis(Xcollapse, percentage[0]);
}


double Simulation::collapseAxis(Vector4d f, double t) {
    double deltak = f[2] - f[1];
    double dk = (f[2] - f[0]) / 2.0;
    double dk1 = (f[3] - f[1]) / 2.0;

    // Monotonic condition
    //dk = (double) sign(deltak) * std::abs(dk);
    //dk1 = (double) sign(deltak) * std::abs(dk1);

    //monotonic but more restrictive
    if (deltak > 0) {
            if (dk < 0) dk = 0;
            if (dk1 < 0) dk1 = 0;
        } else if (deltak < 0) {
            if (dk > 0) dk = 0;
            if (dk1 > 0) dk1 = 0;
        }


    double a0 = f[1];
    double a1 = dk;
    double a2 = 3.0 * deltak - 2.0 * dk - dk1;
    double a3 = dk + dk1 - deltak;

    double collapse = a3 * pow(t, 3) + a2 * pow(t, 2) + a1 * (t) + a0;
    return collapse;
}

Vector3d Simulation::getVel(Vector3d &pos) {
    Vector3d vel;
    vel[0] = getVelAxis(pos, 0);
    vel[1] = getVelAxis(pos, 1);
    vel[2] = getVelAxis(pos, 2);
    return vel;
};

double Simulation::getVelAxis(Vector3d &pos, int axis) {
    switch (axis) {
        case 0:
            return cubicInterpolator(pos - voxelSize * Vector3d(0.0, -0.5, -0.5), VELOCITY, 0);
            break;
        case 1:
            return cubicInterpolator(pos - voxelSize * Vector3d(-0.5, 0.0, -0.5), VELOCITY, 1);
            break;
        case 2:
            return cubicInterpolator(pos - voxelSize * Vector3d(-0.5, -0.5, 0.0), VELOCITY, 2);
            break;
    }
};

Vector4i Simulation::clampIndex(Vector4i index, int axis) {
    Vector4i clamped;
    for (int i = 0; i < 4; i++) {
        switch (axis) {
            case 0:
                clamped[i] = std::min(std::max(index[i],0), SIZE_X - 1);
                break;
            case 1:
                clamped[i] = std::min(std::max(index[i],0), SIZE_Y - 1);
                break;
            case 2:
                clamped[i] = std::min(std::max(index[i],0), SIZE_Z - 1);
                break;
        }
    }
    return clamped;
}

Vector3d Simulation::clampPos(Vector3d pos) {
    // TODO: do we need to account for 0.5 on either end?
    double x = std::min(std::max(0.0, pos[0]), SIZE_X * voxelSize);
    double y = std::min(std::max(0.0, pos[1]), SIZE_Y * voxelSize);
    double z = std::min(std::max(0.0, pos[2]), SIZE_Z * voxelSize);

    return Vector3d(x, y, z);
}

double Simulation::zero(double x) {
    if (abs(x) < epsilon) {
        return 0.0;
    } else {
        return x;
    }
}

Vector3d Simulation::zero(Vector3d x) {
    return Vector3d(zero(x[0]), zero(x[1]), zero(x[2]));
}


double Simulation::clampUnit(double x) {
    return (std::min(std::max(0.0, x), 1.0));
}

Vector3d Simulation::clampUnit(Vector3d x) {
    return Vector3d(clampUnit(x[0]), clampUnit(x[1]), clampUnit(x[2]));
}


int Simulation::sign(double x) {
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

double Simulation::totalDensity() {
    double sum = 0.0;
    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                sum += grid->grid[i][j][k]->density;
            }
        }
    }
    return sum;
}

