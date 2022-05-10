#include "simulation.h"
#include "constants.h"


using namespace Eigen;

//flag every single voxel which intersects with the object as being occupied,
//set velocity to be same as immersed object, set temperature same as well
//DENSITY ZERO in intersecting voxel... boundary voxel desity = closest unoccupied voxel

//// smoke emission! pass in a list of indices of emitting voxels and it will set their density to 1 and upward velocity to 50
void Simulation::emitSmoke(std::vector<Eigen::Vector3i> indices) {
    for (auto voxel_index : indices) {
        // grid->grid[voxel_index[0]][voxel_index[1]][voxel_index[2]]->density = 1.0;
        // grid->faces[1][voxel_index[0]][voxel_index[1]][voxel_index[2]]->vel = 1.0;
        // grid->faces[2][voxel_index[0]][voxel_index[1]][voxel_index[2]]->vel = 1.0;
    }
}

void Simulation::updateVelocities() {
    // Update face velocities with forces
    for (int i = 0; i < gridSize + 1; i++) {
        for (int j = 0; j < gridSize + 1; j++) {
            for(int k = 0; k < gridSize + 1; k++) {
                // Edge cases
                if (i == 0 || i == gridSize) {
                    grid->faces[0][i][j][k]->vel = 0.0;
                } else if (j < gridSize  && k < gridSize ) {
                    grid->faces[0][i][j][k]->vel += (grid->grid[i][j][k]->force[0] + grid->grid[i - 1][j][k]->force[0]) * timestep / 2.0;
                }

                if (j == 0 || j == gridSize) {
                    grid->faces[1][i][j][k]->vel = 0.0;
                } else if (i < gridSize  && k < gridSize ) {
                    grid->faces[1][i][j][k]->vel += (grid->grid[i][j][k]->force[1] + grid->grid[i][j - 1][k]->force[1]) * timestep / 2.0;
                }

                if (k == 0 || k == gridSize) {
                    grid->faces[2][i][j][k]->vel = 0.0;
                } else if (i < gridSize  && j < gridSize ) {
                    grid->faces[2][i][j][k]->vel += (grid->grid[i][j][k]->force[2] + grid->grid[i][j][k - 1]->force[2]) * timestep / 2.0;
                }
            }
        }
    }
}

void Simulation::addForces() {
    // Calculate the vorticities for each cell
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for(int k = 0; k < gridSize; k++) {
                // Border Cases
                if (i == 0 || j == 0 || k == 0 || i == gridSize - 1 || j == gridSize - 1 || k == gridSize - 1) {
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
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for(int k = 0; k < gridSize; k++) {
                 // Add vertical buoyancy force to z axis where z = 1 is up (eqn. 8)
                 grid->grid[i][j][k]->force = Vector3d();
                 grid->grid[i][j][k]->force[0] = 0.0;
                 grid->grid[i][j][k]->force[1] = -1.0 * alpha * grid->grid[i][j][k]->density + beta * (grid->grid[i][j][k]->temp - Tambient);
                 grid->grid[i][j][k]->force[2] = 0.0;

                // Border Cases
                if (i == 0 || j == 0 || k == 0 || i == gridSize - 1 || j == gridSize - 1 || k == gridSize - 1) {
                    continue;
                }

                // Gradient of vorticity
                double g_x = zero((grid->grid[i + 1][j][k]->vort.norm() - grid->grid[i - 1][j][k]->vort.norm())) / (2.0 * voxelSize);
                double g_y = zero((grid->grid[i][j + 1][k]->vort.norm() - grid->grid[i][j - 1][k]->vort.norm())) / (2.0 * voxelSize);
                double g_z = zero((grid->grid[i][j][k + 1]->vort.norm() - grid->grid[i][j][k - 1]->vort.norm())) / (2.0 * voxelSize);

                // Normalized vorticity location vector
                Eigen::Vector3d N = Eigen::Vector3d(g_x, g_y, g_z).normalized();

                // Add confinement force to voxel forces
                Eigen::Vector3d f_c = zero(epsilon * voxelSize * N.cross(grid->grid[i][j][k]->vort));
                grid->grid[i][j][k]->force += f_c;
            }
        }
    }
}

//// solve for advection term (in eqn 3)
///// updateAdvection (semi Langrangian scheme for advection in eqn 3)
///// builds new grid from precomputed
///// trace midpoints of each face through field
///// new vels interpolated--> transferred to face cells of origin
///// ** boundary (clip to furthest boundary point fig 2)
void Simulation::advectVelocity() {
    for (int i = 0; i < gridSize + 1; i++) {
        for (int j = 0; j < gridSize + 1; j++) {
            for (int k = 0; k < gridSize + 1; k++) {
                Vector3d newVel;
                // Iterate 3 times for x, y, and z faces
                for (int c = 0; c < 3; c++) {
                    int ii = i;
                    if (i == gridSize) {
                        ii = i-1;
                    }
                    int jj = j;
                    if (j == gridSize) {
                        jj = j-1;
                    }
                    int kk = k;
                    if (k == gridSize) {
                        kk = k-1;
                    }

                    Eigen::Vector3d pos;
                    //Eigen::Vector3d vel = grid->grid[ii][jj][kk]->centerVel;

                    switch(c) {
                    case 0: //x
                        pos = Vector3d((i + 0.5),j,k) * voxelSize;
                        break;
                    case 1: //y
                        pos = Vector3d(i,(j + 0.5),k) * voxelSize;
                        break;
                    case 2: //z
                        pos = Vector3d(i,j,(k + 0.5)) * voxelSize;
                        break;
                    }
                    double velx = cubicInterpolator(pos, INTERP_TYPE::VELOCITY, 0);
                    double vely = cubicInterpolator(pos, INTERP_TYPE::VELOCITY, 1);
                    double velz = cubicInterpolator(pos, INTERP_TYPE::VELOCITY, 2);
                    Eigen::Vector3d vel = Vector3d(velx, vely, velz);//grid->grid[ii][jj][kk]->centerVel;
                    Eigen::Vector3d midPos = pos - (timestep * vel)/2;
                    double midVelx = cubicInterpolator(midPos, INTERP_TYPE::VELOCITY, 0);
                    double midVely = cubicInterpolator(midPos, INTERP_TYPE::VELOCITY, 1);
                    double midVelz = cubicInterpolator(midPos, INTERP_TYPE::VELOCITY, 2);
                    Eigen::Vector3d midVel = Vector3d(midVelx, midVely, midVelz);
                    pos -= midVel*timestep;
                    newVel[c] = cubicInterpolator(pos, INTERP_TYPE::VELOCITY, c);
                    grid->faces[c][i][j][k]->nextVel = newVel[c];
                }
            }
        }
    }

    for (int i = 0; i < gridSize + 1; i++) {
        for (int j = 0; j < gridSize + 1; j++) {
            for (int k = 0; k < gridSize + 1; k++) {
                for (int c = 0; c < 3; c++) {
                    grid->faces[c][i][j][k]->vel = grid->faces[c][i][j][k]->nextVel;
                }
            }
        }
    }
}

void Simulation::advectTemp() {
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for (int k = 0; k < gridSize; k++) {
                double newTemp;

                Eigen::Vector3d pos = Vector3d(i, j, k) * voxelSize;
                double velx = cubicInterpolator(pos, INTERP_TYPE::VELOCITY, 0);
                double vely = cubicInterpolator(pos, INTERP_TYPE::VELOCITY, 1);
                double velz = cubicInterpolator(pos, INTERP_TYPE::VELOCITY, 2);
                Eigen::Vector3d vel = Vector3d(velx, vely, velz);//grid->grid[ii][jj][kk]->centerVel;
                Eigen::Vector3d midPos = pos - (timestep * vel)/2;
                double midVelx = cubicInterpolator(midPos, INTERP_TYPE::VELOCITY, 0);
                double midVely = cubicInterpolator(midPos, INTERP_TYPE::VELOCITY, 1);
                double midVelz = cubicInterpolator(midPos, INTERP_TYPE::VELOCITY, 2);
                Eigen::Vector3d midVel = Vector3d(midVelx, midVely, midVelz);
                pos -= midVel*timestep;
//                Eigen::Vector3d vel;
//                pos -= timestep * grid->grid[i][j][k]->centerVel;
                newTemp = cubicInterpolator(pos, INTERP_TYPE::TEMPERATURE, 0);
                grid->grid[i][j][k]->nextTemp = newTemp;
            }
        }
    }

    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for (int k = 0; k < gridSize; k++) {
                grid->grid[i][j][k]->temp = grid->grid[i][j][k]->nextTemp;
            }
        }
    }
}

void Simulation::advectDensity() {
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for (int k = 0; k < gridSize; k++) {
                double newDensity;
                Eigen::Vector3d pos = Vector3d(i, j, k) * voxelSize;
                double velx = cubicInterpolator(pos, INTERP_TYPE::VELOCITY, 0);
                double vely = cubicInterpolator(pos, INTERP_TYPE::VELOCITY, 1);
                double velz = cubicInterpolator(pos, INTERP_TYPE::VELOCITY, 2);
                Eigen::Vector3d vel = Vector3d(velx, vely, velz);//grid->grid[ii][jj][kk]->centerVel;
                Eigen::Vector3d midPos = pos - (timestep * vel)/2;
                double midVelx = cubicInterpolator(midPos, INTERP_TYPE::VELOCITY, 0);
                double midVely = cubicInterpolator(midPos, INTERP_TYPE::VELOCITY, 1);
                double midVelz = cubicInterpolator(midPos, INTERP_TYPE::VELOCITY, 2);
                Eigen::Vector3d midVel = Vector3d(midVelx, midVely, midVelz);
                pos -= midVel*timestep;
//                Eigen::Vector3d pos = Vector3d(i, j, k) * voxelSize;
//                pos -= timestep * grid->grid[i][j][k]->centerVel;
                newDensity = cubicInterpolator(pos, INTERP_TYPE::DENSITY, 0);
                grid->grid[i][j][k]->nextDensity = newDensity;
            }
        }
    }

    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for (int k = 0; k < gridSize; k++) {
                grid->grid[i][j][k]->density = grid->grid[i][j][k]->nextDensity;
            }
        }
    }
}

void Simulation::solvePressure() {
    std::vector<Triplet<double>> t;
    // TODO: try other solvers - SparseLDLT
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> solver;
    //Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> solver;
    Eigen::SparseMatrix<double, Eigen::RowMajor> A(cubeSize, cubeSize);
    A.setZero();
    Eigen::VectorXd b(cubeSize);
    b.setZero();
    Eigen::VectorXd p(cubeSize);
    p.setZero();

    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for (int k = 0; k < gridSize; k++) {

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

                if (i < gridSize - 1) {
                    b[INDEX(i, j, k)] += grid->faces[0][i + 1][j][k]->vel;
                    neighbors += 1.0;
                    t.push_back(Eigen::Triplet(INDEX(i, j, k), INDEX(i + 1, j, k), 1.0));
                }

                if (j < gridSize - 1) {
                    b[INDEX(i, j, k)] += grid->faces[1][i][j + 1][k]->vel;
                    neighbors += 1.0;
                    t.push_back(Eigen::Triplet(INDEX(i, j, k), INDEX(i, j + 1, k), 1.0));
                }

                if (k < gridSize - 1) {
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
    // TODO: check pressure
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for (int k = 0; k < gridSize; k++) {
                // TODO: do the timestep and voxelSize cancel out?
                if (i < gridSize - 1) {
                    grid->faces[0][i + 1][j][k]->vel -= (p[INDEX(i + 1, j, k)] - p[INDEX(i, j, k)]) * timestep / voxelSize;
                }

                if (j < gridSize - 1) {
                    grid->faces[1][i][j + 1][k]->vel -= (p[INDEX(i, j + 1, k)] - p[INDEX(i, j, k)]) * timestep / voxelSize;
                }

                if (k < gridSize - 1) {
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

    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            for(int k = 0; k < gridSize; k++) {
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
    Vector3d posClamped = Vector3d(clamp(position[0]), clamp(position[1]), clamp(position[2]));
    Vector3i indexCast;
    Vector3d percentage;

    // Find the voxel index based on clamped position
    for (int c = 0; c < 3 ; c++) {
        indexCast[c] = (int) (posClamped[c] / voxelSize);
        percentage[c] = posClamped[c] / voxelSize - (double) indexCast[c];
        assert (percentage[c] < 1.0 && percentage[c] >= 0);
    }



    // Compute indices for the interpolation
    Vector4i x_indices = clampIndex(Vector4i{indexCast[0] - 1, indexCast[0], indexCast[0] + 1, indexCast[0] + 2});
    Vector4i y_indices = clampIndex(Vector4i{indexCast[1] - 1, indexCast[1], indexCast[1] + 1, indexCast[1] + 2});
    Vector4i z_indices = clampIndex(Vector4i{indexCast[2] - 1, indexCast[2], indexCast[2] + 1, indexCast[2] + 2});

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

Vector4i Simulation::clampIndex(Vector4i(index)) {
    Vector4i clamped;
    for (int i = 0; i < 4; i++) {
        clamped[i] = std::min(std::max(index[i],0), gridSize - 1);
    }
    return clamped;
}

double Simulation::clamp(double input) {
    return (std::min(std::max(0.0, input), 1.0 * gridSize * voxelSize - epsilon));
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
/*
double Simulation::cubicInterpolator(Vector3d position, INTERP_TYPE var, int axis)
{

    Vector3d pos = Vector3d(position[0] - 0.5*voxelSize, position[1] - 0.5*voxelSize, position[2] - 0.5*voxelSize);

        int i = (int) (pos[0]/voxelSize);
        int j = (int) (pos[1]/voxelSize);
        int k = (int) (pos[2]/voxelSize);

        double scale = 1.0/voxelSize;
        double fractx = scale*(pos[0] - i*voxelSize);
        double fracty = scale*(pos[1] - j*voxelSize);
        double fractz = scale*(pos[2] - k*voxelSize);

    #ifdef _DEBUG
        assert (fractx < 1.0 && fractx >= 0);
       assert (fracty < 1.0 && fracty >= 0);
       assert (fractz < 1.0 && fractz >= 0);
    #endif



        double t[4][4];
        double u[4];
        double f;
    #define ONE 1

        for (int x = -1; x <= 2; x++) {
            for (int y = -1; y <= 2; y++) {
                switch (var) {
                    case INTERP_TYPE::DENSITY:
                    t[x+ONE][y+ONE] = CINT( grid->grid[i+x][j+y][k-1]->density, grid->grid[i+x][j+y][k+0]->density, grid->grid[i+x][j+y][k+1]->density, grid->grid[i+x][j+y][k+2]->density, fractz );
                        break;
                    case INTERP_TYPE::TEMPERATURE:
                    t[x+ONE][y+ONE] = CINT( grid->grid[i+x][j+y][k-1]->temp, grid->grid[i+x][j+y][k+0]->temp, grid->grid[i+x][j+y][k+1]->temp, grid->grid[i+x][j+y][k+2]->temp, fractz );
                        break;
                    case INTERP_TYPE::VELOCITY:
                    t[x+ONE][y+ONE] = CINT( grid->faces[axis][i+x][j+y][k-1]->vel, grid->faces[axis][i+x][j+y][k+0]->vel, grid->faces[axis][i+x][j+y][k+1]->vel, grid->faces[axis][i+x][j+y][k+2]->vel, fractz );
                        break;
                }
            }
        }

        for (int x = -1; x <= 2; x++) {
            u[x+ONE] = CINT( t[x+ONE][-1+ONE], t[x+ONE][0+ONE], t[x+ONE][1+ONE], t[x+ONE][2+ONE], fracty );
        }
        f = CINT( u[-1+ONE], u[0+ONE], u[1+ONE], u[2+ONE], fractx );
    #undef ONE
        return f;
}

double Simulation::CINT(double q_i_minus_1, double q_i, double q_i_plus_1, double q_i_plus_2, double x) {

    // The slopes:
    double d_i = (q_i_plus_1 - q_i_minus_1) / 2.0;
    double d_i_plus_1 = (q_i_plus_2 - q_i) / 2.0;

    // Delta q:
    double delta_q = q_i_plus_1 - q_i;

    // Restrict the slopes:
    if (delta_q > 0) {
        if (d_i < 0) d_i = 0;
        if (d_i_plus_1 < 0) d_i_plus_1 = 0;
    } else if (delta_q < 0) {
        if (d_i > 0) d_i = 0;
        if (d_i_plus_1 > 0) d_i_plus_1 = 0;
    }

    // The Hermite cubic:
    double q_x = q_i + d_i * x + (3.0 * delta_q - 2.0 * d_i - d_i_plus_1) * (x * x) + (-2.0 * delta_q + d_i + d_i_plus_1) * (x * x * x);

    // Done:
    return q_x;

}
*/




