#include "macgrid.h"
#include <Eigen/Sparse>

MACgrid::MACgrid()
{
    initCellData();
    initFaceData();
    buildA();
}

void MACgrid::initCellData()
{
    for (int i  = 0; i<SIZE_X; i++)
    {
        std::vector<std::vector<double>> vxy;
        std::vector<std::vector<double>> vyy;
        std::vector<std::vector<double>> vzy;

        std::vector<std::vector<double>> vortxy;
        std::vector<std::vector<double>> vortyy;
        std::vector<std::vector<double>> vortzy;

        std::vector<std::vector<double>> dy;
        std::vector<std::vector<double>> ty;
        std::vector<std::vector<double>> ndy;
        std::vector<std::vector<double>> nty;
        for (int j = 0; j<SIZE_Y; j++)
        {
            std::vector<double> vxz;
            std::vector<double> vyz;
            std::vector<double> vzz;

            std::vector<double> vortxz;
            std::vector<double> vortyz;
            std::vector<double> vortzz;

            std::vector<double> dz;
            std::vector<double> tz;
            std::vector<double> ndz;
            std::vector<double> ntz;
            for (int k = 0; k< SIZE_Z; k++)
            {
                vxz.push_back(INIT_X_VEL);
                vyz.push_back(INIT_Y_VEL);
                vzz.push_back(INIT_Z_VEL);

                vortxz.push_back(0.0);
                vortyz.push_back(0.0);
                vortzz.push_back(0.0);

                // CREATING A BLOCK OF DENSITY 1 VOXELS TO START
                if (i>=START_INDEX&&i<=END_INDEX&&j>=START_INDEX&&j<=END_INDEX&&k>=START_INDEX&&k<=END_INDEX)
                {
                    dz.push_back(1.0);
                } else {
                    dz.push_back(0.0);
                }
                tz.push_back(T_AMBIENT);
                ndz.push_back(0);
                ntz.push_back(T_AMBIENT);
            }
            vxy.push_back(vxz);
            vyy.push_back(vyz);
            vzy.push_back(vzz);

            vortxy.push_back(vortxz);
            vortyy.push_back(vortyz);
            vortzy.push_back(vortzz);

            dy.push_back(dz);
            ty.push_back(tz);
            ndy.push_back(ndz);
            nty.push_back(ntz);
        }
        center_vel_x.push_back(vxy);
        center_vel_y.push_back(vyy);
        center_vel_z.push_back(vzy);

        vorticity_x.push_back(vortxy);
        vorticity_y.push_back(vortyy);
        vorticity_z.push_back(vortzy);
        vorticity.push_back(vortxy);

        vorticity_grad_x.push_back(vortxy);
        vorticity_grad_y.push_back(vortyy);
        vorticity_grad_z.push_back(vortzy);

        vcf_x.push_back(vortxy);
        vcf_y.push_back(vortyy);
        vcf_z.push_back(vortzy);

        density.push_back(dy);
        temperature.push_back(ty);
        pressure.push_back(dy);
        divergence.push_back(dy);
        next_density.push_back(ndy);
        next_temperature.push_back(nty);
    }
}

void MACgrid::initFaceData()
{
    //X_FACES
    for (int i = 0; i<SIZE_X+1; i++)
    {
        std::vector<std::vector<double>> vxy;
        std::vector<std::vector<double>> nvxy;
        for (int j = 0; j<SIZE_Y; j++)
        {
            std::vector<double> vxz;
            std::vector<double> nvxz;
            for (int k=0; k<SIZE_Z; k++)
            {
                vxz.push_back(INIT_X_VEL);
                nvxz.push_back(0);
            }
            vxy.push_back(vxz);
            nvxy.push_back(nvxz);
        }
        face_vel_x.push_back(vxy);
        next_face_vel_x.push_back(nvxy);
    }
    //Y_FACES
    for (int i = 0; i<SIZE_X; i++)
    {
        std::vector<std::vector<double>> vyy;
        std::vector<std::vector<double>> nvyy;
        for (int j = 0; j<SIZE_Y+1; j++)
        {
            std::vector<double> vyz;
            std::vector<double> nvyz;
            for (int k=0; k<SIZE_Z; k++)
            {
                vyz.push_back(INIT_Y_VEL);
                nvyz.push_back(0);
            }
            vyy.push_back(vyz);
            nvyy.push_back(nvyz);
        }
        face_vel_y.push_back(vyy);
        next_face_vel_y.push_back(nvyy);
    }
    //Z_FACES
    for (int i = 0; i<SIZE_X; i++)
    {
        std::vector<std::vector<double>> vzy;
        std::vector<std::vector<double>> nvzy;
        for (int j = 0; j<SIZE_Y; j++)
        {
            std::vector<double> vzz;
            std::vector<double> nvzz;
            for (int k=0; k<SIZE_Z+1; k++)
            {
                vzz.push_back(INIT_Z_VEL);
                nvzz.push_back(0);
            }
            vzy.push_back(vzz);
            nvzy.push_back(nvzz);
        }
        face_vel_z.push_back(vzy);
        next_face_vel_z.push_back(nvzy);
    }
}

void MACgrid::buildA()
{
    Eigen::SparseMatrix<double, Eigen::RowMajor> A(SIZE_CUBE, SIZE_CUBE);
    A.setZero();
    std::vector<Eigen::Triplet<double>> t;

    for (int i = 0; i<SIZE_X; i++)
    {
        for (int j = 0; j<SIZE_Y; j++)
        {
            for (int k=0; k<SIZE_Z; k++)
            {
                int total_neighbors = 0;
                if (i-1 >=0)
                {
                    total_neighbors++;
                    t.push_back(Eigen::Triplet(INDEX(i,j,k), INDEX(i-1,j,k),-1.0));
                }
                if (i+1 < SIZE_X) {
                    total_neighbors++;
                    t.push_back(Eigen::Triplet(INDEX(i,j,k), INDEX(i+1,j,k),-1.0));
                }
                if (j-1 >= 0) {
                    total_neighbors++;
                    t.push_back(Eigen::Triplet(INDEX(i,j,k), INDEX(i,j-1,k),-1.0));
                }
                if (j+1 < SIZE_Y) {
                    total_neighbors++;
                    t.push_back(Eigen::Triplet(INDEX(i,j,k), INDEX(i,j+1,k),-1.0));
                }
                if (k-1 >= 0) {
                    total_neighbors++;
                    t.push_back(Eigen::Triplet(INDEX(i,j,k), INDEX(i,j,k-1),-1.0));
                }
                if (k+1 < SIZE_Z) {
                    total_neighbors++;
                    t.push_back(Eigen::Triplet(INDEX(i,j,k), INDEX(i,j,k+1),-1.0));
                }
                // Set the diagonal:
                t.push_back(Eigen::Triplet(INDEX(i,j,k), INDEX(i,j,k),1.0*total_neighbors));
            }
        }
    }
    std::cout<<"BEFORE TRIPLETS"<<std::endl;
    A.setFromTriplets(t.begin(), t.end());
    std::cout<<"AFTER TRIPLETS"<<std::endl;
    solver.compute(A);
}

double MACgrid::getVal(DATA_TYPE type, int i, int j, int k) {
    switch (type) {
        case VELOCITY_X:
            if (i < 0 || i > SIZE_X) return 0.0;

            if (j < 0) j = 0;
            if (j > SIZE_Y - 1) j = SIZE_Y - 1;
            if (k < 0) k = 0;
            if (k > SIZE_Z - 1) k = SIZE_Z - 1;

            return this->face_vel_x[i][j][k];
            break;
        case VELOCITY_Y:
            if (j < 0 || j > SIZE_Y) return 0.0;

            if (i < 0) i = 0;
            if (i > SIZE_X - 1) i = SIZE_X - 1;
            if (k < 0) k = 0;
            if (k > SIZE_Z - 1) k = SIZE_Z - 1;

            return this->face_vel_y[i][j][k];
            break;
        case VELOCITY_Z:
            if (k < 0 || k > SIZE_Z) return 0.0;

            if (i < 0) i = 0;
            if (i > SIZE_X - 1) i = SIZE_X - 1;
            if (j < 0) j = 0;
            if (j > SIZE_Y - 1) j = SIZE_Y - 1;

            return this->face_vel_z[i][j][k];
            break;
        case DENSITY:
            if (i < 0 || j < 0 || k < 0 ||
               i > SIZE_X - 1 ||
               j > SIZE_Y - 1 ||
               k > SIZE_Z - 1) return 0.0;

            return this->density[i][j][k];
            break;
        case TEMPERATURE:
            if (i < 0 || j < 0 || k < 0 ||
               i > SIZE_X - 1 ||
               j > SIZE_Y - 1 ||
               k > SIZE_Z - 1) return 0.0;

            return this->temperature[i][j][k];
            break;
        case PRESSURE:
            if (i < 0 || j < 0 || k < 0 ||
               i > SIZE_X - 1 ||
               j > SIZE_Y - 1 ||
               k > SIZE_Z - 1) return 0.0;

            return this->pressure[i][j][k];
            break;
        case CENTER_VEL_X:
            if (i < 0 || j < 0 || k < 0 ||
                i > SIZE_X - 1 ||
                j > SIZE_Y - 1 ||
                k > SIZE_Z - 1) return 0.0;

            return this->center_vel_x[i][j][k];
            break;
        case CENTER_VEL_Y:
            if (i < 0 || j < 0 || k < 0 ||
                i > SIZE_X - 1 ||
                j > SIZE_Y - 1 ||
                k > SIZE_Z - 1) return 0.0;

            return this->center_vel_y[i][j][k];
            break;
        case CENTER_VEL_Z:
            if (i < 0 || j < 0 || k < 0 ||
                i > SIZE_X - 1 ||
                j > SIZE_Y - 1 ||
                k > SIZE_Z - 1) return 0.0;

            return this->center_vel_z[i][j][k];
            break;
        case VORTICITY:
            if (i < 0 || j < 0 || k < 0 ||
                i > SIZE_X - 1 ||
                j > SIZE_Y - 1 ||
                k > SIZE_Z - 1) return 0.0;

            return this->vorticity[i][j][k];
            break;
        case VCF_X:
            if (i < 0 || j < 0 || k < 0 ||
                i > SIZE_X - 1 ||
                j > SIZE_Y - 1 ||
                k > SIZE_Z - 1) return 0.0;

            return this->vcf_x[i][j][k];
            break;
        case VCF_Y:
            if (i < 0 || j < 0 || k < 0 ||
                i > SIZE_X - 1 ||
                j > SIZE_Y - 1 ||
                k > SIZE_Z - 1) return 0.0;

            return this->vcf_y[i][j][k];
            break;
        case VCF_Z:
            if (i < 0 || j < 0 || k < 0 ||
                i > SIZE_X - 1 ||
                j > SIZE_Y - 1 ||
                k > SIZE_Z - 1) return 0.0;

            return this->vcf_z[i][j][k];
            break;
    }
}

void MACgrid::render(std::string number)
{
    std::vector<float> d1d(SIZE_Z);
    std::vector<std::vector<float>> d2d(SIZE_Y);
    std::vector<std::vector<std::vector<float>>> densities(SIZE_X);

    for (int i = 0; i < SIZE_X; i++) {
        for (int j = 0; j < SIZE_Y; j++) {
            for (int k = 0; k < SIZE_Z; k++) {
                d1d[k] = density[i][j][k];
            }
            d2d[j] = d1d;
        }
        densities[i] = d2d;
    }
    std::string filepathFolders = "C:\\Users\\annaf\\course\\cs2240\\final\\smoke-signal\\src\\rendering\\render-5-12.1\\";
    std::string fileName = "SMOKE_";
    std::string fileType = ".vol";
    std::string filePath_FULL = filepathFolders+fileName+number+fileType;
    Rendering::write_vol(filePath_FULL, densities);
}
