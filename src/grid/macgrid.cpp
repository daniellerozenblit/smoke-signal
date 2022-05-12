#include "macgrid.h"

MACgrid::MACgrid()
{
    initCellData();
    initFaceData();
}

void MACgrid::initCellData()
{

    for (int i  = 0; i<SIZE_X; i++)
    {
        std::vector<std::vector<double>> vxy;
        std::vector<std::vector<double>> vyy;
        std::vector<std::vector<double>> vzy;
        std::vector<std::vector<double>> dy;
        std::vector<std::vector<double>> ty;
        std::vector<std::vector<double>> ndy;
        std::vector<std::vector<double>> nty;
        for (int j = 0; j<SIZE_Y; j++)
        {
            std::vector<double> vxz;
            std::vector<double> vyz;
            std::vector<double> vzz;
            std::vector<double> dz;
            std::vector<double> tz;
            std::vector<double> ndz;
            std::vector<double> ntz;
            for (int k = 0; k< SIZE_Z; k++)
            {
                vxz.push_back(INIT_X_VEL);
                vyz.push_back(INIT_Y_VEL);
                vzz.push_back(INIT_Z_VEL);

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
            dy.push_back(dz);
            ty.push_back(tz);
            ndy.push_back(ndz);
            nty.push_back(ntz);
        }
        center_vel_x.push_back(vxy);
        center_vel_y.push_back(vyy);
        center_vel_z.push_back(vzy);

        density.push_back(dy);
        temperature.push_back(ty);
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
