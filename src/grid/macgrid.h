#ifndef MACGRID_H
#define MACGRID_H
#include <Eigen/StdVector>;
#include "constants.h";

using namespace Eigen;
class MACgrid
{
public:
    MACgrid();
    void initCellData();
    void initFaceData();

    // VOXEL DATA
    std::vector<std::vector<std::vector<double>>> center_vel_x;
    std::vector<std::vector<std::vector<double>>> center_vel_y;
    std::vector<std::vector<std::vector<double>>> center_vel_z;

    std::vector<std::vector<std::vector<double>>> vorticity_x;
    std::vector<std::vector<std::vector<double>>> vorticity_y;
    std::vector<std::vector<std::vector<double>>> vorticity_z;
    std::vector<std::vector<std::vector<double>>> vorticity;


    std::vector<std::vector<std::vector<double>>> vorticity_grad_x;
    std::vector<std::vector<std::vector<double>>> vorticity_grad_y;
    std::vector<std::vector<std::vector<double>>> vorticity_grad_z;

    std::vector<std::vector<std::vector<double>>> vcf_x;
    std::vector<std::vector<std::vector<double>>> vcf_y;
    std::vector<std::vector<std::vector<double>>> vcf_z;

    std::vector<std::vector<std::vector<double>>> temperature;
    std::vector<std::vector<std::vector<double>>> density;
    std::vector<std::vector<std::vector<double>>> pressure;


    //nextDATA
    std::vector<std::vector<std::vector<double>>> next_temperature;
    std::vector<std::vector<std::vector<double>>> next_density;

    //FACE DATA
    std::vector<std::vector<std::vector<double>>> face_vel_x;
    std::vector<std::vector<std::vector<double>>> face_vel_y;
    std::vector<std::vector<std::vector<double>>> face_vel_z;

    //nextDATA
    std::vector<std::vector<std::vector<double>>> next_face_vel_x;
    std::vector<std::vector<std::vector<double>>> next_face_vel_y;
    std::vector<std::vector<std::vector<double>>> next_face_vel_z;

};

#endif // MACGRID_H
