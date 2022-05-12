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

    std::vector<std::vector<std::vector<double>>> temperature;
    std::vector<std::vector<std::vector<double>>> density;


    //FACE DATA
    std::vector<std::vector<std::vector<double>>> face_vel_x;
    std::vector<std::vector<std::vector<double>>> face_vel_y;
    std::vector<std::vector<std::vector<double>>> face_vel_z;


};

#endif // MACGRID_H
