#ifndef MACGRID_H
#define MACGRID_H
#include <Eigen/StdVector>
#include <Eigen/Sparse>
#include "constants.h"
#include "rendering/rendering.h"

using namespace Eigen;
class MACgrid
{
public:
    MACgrid();
    void initCellData();
    void initFaceData();
    void buildA();
    double getVal(DATA_TYPE type, int i, int j, int k);

    void render(std::string number);

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
    std::vector<std::vector<std::vector<double>>> divergence;

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

    //for pressure solver
//    SparseMatrix<double, Eigen::RowMajor> A;
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> solver;

};

#endif // MACGRID_H
