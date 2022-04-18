#ifndef TET_H
#define TET_H

#include <Eigen/Dense>
#include <iostream>
#include "node.h"
#include "face.h"
using namespace Eigen;

const double _lambda = 10; //incompressibility for the whole material
const double _mu = 20; //rigidity for the whole material
const double _phi = 5; //coefficients of viscosity
const double _psi = 5;
const double _rho = 1200.0; //density


class Tet {
public:
    Tet(std::shared_ptr<Node> n_1, std::shared_ptr<Node> n_2, std::shared_ptr<Node> n_3, std::shared_ptr<Node> n_4);
    std::vector<std::shared_ptr<Face>> get_faces();
    void update_forces(bool mid);
    void push(Vector3d f);

private:
    // Strain and stress
    Matrix3d m_strain;
    Matrix3d m_stress;
    Matrix3d m_strain_rate;
    Matrix3d m_beta; // Deformation constant
    Matrix3d m_deformation_gradient;
    Matrix3d m_velocity_gradient;
    double m_mass;

    // Tet nodes and faces
    std::shared_ptr<Node> n_1;
    std::shared_ptr<Node> n_2;
    std::shared_ptr<Node> n_3;
    std::shared_ptr<Node> n_4;

    std::shared_ptr<Face> f_1;
    std::shared_ptr<Face> f_2;
    std::shared_ptr<Face> f_3;
    std::shared_ptr<Face> f_4;

    void update_deformation_gradient(bool mid);
    void update_velocity_gradient(bool mid);
    void update_strain();
    void update_stress();
    void update_strain_rate();
};

#endif // TET_H
