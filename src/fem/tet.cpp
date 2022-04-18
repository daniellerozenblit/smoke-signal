#include "tet.h"

#include "graphics/Shader.h"

Tet::Tet(std::shared_ptr<Node> n_1, std::shared_ptr<Node> n_2, std::shared_ptr<Node> n_3, std::shared_ptr<Node> n_4)
    : n_1(n_1),
      n_2(n_2),
      n_3(n_3),
      n_4(n_4)
{
    // Initialize deformation constant (beta)
    Vector3d col_1 = n_1->get_loc() - n_4->get_loc();
    Vector3d col_2 = n_2->get_loc() - n_4->get_loc();
    Vector3d col_3 = n_3->get_loc() - n_4->get_loc();

    m_beta << col_1[0], col_2[0], col_3[0],
              col_1[1], col_2[1], col_3[1],
              col_1[2], col_2[2], col_3[2];

    m_beta = m_beta.inverse().eval();

    // Create faces from nodes
    f_1 = std::shared_ptr<Face>(new Face(n_4, n_2, n_3));
    f_2 = std::shared_ptr<Face>(new Face(n_3, n_1, n_4));
    f_3 = std::shared_ptr<Face>(new Face(n_4, n_1, n_2));
    f_4 = std::shared_ptr<Face>(new Face(n_2, n_1, n_3));

    // Calculate tet mass
    Vector3d a = n_2->get_loc() - n_1->get_loc();
    Vector3d b = n_3->get_loc() - n_1->get_loc();
    Vector3d c = n_4->get_loc() - n_1->get_loc();

    m_mass = _rho * (1.0 / 6.0) * abs((a.cross(b)).dot(c));

    n_1->add_mass(m_mass / 4.0);
    n_2->add_mass(m_mass / 4.0);
    n_3->add_mass(m_mass / 4.0);
    n_4->add_mass(m_mass / 4.0);
}

std::vector<std::shared_ptr<Face>> Tet::get_faces() {
    return std::vector<std::shared_ptr<Face>>({f_1, f_2, f_3, f_4});
}

void Tet::update_forces(bool mid) {
    update_deformation_gradient(mid);
    update_velocity_gradient(mid);
    update_strain();
    update_strain_rate();
    update_stress();

    // Update forces on each node
    n_1->add_force(m_deformation_gradient * m_stress * f_1->get_area() * f_1->get_normal());
    n_2->add_force(m_deformation_gradient * m_stress * f_2->get_area() * f_2->get_normal());
    n_3->add_force(m_deformation_gradient * m_stress * f_3->get_area() * f_3->get_normal());
    n_4->add_force(m_deformation_gradient * m_stress * f_4->get_area() * f_4->get_normal());
}

void Tet::update_deformation_gradient(bool mid) {
    Vector3d col_1;
    Vector3d col_2;
    Vector3d col_3;

    if (mid) { // If calculating the midpoint
        col_1 = n_1->get_loc() - n_4->get_loc();
        col_2 = n_2->get_loc() - n_4->get_loc();
        col_3 = n_3->get_loc() - n_4->get_loc();
    } else { // If calculating the final step with the midpoint
        col_1 = n_1->get_loc_mid() - n_4->get_loc_mid();
        col_2 = n_2->get_loc_mid() - n_4->get_loc_mid();
        col_3 = n_3->get_loc_mid() - n_4->get_loc_mid();
    }

    Matrix3d P;

    P << col_1[0], col_2[0], col_3[0],
              col_1[1], col_2[1], col_3[1],
              col_1[2], col_2[2], col_3[2];

    m_deformation_gradient = P * m_beta;

    if (m_deformation_gradient.isIdentity()) {
        m_deformation_gradient = Matrix3d::Identity();
    }
}

void Tet::update_velocity_gradient(bool mid) {
    Vector3d col_1;
    Vector3d col_2;
    Vector3d col_3;

    if (!mid) {
        col_1 = n_1->get_vel() - n_4->get_vel();
        col_2 = n_2->get_vel() - n_4->get_vel();
        col_3 = n_3->get_vel() - n_4->get_vel();
    } else {
        col_1 = n_1->get_vel_mid() - n_4->get_vel_mid();
        col_2 = n_2->get_vel_mid() - n_4->get_vel_mid();
        col_3 = n_3->get_vel_mid() - n_4->get_vel_mid();
    }

    Matrix3d V;

    V << col_1[0], col_2[0], col_3[0],
              col_1[1], col_2[1], col_3[1],
              col_1[2], col_2[2], col_3[2];

    m_velocity_gradient = V * m_beta;

    if (m_velocity_gradient.isIdentity()) {
        m_velocity_gradient = Matrix3d::Identity();
    }
}

void Tet::update_strain() {
    Matrix3d F = m_deformation_gradient;

    m_strain = F.transpose() * F - Matrix3d::Identity();

    if (m_strain.isZero()) {
        m_strain = Matrix3d::Zero();
    }
}

void Tet::update_strain_rate() {
    Matrix3d dx = m_deformation_gradient;
    Matrix3d dv = m_velocity_gradient;

    m_strain_rate = dx.transpose() * dv + dv.transpose() * dx;

    if (m_strain_rate.isZero()) {
        m_strain_rate = Matrix3d::Zero();
    }
}

void Tet::update_stress() {
    Matrix3d elastic_stress;
    Matrix3d viscous_stress;

    elastic_stress = _lambda * Matrix3d::Identity() * m_strain.trace()
            + 2.0 * _mu * m_strain;

    viscous_stress = _phi * Matrix3d::Identity() * m_strain_rate.trace()
            + 2.0 * _psi * m_strain_rate;

    m_stress = elastic_stress + viscous_stress;

    if (m_stress.isIdentity()) {
        m_stress = Matrix3d::Identity();
    }
}

void Tet::push(Vector3d f) {
        // Scale the normal and tangential components
        n_1->set_vel(f * (0.1));
        n_2->set_vel(f* (0.1));
        n_3->set_vel(f * (0.1));
        n_4->set_vel(f * (0.1));
}
