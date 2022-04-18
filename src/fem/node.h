#ifndef NODE_H
#define NODE_H

#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

const Vector3d _gravity = Vector3d(0.0, -0.1, 0.0);

class Node {
    public:
        Node(Vector3d loc);
        Vector3d get_loc();
        Vector3d get_loc_mid();
        void set_loc(Vector3d loc);
        Vector3d get_vel();
        Vector3d get_vel_mid();
        void set_vel(Vector3d vel);
        void add_mass(double m);
        double get_mass();
        void add_force(Vector3d f);
        void update(double seconds, bool mid);
    private:
        Vector3d m_loc;
        Vector3d m_vel = Vector3d(0.0, 0.0, 0.0);
        Vector3d m_loc_mid;
        Vector3d m_vel_mid = Vector3d(0.0, 0.0, 0.0);
        double m_mass = 0.0;
        Vector3d m_force = _gravity;
};

#endif // NODE_H
