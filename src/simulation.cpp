#include "simulation.h"
#include <iostream>
#include "graphics/MeshLoader.h"
#include "graphics/sphere.h"
#include "graphics/cone.h"
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <graphics/Shader.h>

using namespace Eigen;

float tilt = 0.0;
double r = 0.5;
Vector3d center = Vector3d(0.5, 0.0, 0.0);
//std::string file = "example-meshes/sphere.mesh";

Simulation::Simulation() {
    //grid = std::make_shared<Grid>();
}

void Simulation::init()
{
    // STUDENTS: This code loads up the tetrahedral mesh in 'example-meshes/single-tet.mesh'
    //    (note: your working directory must be set to the root directory of the starter code
    //    repo for this file to load correctly). You'll probably want to instead have this code
    //    load up a tet mesh based on e.g. a file path specified with a command line argument.
    std::vector<Vector3d> vertices;
    std::vector<Vector4i> tets;

    // Mesh translation
    Affine3d t = Affine3d(Translation3d(0, 2, 0));

    // Mesh rotation
    Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(0.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(0.0, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    t.rotate(q);



    Affine3f t_f = t.cast <float> ();
    m_shape.setModelMatrix(t_f);
    initGround();
    initSphere();
}



void Simulation::update(float seconds)
{
    for (int i = 0; i < seconds / timestep; i++)
    {
        m_tetmesh->update(timestep * 10);
        m_tetmesh->collision(m_colliders);
        m_shape.setVertices(m_tetmesh->get_surface_nodes());
    }
}


