#include "simulation.h"

#include <iostream>
#include "graphics/MeshLoader.h"
using namespace Eigen;

double timestep = 0.03;
float tilt = 0.0;
double r = 0.5;
Vector3d center = Vector3d(0.5, 0.0, 0.0);
std::string file = "example-meshes/sphere.mesh";

Simulation::Simulation() {
    grid = std::make_shared<Grid>();
}

void Simulation::init() {
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

    if(MeshLoader::loadTetMesh(file, vertices, tets)) {
        std::vector<std::shared_ptr<Node>> vertices_o;
        std::vector<std::shared_ptr<Tet>> tets_o;

        // Create vertex objects
        for (Vector3d v_i : vertices) {
            std::shared_ptr<Node> n = std::shared_ptr<Node>(new Node(v_i));
            vertices_o.push_back(n);
        }

        // Create tet objects
        for (Vector4i t_i : tets) {
            std::shared_ptr<Tet> t = std::shared_ptr<Tet>(new Tet(vertices_o[t_i[0]], vertices_o[t_i[1]], vertices_o[t_i[2]], vertices_o[t_i[3]]));
            tets_o.push_back(t);
        }

        m_tetmesh = std::shared_ptr<Mesh>(new Mesh(vertices_o, tets_o, t.matrix()));

        std::vector<Vector3i> surface_faces = m_tetmesh->get_surface_faces();
        std::vector<Vector3d> surface_vertices = m_tetmesh->get_surface_nodes();

        m_shape.init(surface_vertices, surface_faces);
    }

    Affine3f t_f = t.cast <float> ();
    m_shape.setModelMatrix(t_f);
    initGround();
    initSphere();
}

void Simulation::update(float seconds) {
    for (int i = 0; i < seconds / timestep; i++) {
        m_tetmesh->update(timestep * 10);
        m_tetmesh->collision(m_colliders);
        m_shape.setVertices(m_tetmesh->get_surface_nodes());
    }



}

void Simulation::draw(Shader *shader) {
    m_shape.draw(shader);
    m_ground.draw(shader);
    m_sphere.draw(shader);
}

void Simulation::toggleWire() {
    m_shape.toggleWireframe();
}

void Simulation::initGround() {
    // Ground rotation
    Affine3d t = Affine3d(Translation3d(0.0, 0.0, 0.0));
    Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(0.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(tilt, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    t.rotate(q);
    Affine3f t_f = t.cast <float> ();
    m_ground.setModelMatrix(t_f);

    std::vector<Vector3d> groundVerts;
    std::vector<Vector3i> groundFaces;

    groundVerts.emplace_back(-10, 0, -10);
    groundVerts.emplace_back(-10, 0, 10);
    groundVerts.emplace_back(10, 0, 10);
    groundVerts.emplace_back(10, 0, -10);

    groundFaces.emplace_back(0, 1, 2);
    groundFaces.emplace_back(0, 2, 3);

    m_ground.init(groundVerts, groundFaces);

    // Add ground collider with normal based on tilt
    Vector4d v_4 = Vector4d(0.0, 1.0, 0.0, 1.0);
    Vector4d n_t = t.matrix() * v_4;
    Vector3d normal = Vector3d(n_t[0], n_t[1], n_t[2]);

    m_ground_collider = std::shared_ptr<Plane>(new Plane(Vector3d(0.0, 0.0, 0.0), normal));
    m_colliders.push_back(m_ground_collider);
}

void Simulation::initSphere() {
    Affine3d t = Affine3d(Translation3d(center[0], center[1], center[2]));
    t.scale(r);
    Affine3f t_f = t.cast <float> ();
    m_sphere.setModelMatrix(t_f);

    std::vector<Vector3d> vertices;
    std::vector<Vector4i> tets;

    if(MeshLoader::loadTetMesh("example-meshes/sphere.mesh", vertices, tets)) {
        // Create faces from nodes
        std::vector<Vector3i> faces;

        // Create faces from nodes
        for (Vector4i t : tets) {
            faces.emplace_back(t[3], t[1], t[2]);
            faces.emplace_back(t[2], t[0], t[3]);
            faces.emplace_back(t[3], t[0], t[1]);
            faces.emplace_back(t[1], t[0], t[2]);
        }

        m_sphere.init(vertices, faces);
    }

    // Add sphere collider
    std::shared_ptr<Sphere> s = std::shared_ptr<Sphere>(new Sphere(center, r));
    m_colliders.push_back(s);
}

void Simulation::interaction(Vector3d dir) {
    m_tetmesh->interaction(dir);
}

void Simulation::tilt_ground(float dir) {
    tilt = tilt + dir;

    if (tilt < -0.5) {
        tilt = -0.5;
    }

    if (tilt > 0.5) {
        tilt = 0.5;
    }

    Affine3d t = Affine3d(Translation3d(0.0, 0.0, 0.0));
    Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(0.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(tilt, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    t.rotate(q);
    Affine3f t_f = t.cast <float> ();
    m_ground.setModelMatrix(t_f);

    Vector4d n = Vector4d(0.0, 1.0, 0.0, 1.0);
    Vector4d n_t = t.matrix() * n;
    Vector3d normal = Vector3d(n_t[0], n_t[1], n_t[2]);

    m_ground_collider->set_normal(normal);
}



