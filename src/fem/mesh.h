#ifndef MESH_H
#define MESH_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <unordered_map>
#include "node.h"
#include "tet.h"
#include "face.h"
#include "../colliders/collider.h"
#include "../colliders/plane.h"
#include "../colliders/sphere.h"

using namespace Eigen;

class Mesh {
public:
    Mesh(std::vector<std::shared_ptr<Node>> vertices,
         std::vector<std::shared_ptr<Tet>> tets,
         Matrix4d model);
    std::vector<Vector3i> get_surface_faces();
    std::vector<Vector3d> get_surface_nodes();
    void update(double seconds);
    void collision(std::vector<std::shared_ptr<Collider>>);
    void interaction(Vector3d dir);

private:
    std::vector<std::shared_ptr<Node>> m_vertices;
    std::vector<std::shared_ptr<Tet>> m_tets;
    std::vector<std::shared_ptr<Face>> m_surface_faces;
    std::vector<std::shared_ptr<Node>> m_surface_nodes;
    std::unordered_map<std::shared_ptr<Node>, int> node_map;
    Matrix4d m_model;
    void initialize_surface_faces();
    void initialize_surface_nodes();
};

#endif // MESH_H
