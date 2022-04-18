#include "mesh.h"

double epsilon = 1e-3;

Mesh::Mesh(std::vector<std::shared_ptr<Node>> vertices,
           std::vector<std::shared_ptr<Tet>> tets,
           Matrix4d model)
    : m_vertices(vertices), m_tets(tets), m_model(model) {

    initialize_surface_faces();
    initialize_surface_nodes();
}

/**
 * @brief Mesh::initialize_surface_faces - determines which faces in the mesh are surface faces
 */
void Mesh::initialize_surface_faces() {
    // Find the surface faces
    std::set<std::shared_ptr<Face>> surface_faces;

    // Iterate through the faces of each tet
    for (std::shared_ptr<Tet> t : m_tets) {
        std::vector<std::shared_ptr<Face>> faces = t->get_faces();

        for (std::shared_ptr<Face> f : faces) {
            if (surface_faces.size() == 0) {
                surface_faces.insert(f);
            } else {
                bool contains = false;
                for (std::shared_ptr<Face> sf : surface_faces) {
                    // Check if surface faces contains the current face
                    if (f->eq(sf)) {
                        surface_faces.erase(sf);
                        contains = true;
                        break;
                    }
                }

                if (!contains) {
                    surface_faces.insert(f);
                }
            }
        }
    }

    for (std::shared_ptr<Face> f : surface_faces) {
        m_surface_faces.push_back(f);
    }
}

/**
 * @brief Mesh::initialize_surface_nodes - determines which nodes in the mesh are surface nodes
 */
void Mesh::initialize_surface_nodes() {
    // Find the surface nodes
    std::set<std::shared_ptr<Node>> surface_nodes;

    int id = 0;
    for (std::shared_ptr<Face> f : m_surface_faces) {
        for (std::shared_ptr<Node> n : f->get_vertices()) {
            // Add surface nodes and node indices
            if (!surface_nodes.count(n)) {
                surface_nodes.insert(n);
                m_surface_nodes.push_back(n);
                node_map[n] = id;
                id+=1;
            }
        }
    }
}

std::vector<Vector3i> Mesh::get_surface_faces() {
    std::vector<Vector3i> faces;
    for (std::shared_ptr<Face> f : m_surface_faces) {
        faces.push_back(Vector3i(node_map[f->get_n_1()], node_map[f->get_n_2()], node_map[f->get_n_3()]));
    }
    return faces;
}

std::vector<Vector3d> Mesh::get_surface_nodes() {
    std::vector<Vector3d> nodes;
    for (std::shared_ptr<Node> n : m_surface_nodes) {
        nodes.push_back(n->get_loc());
    }
    return nodes;
}

void Mesh::update(double seconds) {
    bool mid;

    // Update the forces of each tet
    mid = true;

    #pragma omp parallel for
    for (std::shared_ptr<Tet> t : m_tets) {
        t->update_forces(mid);
    }

    // Update the midpoint position and velocity of each node
    #pragma omp parallel for
    for (std::shared_ptr<Node> n : m_vertices) {
        n->update(seconds, mid);
    }

    // Update the forces of each tet at the midpoint
    mid = false;

    #pragma omp parallel for
    for (std::shared_ptr<Tet> t : m_tets) {
        t->update_forces(mid);
    }

    // Update the position and velocity of each node
    #pragma omp parallel for
    for (std::shared_ptr<Node> n : m_vertices) {
        n->update(seconds, mid);
    }
}

void Mesh::collision(std::vector<std::shared_ptr<Collider>> colliders) {
    // Check each surface node in the mesh for collisions with ground plane
    for (std::shared_ptr<Node> n : m_surface_nodes) {
        for (std::shared_ptr<Collider> c : colliders) {
            c->collision(n, m_model);
        }
    }
}

bool sameSide(Vector3d p1, Vector3d p2, Vector3d a, Vector3d b) {
    Vector3d cp1 = (b-a).cross(p1-a);
    Vector3d cp2 = (b-a).cross(p2-a);

    return (cp1.dot(cp2) >= -0.01);
}

Vector3d to_object(Vector3d v, Matrix4d model) {
    Vector4d v_4 = Vector4d(v[0], v[1], v[2], 1.0);
    Vector4d object = model.inverse() * v_4;

    return Vector3d(object[0], object[1], object[2]);
}


void Mesh::interaction(Vector3d dir) {
    m_tets[0]->push(dir);
}

