#ifndef SIMULATION_H
#define SIMULATION_H

#include "graphics/shape.h"
#include "fem/mesh.h"

class Shader;

class Simulation
{
public:
    Simulation();

    void init();

    void update(float seconds);

    void draw(Shader *shader);

    void toggleWire();

    void interaction(Vector3d dir);
    void tilt_ground(float dir);

    Shape m_sphere;
    Shape arrow;

private:
    Shape m_shape;
    Shape m_ground;
    std::vector<Shape> voxels;
    std::vector<std::vector<Shape>> densitySpheres;
    //Shape m_visulizer;
    void initGround();
    void initSphere();
    std::shared_ptr<Mesh> m_tetmesh;
    std::vector<std::shared_ptr<Collider>> m_colliders;
    std::shared_ptr<Plane> m_ground_collider;
};

#endif // SIMULATION_H
