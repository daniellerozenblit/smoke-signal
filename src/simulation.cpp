#include "simulation.h"
#include <iostream>
#include "graphics/MeshLoader.h"
#include "graphics/sphere.h"
#include "graphics/cone.h"
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>

using namespace Eigen;

double timestep = 0.03;
int MAXDENSITYSPHERES = 30;
float tilt = 0.0;
double r = 0.5;
Vector3d center = Vector3d(0.5, 0.0, 0.0);
//std::string file = "example-meshes/sphere.mesh";

Simulation::Simulation() {
    std::cout<<"print"<<std::endl;
    //initGrid();
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

//    if(MeshLoader::loadTetMesh(file, vertices, tets))
//    {
//        std::vector<std::shared_ptr<Node>> vertices_o;
//        std::vector<std::shared_ptr<Tet>> tets_o;

//        // Create vertex objects
//        for (Vector3d v_i : vertices)
//        {
//            std::shared_ptr<Node> n = std::shared_ptr<Node>(new Node(v_i));
//            vertices_o.push_back(n);
//        }

//        // Create tet objects
//        for (Vector4i t_i : tets)
//        {
//            std::shared_ptr<Tet> t = std::shared_ptr<Tet>(new Tet(vertices_o[t_i[0]], vertices_o[t_i[1]], vertices_o[t_i[2]], vertices_o[t_i[3]]));
//            tets_o.push_back(t);
//        }

//        m_tetmesh = std::shared_ptr<Mesh>(new Mesh(vertices_o, tets_o, t.matrix()));

//        std::vector<Vector3i> surface_faces = m_tetmesh->get_surface_faces();
//        std::vector<Vector3d> surface_vertices = m_tetmesh->get_surface_nodes();

//        m_shape.init(surface_vertices, surface_faces);
//    }

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

void Simulation::draw(Shader *shader)
{
    for(Shape s : voxels)
    {
        s.draw(shader, true, s.alpha);
    }


    for(std::vector<Shape> a : densitySpheres)
    {
        for(Shape s: a )
        {
            //std::cout << 1.f-(float)a.size()/(float)MAXDENSITYSPHERES << std::endl;
            s.m_red = (float)a.size()/(float)MAXDENSITYSPHERES;
            s.m_green = (1.f-(((float)a.size()/(float)MAXDENSITYSPHERES)))-0.5;
            if(s.m_green < 0)
            {
                s.m_green = 0;
            }
            s.m_blue = 0.f;//1.f-(((float)a.size()/(float)MAXDENSITYSPHERES));
            s.m_wireframe = true;
            s.draw(shader, false, 1.f);

        }
    }

}

void Simulation::toggleWire()
{
    m_shape.toggleWireframe();
}

void Simulation::initGround()
{
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

void Simulation::initSphere()
{

    int voxNum = 3;

    for(int i = 0; i < voxNum; i++)
    {
        for(int j = 0; j < voxNum; j++)
        {
            for(int k = 0; k < voxNum; k++)
            {
                Shape voxel;
                voxel.alpha = 1.0f;//(((float) rand())/RAND_MAX);

                std::vector<Vector3d> vertices;
                std::vector<Vector4i> tets;

                Vector3d a = {-0.5+i, -0.5+j, -0.5+k};
                Vector3d b = {0.5 +i, -0.5+j, -0.5+k};
                Vector3d c = {0.5 +i, -0.5+j, 0.5 +k};
                Vector3d d = {-0.5+i, -0.5+j, 0.5 +k};

                Vector3d e = {-0.5+i, 0.5 +j, -0.5+k};
                Vector3d f = {0.5 +i, 0.5 +j, -0.5+k};
                Vector3d g = {0.5 +i, 0.5 +j, 0.5 +k};
                Vector3d h = {-0.5+i, 0.5 +j, 0.5 +k};

                vertices = { a, b, c, d, e, f, g, h };


                // Create faces from nodes
                std::vector<Vector3i> faces;

                // Create faces from nodes

                faces.emplace_back(0, 1, 0);
                faces.emplace_back(1, 2, 1);
                faces.emplace_back(2, 3, 2);
                faces.emplace_back(3, 0, 3);

                faces.emplace_back(4, 5, 4);
                faces.emplace_back(5, 6, 5);
                faces.emplace_back(6, 7, 6);
                faces.emplace_back(7, 4, 7);

                faces.emplace_back(0, 4, 0);
                faces.emplace_back(3, 7, 3);
                faces.emplace_back(1, 5, 1);
                faces.emplace_back(2, 6, 2);


                voxel.init(vertices, faces);
                voxels.push_back(voxel);


                std::vector<Shape> desityS;
                int density_amt = rand() % MAXDENSITYSPHERES;
                float scale = 25.f;
                Eigen::Vector3d offset = {0,0,0};


                for(int o = 0; o < density_amt; o++)
                {
                    float sphereSize = 1.f/scale;
                    float x = (float)rand()/(float)RAND_MAX;
                    float y = (float)rand()/(float)RAND_MAX;
                    float z = (float)rand()/(float)RAND_MAX;


                    offset[0] = ((x-0.5f)) + i+(i*0.5);
                    offset[1] = ((y-0.5f)) + j+(j*0.5);
                    offset[2] = ((z-0.5f)) + k+(k*0.5);

                    offset *= scale;

                    std::vector<GLfloat> sphereData = SPHERE_VERTEX_POSITIONS;
                    std::vector<Eigen::Vector3d> sd;
                    std::vector<Eigen::Vector3i> triangles;


                    Eigen::Vector3i t = {};
                    Eigen::Vector3d avg = {0,0,0};

                    for(size_t j = 0; j < sphereData.size(); j+=3)
                    {
                        Eigen::Vector3d f = {sphereData[j], sphereData[j+1], sphereData[j+2]};
                        f += offset;
                        avg += f;
                    }

                    avg[0] = avg[0] / sphereData.size();
                    avg[1] = avg[1] / sphereData.size();
                    avg[2] = avg[2] / sphereData.size();

                    for(size_t j = 0; j < sphereData.size(); j+=3)
                    {
                        Eigen::Vector3d f = {sphereData[j], sphereData[j+1], sphereData[j+2]};
                        f += offset;
                        Eigen::Vector3d to = f-avg;
                        sd.push_back(to*sphereSize);
                    }


                    for(size_t j = 0; j < sd.size(); j+=3)
                    {
                        t = {j, j+1, j+2};
                        triangles.push_back(t);
                    }

                    m_sphere = Shape();
                    m_sphere.init(sd, triangles);
                    desityS.push_back(m_sphere);
                }
                densitySpheres.push_back(desityS);






            }
        }
    }


//    std::vector<GLfloat> arrowData = arrowDATA;
//    std::vector<Eigen::Vector3d> pos;
//    for(size_t j = 0; j < arrowData.size(); j+=3)
//    {
//        Eigen::Vector3d a = {arrowData[j],arrowData[j+1],arrowData[j+2]};
//        pos.push_back(a);
//    }

//    std::vector<Eigen::Vector3i> triangles;
//    Eigen::Vector3i a;
//    a = {0,1,2};triangles.push_back(a);
//    a = {0,2,3};triangles.push_back(a);
//    a = {0,3,4};triangles.push_back(a);
//    a = {0,4,1};triangles.push_back(a);

//    a = {5,2,1};triangles.push_back(a);
//    a = {5,3,2};triangles.push_back(a);
//    a = {5,4,3};triangles.push_back(a);
//    a = {5,1,4};triangles.push_back(a);

//    a = {0,5,6};triangles.push_back(a);

//    a = {1,10,2};triangles.push_back(a);
//    a = {2,10,3};triangles.push_back(a);
//    a = {3,10,4};triangles.push_back(a);
//    a = {5,10,6};triangles.push_back(a);
//    a = {6,10,7};triangles.push_back(a);
//    a = {7,10,8};triangles.push_back(a);
//    a = {8,10,9};triangles.push_back(a);
//    a = {9,10,1};triangles.push_back(a);
    //a = {0,10,11};triangles.push_back(a);




//    arrow.init(pos, triangles);
}

void Simulation::interaction(Vector3d dir)
{
    m_tetmesh->interaction(dir);
}

void Simulation::tilt_ground(float dir)
{
    tilt = tilt + dir;

    if (tilt < -0.5)
    {
        tilt = -0.5;
    }

    if (tilt > 0.5)
    {
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

void Simulation::setfaces(std::vector<std::vector<std::vector<std::vector<std::shared_ptr<VoxelFace>>>>> facesin)
{
    faces = facesin;
}

