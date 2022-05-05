#include "simulation.h"

float tilt = 0.0;
double r = 0.5;
Vector3d center = Vector3d(0.5, 0.0, 0.0);

void Simulation::toggleARROWS()
{
    arrowsBOOL = !arrowsBOOL;
}
void Simulation::toggleDENSITY()
{
    densitiesBOOL = !densitiesBOOL;
}
void Simulation::toggleVOXELS()
{
    voxelsBOOL = !voxelsBOOL;
}

void Simulation::draw(Shader *shader, Shader *m_normalsShader, Shader *m_normalsArrowShader)
{
    shader->bind();

    if(voxelsBOOL)
    {
        for(Shape s : voxels)
        {
            s.draw(shader, true, s.alpha);
        }
    }

    if(densitiesBOOL)
    {
        for(std::vector<Shape> a : densitySpheres)
        {
            for(Shape s: a )
            {
                s.m_red = (float)a.size()/(float)MAXDENSITYSPHERES;
                s.m_green = (1.f-(((float)a.size()/(float)MAXDENSITYSPHERES)))-0.5;
                if(s.m_green < 0)
                {
                    s.m_green = 0;
                }
                s.m_blue = 0.f;
                s.m_wireframe = true;
                s.draw(shader, false, 1.f);

            }
        }
    }

    if(arrowsBOOL)
    {
        for(Shape _arrow: arrows)
        {
            _arrow.draw(shader, false, 1.f);
        }

        for(Shape _stem: stems)
        {
            _stem.draw(shader, true, 1.f);
        }
    }

    shader->unbind();
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

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}


void Simulation::initSphere(std::shared_ptr<Grid> grid)
{
    //glClear(GL_COLOR_BUFFER_BIT);
    int voxNum = 6;

    for(int i = 0; i < voxNum; i++)
    {
        for(int j = 0; j < voxNum; j++)
        {
            for(int k = 0; k < voxNum; k++)
            {
                //Eigen::Vector3f normal = {(float)rand()/RAND_MAX - 0.5, (float)rand()/RAND_MAX - 0.5, (float)rand()/RAND_MAX - 0.5};
                Eigen::Vector3f normal = Eigen::Vector3f((float) grid->grid[i][j][k]->centerVel[0], (float) grid->grid[i][j][k]->centerVel[1], (float) grid->grid[i][j][k]->centerVel[2]).normalized();
                int density_amt = (int) (grid->grid[i][j][k]->density * MAXDENSITYSPHERES); // rand() % MAXDENSITYSPHERES;  //



                Shape voxel;
                voxel.alpha = 1.0f;


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

                    for(size_t n = 0; n < sphereData.size(); n+=3)
                    {
                        Eigen::Vector3d f = {sphereData[n], sphereData[n+1], sphereData[n+2]};
                        f += offset;
                        Eigen::Vector3d to = f-avg;
                        sd.push_back(to*sphereSize);
                    }


                    for(size_t n = 0; n < sd.size(); n+=3)
                    {
                        t = {n, n+1, n+2};
                        triangles.push_back(t);
                    }

                    m_sphere = Shape();
                    m_sphere.init(sd, triangles);
                    desityS.push_back(m_sphere);
                }
                densitySpheres.push_back(desityS);


                std::vector<GLfloat> arrowData = arrowDATA;
                std::vector<Eigen::Vector3d> pos;
                for(size_t n = 0; n < arrowData.size(); n+=3)
                {
                    Eigen::Vector3d l = {arrowData[n],arrowData[n+1],arrowData[n+2]};
                    pos.push_back(l);
                }

                std::vector<Eigen::Vector3i> triangles;
                Eigen::Vector3i p;
                p = {0,1,2};triangles.push_back(p);
                p = {0,2,3};triangles.push_back(p);
                p = {0,3,4};triangles.push_back(p);
                p = {0,4,1};triangles.push_back(p);

                p = {5,2,1};triangles.push_back(p);
                p = {5,3,2};triangles.push_back(p);
                p = {5,4,3};triangles.push_back(p);
                p = {5,1,4};triangles.push_back(p);


                arrow.init(pos, triangles);

                std::vector<Eigen::Vector3i> STEMtriangles;
                std::vector<Eigen::Vector3d> STEMpos;
                STEMpos.push_back({0.0,0.0,0.0});
                STEMpos.push_back({0.0,-0.1,0.0});
                STEMtriangles.push_back({0,1,0});
                stem.init(STEMpos, STEMtriangles);



                float pitch = asin(-normal[1]);
                float yaw = atan2(normal[0], normal[2]);


                // Mesh translation
                Affine3d t = Affine3d(Translation3d(0, 0, 0));

                // Mesh rotation
                Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
                Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
                Translation<float,3>(i, j, k).translation();
                t.translate(Translation3d(i, j, k).translation());
                t.rotate(q);

                //Translation3d m = Translation3d(i, j, k);
                //t.translate(m);

//                //Eigen::Matrix3d R;
//                // Find your Rotation Matrix
//                Eigen::Vector3d T = {i,j,k};
//                // Find your translation Vector
//                Eigen::Matrix4d Trans; // Your Transformation Matrix
//                Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
//                Trans.block<3,3>(0,0) = t;
//                Trans.block<3,1>(0,3) = T;


//                Eigen::Affine3d r = create_rotation_matrix(1.0, 1.0, 1.0);
//                Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(1,1,2)));

//                Eigen::Matrix4d m = (t * r).matrix();





                Affine3f t_f = t.cast <float> ();
                stem.setModelMatrix(t_f);
                arrow.setModelMatrix(t_f);


                //translate, rotate, then translate to fix issue




                stems.push_back(stem);
                arrows.push_back(arrow);

            }
        }
    }



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


