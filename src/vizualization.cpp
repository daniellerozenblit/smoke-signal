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
//            _arrow.m_red = _arrow.velocity.norm();
//            _arrow.m_green = (1.f-_arrow.velocity.norm())-0.5;
//            if(_arrow.m_green < 0)
//            {
//                _arrow.m_green = 0;
//            }
//            _arrow.m_blue = 0.f;

            _arrow.draw(shader, false, 1.f);
        }

        for(Shape _stem: stems)
        {
//            _stem.m_red = _stem.velocity.norm();
//            _stem.m_green = (1.f-_stem.velocity.norm())-0.5;
//            if(_stem.m_green < 0)
//            {
//                _stem.m_green = 0;
//            }
//            _stem.m_blue = 0.f;

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

void Simulation::initGridViz() {
    for(int i = 0; i < SIZE_X; i++) {
        for(int j = 0; j < SIZE_Y; j++) {
            for(int k = 0; k < SIZE_Z; k++) {
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
            }
        }
    }
}


void Simulation::initSphere(std::shared_ptr<MACgrid> grid)
{
    //glClear(GL_COLOR_BUFFER_BIT);
    densitySpheres.clear();
    arrows.clear();
    stems.clear();

    for(int i = 0; i < SIZE_X; i++) {
        for(int j = 0; j < SIZE_Y; j++) {
            for(int k = 0; k < SIZE_Z; k++) {
                Eigen::Vector3f normal = Eigen::Vector3f((float) grid->center_vel_x[i][j][k], (float) grid->center_vel_y[i][j][k], (float) grid->center_vel_z[i][j][k]);
                int density_amt = (int) (grid->density[i][j][k] * MAXDENSITYSPHERES);

                std::vector<Shape> desityS;

                float scale = 25.f;
                Eigen::Vector3d offset = {0,0,0};

                for(int o = 0; o < density_amt; o++) {
                    float sphereSize = 1.f/scale;
                    float x = (float)rand()/(float)RAND_MAX;
                    float y = (float)rand()/(float)RAND_MAX;
                    float z = (float)rand()/(float)RAND_MAX;


                    offset[0] = ((x - 0.5f)) + i+(i*0.5);
                    offset[1] = ((y - 0.5f)) + j+(j*0.5);
                    offset[2] = ((z - 0.5f)) + k+(k*0.5);

                    offset *= scale;

                    Eigen::Vector3i t = {};
                    Eigen::Vector3d avg = {0,0,0};

                    std::vector<Eigen::Vector3d> sd;

                    std::vector<GLfloat> sphereData = CUBE_VERTEX_POSITIONS;
                    std::vector<Eigen::Vector3d> pos;

                    for(size_t j = 0; j < sphereData.size(); j+=3)
                    {
                        Eigen::Vector3d f = {sphereData[j], sphereData[j+1], sphereData[j+2]};
                        f /= scale;
                        f += offset;
                        avg += f;
                    }

                    avg[0] = avg[0] / sphereData.size();
                    avg[1] = avg[1] / sphereData.size();
                    avg[2] = avg[2] / sphereData.size();

                    for(size_t n = 0; n < sphereData.size(); n+=3) {
                        Eigen::Vector3d f = {sphereData[n], sphereData[n+1], sphereData[n+2]};
                        f += offset;
                        Eigen::Vector3d to = f-avg;
                        sd.push_back(to*sphereSize);
                    }

                    std::vector<Eigen::Vector3i> triangles;
                    Eigen::Vector3i p;

//                    for (size_t n = 0; n < sd.size(); n+=3) {
//                        t = {n, n+1, n+2};
//                        triangles.push_back(t);
//                    }

                    p = {5,4,0};triangles.push_back(p);
                    p = {1,5,0};triangles.push_back(p);
                    p = {6,5,1};triangles.push_back(p);
                    p = {2,6,1};triangles.push_back(p);

                    p = {7,6,2};triangles.push_back(p);
                    p = {3,7,2};triangles.push_back(p);
                    p = {4,7,3};triangles.push_back(p);
                    p = {0,4,3};triangles.push_back(p);

                    p = {6,7,4};triangles.push_back(p);
                    p = {5,6,4};triangles.push_back(p);
                    p = {1,0,3};triangles.push_back(p);
                    p = {2,1,3};triangles.push_back(p);

                    m_sphere = Shape();
                    m_sphere.init(sd, triangles);
                    desityS.push_back(m_sphere);
                }

                densitySpheres.push_back(desityS);


                //normal = {0,-0.9,0};

                if(normal[1] < 0 && normal[0] == 0 && normal[2] == 0)
                {
                    normal[0] = 0.001;
                }

                //arrows scale with size of 'normal' vector (velocity)
                //expecting values to be between 0-1 (if not it will
                //still work the arrows will just be big af

                float attempt_1 = normal.norm();
                if(attempt_1 >= -0.1)
                {
                    std::vector<GLfloat> arrowData = arrowDATA;
                    std::vector<Eigen::Vector3d> pos;

                    for(size_t n = 0; n < arrowData.size(); n+=3)
                    {
                        Eigen::Vector3d l = {arrowData[n]*attempt_1,arrowData[n+1]*attempt_1,arrowData[n+2]*attempt_1};
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
                    STEMpos.push_back({0.0,-0.1*attempt_1,0.0});
                    STEMtriangles.push_back({0,1,0});
                    stem.init(STEMpos, STEMtriangles);

                    arrow.velocity = normal;
                    normal.normalize();


                    Vector3d b  = {(double)normal[0], (double)normal[1], (double)normal[2]};
                    Vector3d a = {0,1,0};
                    Vector3d v = a.cross(b);
                    double c = a.dot(b);
                    Matrix3d R;
                    Matrix3d I = Matrix3d::Identity();
                    Matrix3d Vx;
                    Vx << 0, v[2], -v[1], -v[2], 0, v[0], v[1], -v[0], 0;
                    R = I + Vx + (Vx * Vx)*(1/(1+c));


                    Quaterniond quaternion;
                    quaternion = R;


                    // Mesh translation
                    Affine3d t = Affine3d(Translation3d(0, 0, 0));

                    // Mesh rotation
                    Translation<float,3>(i, j, k).translation();
                    t.translate(Translation3d(i, j, k).translation());
                    t.rotate(quaternion);


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


