#ifndef SHAPE_H
#define SHAPE_H

#include <GL/glew.h>
#include <vector>
#include <Eigen/Dense>
#include "../fem/node.h"
#include "../fem/tet.h"

class Shader;

class Shape
{
public:
    Shape();

    void init(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3d> &normals, const std::vector<Eigen::Vector3i> &triangles);
    void init(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3i> &triangles);
    void init(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3i> &triangles, const std::vector<Eigen::Vector4i> &tetIndices);
    void initQuads(const std::vector<Eigen::Vector4d> &vertices, const std::vector<Eigen::Vector4i> &quads);

    void setVertices(const std::vector<Eigen::Vector3d> &vertices, const std::vector<Eigen::Vector3d> &normals);
    void setVertices(const std::vector<Eigen::Vector3d> &vertices);

    void setModelMatrix(const Eigen::Affine3f &model);

    void toggleWireframe();

    void draw(Shader *shader, bool wf, float trans);
    float alpha = 1.f;

    float m_red;
    float m_blue;
    float m_green;
    float m_alpha;
    bool m_wireframe;

    Eigen::Vector3f velocity = {0,0,0};

private:
    GLuint m_surfaceVao;
    GLuint m_tetVao;
    GLuint m_surfaceVbo;
    GLuint m_tetVbo;
    GLuint m_surfaceIbo;
    GLuint m_tetIbo;

    unsigned int m_numSurfaceVertices;
    unsigned int m_numTetVertices;
    unsigned int m_verticesSize;


    std::vector<Eigen::Vector3i> m_faces;
    std::vector<std::shared_ptr<Node>> m_vertices;
    std::vector<std::shared_ptr<Tet>> m_tets;

    Eigen::Matrix4f m_modelMatrix;



};

#endif // SHAPE_H
