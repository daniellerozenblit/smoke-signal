#ifndef FACE_H
#define FACE_H
#include <set>
#include <vector>
#include <algorithm>
#include "node.h"

using namespace Eigen;

class Face
{
    public:
        Face(std::shared_ptr<Node> n_1, std::shared_ptr<Node> n_2, std::shared_ptr<Node> n_3);
        std::vector<std::shared_ptr<Node>> get_vertices();
        std::shared_ptr<Node> get_n_1();
        std::shared_ptr<Node> get_n_2();
        std::shared_ptr<Node> get_n_3();
        double get_area();
        Vector3d get_normal();
        bool eq(std::shared_ptr<Face> rhs);

    private:
        std::shared_ptr<Node> n_1;
        std::shared_ptr<Node> n_2;
        std::shared_ptr<Node> n_3;
        double m_area;
        Vector3d m_normal;
};

#endif // FACE_H
