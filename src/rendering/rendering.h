#ifndef RENDERING_H
#define RENDERING_H

#include <Eigen/Dense>
#include <vector>
#include <QFileInfo>
#include <QString>
#include <iostream>
#include <fstream>

using namespace Eigen;

class Rendering {
public:
    static void write_vol(const std::string &file, std::vector<std::vector<std::vector<float>>> values);
    static void test();
};

#endif // COLLIDER_H
