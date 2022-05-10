#include "rendering.h"

void Rendering::write_vol(const std::string &file, std::vector<std::vector<std::vector<float>>> values) {
    std::ofstream f;
    f.open(file, std::ios::out | std::ios::binary);

    if(!f.is_open()) {
          std::cout << "ERROR: Cannot open output file" << std::endl;
          return;
    }

    int x_size = values.size();
    int y_size = values[0].size();
    int z_size = values[0][0].size();

    char vol[3] = {'V', 'O', 'L'};
    f.write((char*) &vol, sizeof(vol));

    uint8_t version = 3; // File format version number
    f.write((char*) &version, sizeof(version));

    int32_t type = 1; // Encoding identifier (float32).
    f.write((char*) &type, sizeof(type));

    int32_t size[3] = {x_size, y_size, z_size}; // Number of cells in each dimension
    f.write((char*) &size, sizeof(size));

    int32_t channels = 1; // Number of channels
    f.write((char*) &channels, sizeof(channels));

    float bbox[6] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};
    f.write((char*) &bbox, sizeof(bbox));

    for (int x = 0; x < x_size; ++x) {
        for (int y = 0; y < y_size; ++y) {
              for (int z = 0; z < z_size; ++z) {
                    {
                      float data = values[x][y][z];
                      f.write((char*) &data, sizeof(float));
                    }
              }
        }
    }
    f.close();
    std::cout << "Wrote to vol file" << std::endl;
}

float sigmat_f(int x, int y, int z) {
    if (x > 0.2 && y > 0.7 && z < 0.5)
        return 5;
    return std::max(0.0, (std::sin(16 * (x + 4 * std::pow(y - 0.5, 2) - 0.25)) + 1) * 0.5);
}

void Rendering::test() {
    int res = 16;

    std::vector<std::vector<std::vector<float>>> sigmat;

    for (int x = 0; x < res; ++x) {
        std::vector< std::vector<float>> v_x;
        for (int y = 0; y < res; ++y) {
            std::vector<float> v_y;

            for (int z = 0; z < res; ++z) {
                float val = sigmat_f(x / res, y / res, z / res);
                v_y.push_back(val);
            }

            v_x.push_back(v_y);
        }

        sigmat.push_back(v_x);
    }
    write_vol("./sigmat.vol", sigmat);
}
