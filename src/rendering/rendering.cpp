#include "rendering.h"

void Rendering::write_vol(const std::string &file, std::vector<std::vector<std::vector<float>>> values) {
    std::ofstream f;
    f.open(file, std::ios::out | std::ios::binary);

    if(!f) {
          std::cout << "ERROR: Cannot open output file" << std::endl;
    }

    int vol[4] = {'V', 'O', 'L'};
    f.write((char*) &vol, sizeof(vol));

    int version[1] = {3}; // File format version number
    f.write((char*) &version, sizeof(version));

    int type[1] = {1}; // Encoding identifier (float32).
    f.write((char*) &type, sizeof(type));

    int x = values[0].size();
    int y = values[1].size();
    int z = values[2].size();
    int size[3] = {x, y, z}; // Number of cells in each dimension
    f.write((char*) &size, sizeof(size));

    int channels[1] = {1}; // Number of channels
    f.write((char*) &channels, sizeof(channels));

    float bbox[6] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};
    f.write((char*) &bbox, sizeof(bbox));

    // f.write(reinterpret_cast<const char*>( &f ), sizeof( float ));


    //f.write(values.ravel().astype(np.float32).tobytes())

    f.close();
}
