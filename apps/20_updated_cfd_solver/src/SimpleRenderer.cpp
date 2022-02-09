#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "SimpleRenderer.h"
#include "include/stb_image_write.h"

SimpleRenderer::SimpleRenderer(int _grid_px_size, int _grid_num) {
    grid_px_size = _grid_px_size;
    grid_num = _grid_num;
}

void SimpleRenderer::saveImage(const std::string &image_name, std::vector<SpaceType> &spaceTypes) const {
    std::vector<unsigned char> image;
    image.resize(grid_num * grid_num * 4 * grid_px_size * grid_px_size);

    fillImage(image, spaceTypes);

    stbi_write_jpg(image_name.c_str(), grid_num * grid_px_size,
                   grid_num * grid_px_size, 4, &image[0], 100);
}

void SimpleRenderer::fillImage(vector<unsigned char> &image, std::vector<SpaceType> &spaceTypes) const {
    const Color solidColor = Color(121, 134, 133);
    const Color emptyColor = Color(255, 255, 255);
    const Color fluidColor = Color(5, 125, 158);

    for (int i = 0; i < grid_num * grid_num; ++i) {
        if (spaceTypes[i] == SpaceType::Solid) {
            drawSquare(solidColor, i, image);
        } else if (spaceTypes[i] == SpaceType::Empty) {
            drawSquare(emptyColor, i, image);
        } else {
            drawSquare(fluidColor, i, image);
        }
    }
}

int SimpleRenderer::countPixelIndex(int i, int j, int k) const {
    return 4 * (i % grid_num * grid_px_size + (i / grid_num) * grid_px_size * grid_px_size * grid_num + k +
                j * grid_num * grid_px_size);
}

void SimpleRenderer::drawSquare(const SimpleRenderer::Color color, int i, vector<unsigned char> &image) const {
    for (int j = 0; j < grid_px_size; ++j) {
        for (int k = 0; k < grid_px_size; ++k) {
            int indx = countPixelIndex(i, j, k);
            image[indx] = color.r;
            image[indx + 1] = color.g;
            image[indx + 2] = color.b;
            image[indx + 3] = 255;
        }
    }
}



