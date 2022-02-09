#ifndef TEST_SIMPLERENDERER_H
#define TEST_SIMPLERENDERER_H

#include <string>
#include <vector>
#include "test_class.h"

class SimpleRenderer {

    class Color {
    public:

        unsigned char r, g, b;

        Color(unsigned char r, unsigned char g, unsigned char b) : r(r), g(g), b(b) {
        };
    };

    int grid_px_size;
    int grid_num; // Квадратная сетка

public:
    SimpleRenderer(int _grid_px_size, int _grid_num);

    void saveImage(const std::string &image_name, std::vector<SpaceType> &spaceTypes) const;

    void fillImage(vector<unsigned char> &image, std::vector<SpaceType> &spaceTypes) const;

    int countPixelIndex(int i, int j, int k) const;

    void drawSquare(Color color, int i, vector<unsigned char> &image) const;

//    void drawCircle(const Color color, int i, vector<unsigned char> &image) const;
};


#endif //TEST_SIMPLERENDERER_H