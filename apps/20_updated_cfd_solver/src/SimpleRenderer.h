#ifndef TEST_SIMPLERENDERER_H
#define TEST_SIMPLERENDERER_H

#include <string>
#include <vector>
#include "test_class.h"

enum class RenderMode {
    Square, Blobbies
};

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

    void saveImage(const std::string &image_name, std::vector<SpaceType> &spaceTypes,
                   std::vector<Particle>, RenderMode mode) const;

    void fillEmptyImage(vector<unsigned char> &image, std::vector<SpaceType> &spaceTypes) const;

    int countPixelIndex(int i, int j, int k) const;

    void drawSquare(Color color, int i, vector<unsigned char> &image) const;

//    void drawCircle(const Color color, int i, vector<unsigned char> &image) const;
    void fillBlobbies(vector<unsigned char> &image, vector<Particle> &particles) const;

    void fillPixel(vector<unsigned char> &image, int indx, Color color) const;

    bool isFluid(int i, int j, vector<Particle> &particles, int particle_indx, float U0, float min_x, float max_x, float min_y, float max_y) const;

    double countPotential(int i, int j, Particle &particle, float min_x, float max_x, float min_y, float max_y) const;

    double countDistance(int i, int j, Particle &particle, float min_x, float max_x, float min_y, float max_y) const;

    void drawCircle(vector<unsigned char> &image, Particle &particle, int radius) const;

    vector<std::pair<int, int>>
    getIndices(vector<Particle> &particles, int particle_idx, float u0, float &left_x, float &right_x, float &top_y, float &bot_y) const;

    int getIndex(float x) const;

    void
    fillSquareImage(vector<unsigned char> &image, vector<SpaceType> &spaceTypes, vector<Particle> &particles) const;

    void addToPixel(vector<unsigned char> &image, int indx, Color color) const;

    void drawSquareWithAddition(const Color color, int i, vector<unsigned char> &image) const;

    Color countToColor(int count) const;

    unsigned char cutValue(unsigned char from, unsigned char to, int w) const;

    void interpolate(vector<unsigned char> &image, int kernelSize) const;
};


#endif //TEST_SIMPLERENDERER_H