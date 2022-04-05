#ifndef TEST_SIMPLERENDERER_H
#define TEST_SIMPLERENDERER_H

#include <string>
#include <vector>
#include "src/test_class.h"
#include "IRenderer.h"
#define Solid 0
#define Empty 1
#define Fluid 2

class SimpleRenderer: public IRenderer {

    class Color {
    public:

        unsigned char r, g, b;

        Color(unsigned char r, unsigned char g, unsigned char b) : r(r), g(g), b(b) {
        };

        Color &operator+(Color color) {
            if (int(color.r) + int(r) <= 255) {
                r += color.r;
            }
            if (int(color.g) + int(g) <= 255) {
                g += color.g;
            }
            if (int(color.b) + int(b) <= 255) {
                b += color.b;
            }
            return *this;
        }

        Color &operator*(int c) {
            if (c < 0) {
                return *this;
            }
            if (c * int(r) <= 255) {
                r *= (unsigned char) c;
            }
            if (c * int(g) <= 255) {
                g *= (unsigned char) c;
            }
            if (c * int(b) <= 255) {
                b *= (unsigned char) c;
            }
            return *this;
        }

    };

    int grid_px_size;
    int radius = 12;
    int max_xy = 4;
    int grid_num; // Квадратная сетка
    double dx;
public:

    SimpleRenderer(int _grid_px_size, int _grid_num, int grid_size);

    void saveImage(const std::string &image_name, std::vector<int> &spaceTypes,
                   std::vector<Particle>, RenderMode mode) override;

    void fillEmptyImage(vector<unsigned char> &image, std::vector<int> &spaceTypes) const;

    int countPixelIndex(int i, int j, int k) const;

    void drawSquare(Color color, int i, vector<unsigned char> &image) const;

//    void drawCircle(const Color color, int i, vector<unsigned char> &image) const;
    void fillBlobbies(vector<unsigned char> &image, vector<Particle> &particles);

    void fillPixel(vector<unsigned char> &image, int indx, Color color) const;

    bool isFluid(int i, int j, vector<Particle> &particles, int particle_indx, float U0, float min_x, float max_x,
                 float min_y, float max_y) const;

    double countPotential(int i, int j, Particle &particle, float min_x, float max_x, float min_y, float max_y) const;

    double countDistance(int i, int j, Particle &particle, float min_x, float max_x, float min_y, float max_y) const;

    void drawCircle(vector<unsigned char> &image, Particle &particle, int radius);

    vector<std::pair<int, int>>
    getIndices(vector<Particle> &particles, int particle_idx, float u0, float &left_x, float &right_x, float &top_y,
               float &bot_y) const;

    int getIndex(float x) const;

    void
    fillSquareImage(vector<unsigned char> &image, vector<int> &spaceTypes, vector<Particle> &particles) const;

    void addToPixel(vector<unsigned char> &image, int indx, Color color) const;

    void drawSquareWithAddition(const Color color, int i, vector<unsigned char> &image) const;

    Color countToColor(int count) const;

    unsigned char cutValue(unsigned char from, unsigned char to, int w) const;

    void interpolate(vector<unsigned char> &image, int kernelSize) const;

    bool isFilled(vector<unsigned char> &image, int index);
    ~SimpleRenderer() override = default;
};


#endif //TEST_SIMPLERENDERER_H