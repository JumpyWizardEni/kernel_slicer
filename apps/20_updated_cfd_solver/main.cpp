#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "test_class.h"
#include <string>
#include "stb_image_write.h"
#include <vector>
#include <iostream>

using std::vector;

const char GRID_SIZE = 20;
const int N = 100;

double randfrom(double min, double max) {
    double range = (max - min);
    double div = RAND_MAX / range;
    return min + (rand() / div);
}

void save_image(int N, const std::string &image_name, std::vector<SpaceType> spaceTypes) {
    std::vector<unsigned char> image;
    image.resize(N * N * 4 * GRID_SIZE * GRID_SIZE);

    for (int i = 0; i < N * N; ++i) {
        unsigned char r, g, b;
        if (spaceTypes[i] == SpaceType::Solid) {
            r = 121;
            g = 134;
            b = 133;
        } else if (spaceTypes[i] == SpaceType::Empty) {
            r = 255;
            g = 255;
            b = 255;
        } else {
            r = 5;
            g = 125;
            b = 158;
        }
        for (int j = 0; j < GRID_SIZE; ++j) {
            for (int k = 0; k < GRID_SIZE; ++k) {
                int indx = 4 * (i % N * GRID_SIZE + (i / N) * GRID_SIZE * GRID_SIZE * N + k + j * N * GRID_SIZE);
                image[indx] = r;
                image[indx + 1] = g;
                image[indx + 2] = b;
                image[indx + 3] = 255;
            }
        }

    }

    stbi_write_jpg(image_name.c_str(), N * GRID_SIZE, N * GRID_SIZE, 4, &image[0], 100);
}


int main() {

    Solver solver = Solver();
    vector<SpaceType> spaceTypes(N * N);
    vector<float> pressure(N * N);
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (j == 0 || i == 0 || j == N - 1 || i == N - 1) {
                spaceTypes[N * i + j] = SpaceType::Solid;
            } else {
                float r = randfrom(0, 1);
                if (r < 0.1) {
                    spaceTypes[N * i + j] = SpaceType::Empty;
                } else {
                    spaceTypes[N * i + j] = SpaceType::Fluid;
                }
            }
        }
    }

    vector<float> vx((N + 1) * N);
    for (int i = 0; i < N * (N + 1); ++i)
        vx[i] = randfrom(-3, 3);
    for (int i = 0; i < N * (N); ++i)
        pressure[i] = randfrom(-3, 3);
    vector<float> vy(N * (N + 1));
    for (int i = 0; i < N * (N + 1); ++i)
        vy[i] = randfrom(-3, 3);
    solver.size = N;
    solver.vx = vx;
    solver.vy = vy;
    solver.spaceTypes = spaceTypes;
    solver.pressure = pressure;

    for (int i = 0; i < 20; ++i) {
        save_image(N, std::to_string(i) + ".jpeg", solver.spaceTypes);
        std::cout << i << std::endl;
        solver.performStep();
    }
    return 0;
}