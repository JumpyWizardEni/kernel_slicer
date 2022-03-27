#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <algorithm>
#include <map>
#include "SimpleRenderer.h"
#include "include/stb_image_write.h"

SimpleRenderer::SimpleRenderer(int _grid_px_size, int _grid_num) {
    grid_px_size = _grid_px_size;
    grid_num = _grid_num;
}

void SimpleRenderer::saveImage(const std::string &image_name, std::vector<SpaceType> &spaceTypes,
                               std::vector<Particle> particles, RenderMode mode) const {
    std::vector<unsigned char> image;
    image.resize(grid_num * grid_num * 4 * grid_px_size * grid_px_size, 0);

    if (mode == RenderMode::Square) {
        fillSquareImage(image, spaceTypes, particles);
    } else {
        fillEmptyImage(image, spaceTypes);

        fillBlobbies(image, particles);
    }


    stbi_write_jpg(image_name.c_str(), grid_num * grid_px_size,
                   grid_num * grid_px_size, 4, &image[0], 100);
}

void SimpleRenderer::fillEmptyImage(vector<unsigned char> &image, std::vector<SpaceType> &spaceTypes) const {
    const Color solidColor = Color(121, 134, 133);
    const Color emptyColor = Color(255, 255, 255);

    for (int i = 0; i < grid_num * grid_num; ++i) {
        if (spaceTypes[i] == SpaceType::Solid) {
            drawSquare(solidColor, i, image);
        } else {
            drawSquare(emptyColor, i, image);
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
            fillPixel(image, indx, color);

        }
    }
}


void
SimpleRenderer::drawSquareWithAddition(const SimpleRenderer::Color color, int i, vector<unsigned char> &image) const {
    for (int j = 0; j < grid_px_size; ++j) {
        for (int k = 0; k < grid_px_size; ++k) {
            int indx = countPixelIndex(i, j, k);
            addToPixel(image, indx, color);

        }
    }
}

struct sort_particles {
    bool operator()(Particle &left, Particle &right) {
        return pow(left.pos_x, 2) + pow(left.pos_y, 2) <= pow(right.pos_x, 2) + pow(right.pos_y, 2);
    }
};

void SimpleRenderer::fillBlobbies(vector<unsigned char> &image, vector<Particle> &particles) const {
    const Color fluidColor = Color(5, 125, 158);
    float U0 = 3;
    std::sort(particles.begin(), particles.end(), sort_particles());
    for (int i = 0; i < particles.size() - 3; i++) {
        float max_x, max_y, min_x, min_y;
        vector<std::pair<int, int>> fillIndices = std::move(
                getIndices(particles, i, U0, min_x, max_x, min_y, max_y));
        for (auto &ind: fillIndices) {
            if (isFluid(ind.first, ind.second, particles, i, U0, min_x, max_x, min_y, max_y)) {
                fillPixel(image, 4 * (ind.first + grid_num * grid_px_size * ind.second), fluidColor);
            }
        }
    }


}

void SimpleRenderer::fillPixel(vector<unsigned char> &image, int indx, Color color) const {
    image[indx] = color.r;
    image[indx + 1] = color.g;
    image[indx + 2] = color.b;
    image[indx + 3] = 255;
}

void SimpleRenderer::addToPixel(vector<unsigned char> &image, int indx, Color color) const {
    if ((int) image[indx] + (int) color.r <= 255) {
        image[indx] += color.r;
    }
    if ((int) image[indx + 1] + (int) color.g <= 255) {
        image[indx + 1] += color.g;

    }
    if ((int) image[indx + 2] + (int) color.b <= 255) {
        image[indx + 2] += color.b;

    }
    image[indx + 3] += 255;
}

bool
SimpleRenderer::isFluid(int i, int j, vector<Particle> &particles, int particle_indx, float U0, float min_x,
                        float max_x, float min_y, float max_y) const {
    double sum = 0;
    for (int k = particle_indx; k < particle_indx + 3; k++) {
        sum += countPotential(i, j, particles[k], min_x, max_x, min_y, max_y);
        if (sum > U0) {
            return false;
        }
    }
    return true;
}

double SimpleRenderer::countPotential(int i, int j, Particle &particle, float min_x, float max_x, float min_y,
                                      float max_y) const {
    return pow(1 - countDistance(i, j, particle, min_x, max_x, min_y, max_y), 2);
}

double SimpleRenderer::countDistance(int i, int j, Particle &particle, float min_x, float max_x, float min_y,
                                     float max_y) const {
    int size = grid_px_size * grid_num;
    float x = particle.pos_x * size;
    float y = particle.pos_y * size;
    //Нормализация в отрезок [0, 1]
    x = (x - min_x) / (max_x - min_x);
    y = (y - min_y) / (max_y - min_y);
    //Нормализация в отрезок [-3, 3]
    float _i = -3 + ((float) (i - min_x)) / (max_x - min_x) * 6;
    float _j = -3 + ((float) (j - min_y)) / (max_y - min_y) * 6;
    float distance = pow(x - _i, 2) + pow(y - _j, 2);
    return distance;
}

void SimpleRenderer::drawCircle(vector<unsigned char> &image, Particle &particle, int radius) const {
    const Color fluidColor = Color(5, 125, 158);
    int x = round(particle.pos_x * grid_num);
    int y = round(particle.pos_y * grid_num);
    int x_center = radius / 2;
    int y_center = radius / 2;
    for (int j = 0; j < radius; ++j) {
        for (int k = 0; k < radius; ++k) {
            if (pow(j - x_center, 2) + pow(k - y_center, 2) <= radius * radius) {
                int indx = countPixelIndex(x + y * grid_num, j, k);
                fillPixel(image, indx, fluidColor);

            }

        }
    }
}

int SimpleRenderer::getIndex(float x) const {
    return round(x * grid_num * grid_px_size);
}

vector<std::pair<int, int>>
SimpleRenderer::getIndices(vector<Particle> &particles, int particle_idx, float u0, float &left_x, float &right_x,
                           float &top_y, float &bot_y) const {
    vector<std::pair<int, int>> res = {};
    Particle &p1 = particles[particle_idx];
    Particle &p2 = particles[particle_idx + 1];
    Particle &p3 = particles[particle_idx + 2];
    int min_x = std::min(std::min(getIndex(p1.pos_x), getIndex(p2.pos_x)), getIndex(p3.pos_x));
    int max_x = std::max(std::max(getIndex(p1.pos_x), getIndex(p2.pos_x)), getIndex(p3.pos_x));
    int min_y = std::min(std::min(getIndex(p1.pos_y), getIndex(p2.pos_y)), getIndex(p3.pos_y));
    int max_y = std::max(std::max(getIndex(p1.pos_y), getIndex(p2.pos_y)), getIndex(p3.pos_y));
    if (max_x - min_x > 20 * grid_px_size || max_y - min_y > 20 * grid_px_size) {
        return {};
    }
    int radius = 3;
    left_x = min_x - grid_px_size * radius;
    right_x = max_x + grid_px_size * radius;
    top_y = min_y - grid_px_size * radius;
    bot_y = max_y + grid_px_size;
    for (int i = left_x; i <= right_x; ++i) {
        for (int j = top_y; j <= bot_y; ++j) {
            if (i > grid_px_size && i < (grid_num - 1) * (grid_px_size) && j > grid_px_size &&
                j < (grid_num - 1) * (grid_px_size)) {
                res.emplace_back(i, j);
            }
        }
    }
    return res;
}

void SimpleRenderer::fillSquareImage(vector<unsigned char> &image, vector<SpaceType> &spaceTypes,
                                     vector<Particle> &particles) const {
    const Color solidColor = Color(121, 134, 133);
    const Color emptyColor = Color(255, 255, 255);
    const Color fluidColor = Color(15, 94, 156);

    for (int i = 0; i < grid_num * grid_num; ++i) {
        if (spaceTypes[i] == SpaceType::Solid) {
            drawSquare(solidColor, i, image);
        } else if (spaceTypes[i] == SpaceType::Empty) {
            drawSquare(emptyColor, i, image);
        } else {
            drawSquare(fluidColor, i, image);
        }
    }
    for (int i = 0; i < particles.size(); ++i) {
        Particle &p = particles[i];
        int x = cutValue(1, grid_num - 2, floor(p.pos_x * grid_num));
        int y = cutValue(1, grid_num - 2, floor(p.pos_y * grid_num));

        drawSquareWithAddition(Color(0, 1, 1), (x + y * grid_num), image);
    }

//    interpolate(image, 1);
//    std::map<int, int> part_count = std::map<int, int>(); // index to number of particles
//    for (int i = 0; i < particles.size(); ++i) {
//        Particle &p = particles[i];
//        int ind = round(p.pos_x * grid_num) + grid_num * round(p.pos_y * grid_num);
//        if (part_count.count(ind)) {
//            part_count[ind]++;
//        } else {
//            part_count[ind] = 1;
//        }
//    }
//    for (auto &count: part_count) {
//        int ind = count.first;
//        int p_count = count.second;
//        drawSquare(countToColor(p_count), ind, image);
//    }
}

SimpleRenderer::Color SimpleRenderer::countToColor(int count) const {
    return SimpleRenderer::Color(0, cutValue(0, 255, 23 * count), cutValue(0, 255, 33 * count));
}

unsigned char SimpleRenderer::cutValue(unsigned char from, unsigned char to, int w) const {
    if (w < from) {
        return from;
    }
    if (w > to) {
        return to;
    }
    return (unsigned char) w;
}

void SimpleRenderer::interpolate(vector<unsigned char> &image, int kernelSize) const {
    int size = grid_num * grid_px_size;
    for (int j = grid_px_size; j < size - grid_px_size; ++j) {
        for (int i = grid_px_size; i < size - grid_px_size; ++i) {
            if (image[4 * (i + size * j)] == 255) {
                int new_r = 0;
                int new_g = 0;
                int new_b = 0;
                int c = 0;
                for (int j1 = j - kernelSize; j1 <= j + kernelSize; ++j1) {
                    for (int i1 = i - kernelSize; i1 <= i + kernelSize; ++i1) {
                        if (j1 >= grid_px_size && i1 >= grid_px_size && j1 <= size - grid_px_size &&
                            i1 <= size - grid_px_size && (j1 != j && i1 != i)) {
                            new_r += image[(i + size * j)];
                            new_g += image[(i + size * j) + 1];
                            new_b += image[(i + size * j) + 2];
                            c++;
                        }
                    }
                }
                fillPixel(image, 4 * (i + size * j),
                          Color((unsigned char) new_r / c, (unsigned char) new_g / c, (unsigned char) new_b / c));
            }
        }
    }
}




