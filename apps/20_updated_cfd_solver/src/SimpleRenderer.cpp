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
    float U0 = 1.5 * sqrt(2);
    float size = grid_num * grid_px_size * grid_num * grid_px_size;
//    for (int i = 0; i < size; ++i) {
//        if (isFluid(i, particles, U0)) {
//            fillPixel(image, 4 * i, fluidColor);
//        }
//    }
    std::sort(particles.begin(), particles.end(), sort_particles());
    for (int i = 0; i < 1; ++i) {
        float dx;
        float dy;
        vector<int> fillIndices = getIndices(particles, i, U0, dx, dy);
        for (auto &ind: fillIndices) {
//            if (isFluid(ind, particles, i, U0, dx, dy)) {
            fillPixel(image,
                      4 * (getIndex(particles[i].pos_x) + grid_num * grid_px_size * getIndex(particles[i].pos_y)),
                      Color(0, 0, 0));
            fillPixel(image,
                      4 * (getIndex(particles[i + 1].pos_x) + grid_num * grid_px_size * getIndex(particles[i].pos_y)),
                      Color(0, 0, 0));
            fillPixel(image,
                      4 * (getIndex(particles[i + 2].pos_x) + grid_num * grid_px_size * getIndex(particles[i].pos_y)),
                      Color(0, 0, 0));
            fillPixel(image, 4 * ind, fluidColor);
//            }
        }
//        drawCircle(image, particles[i]);
    }
}

void SimpleRenderer::fillPixel(vector<unsigned char> &image, int indx, Color color) const {
    image[indx] = color.r;
    image[indx + 1] = color.g;
    image[indx + 2] = color.b;
    image[indx + 3] = 255;
}

void SimpleRenderer::addToPixel(vector<unsigned char> &image, int indx, Color color) const {
    if (image[indx] < 255) {
        image[indx] += color.r;
    }
    if (image[indx + 1] < 255) {
        image[indx + 1] += color.g;

    }
    if (image[indx + 2] < 255) {
        image[indx + 2] += color.b;

    }
    image[indx + 3] += 255;
}

bool
SimpleRenderer::isFluid(int indx, vector<Particle> &particles, int particle_indx, float U0, float dx, float dy) const {
    double sum = 0;
    for (int i = particle_indx; i < particle_indx + 3; ++i) {
        sum += countPotential(indx, particles[i], dx, dy);
        if (sum > U0) {
            return true;
        }
    }
    return false;
}

double SimpleRenderer::countPotential(int i, Particle &particle, float dx, float dy) const {
    return pow(1 - countDistance(i, particle, dx, dy), 2);
}

double SimpleRenderer::countDistance(int indx, Particle &particle, float dx, float dy) const {
    int size = grid_px_size * grid_num;
    float x = particle.pos_x * size;
    float i = ((float) (indx % size));
    float y = particle.pos_y * size;
    float j = ((float) (indx / size));
    float distance = pow((x - i) / dx, 2) + pow((y - j) / dy, 2);
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
    return round(x * grid_num) * grid_px_size;
}

vector<int>
SimpleRenderer::getIndices(vector<Particle> &particles, int particle_idx, float u0, float &dx, float &dy) const {
    vector<int> res = vector<int>();
    Particle &p1 = particles[particle_idx];
    Particle &p2 = particles[particle_idx + 1];
    Particle &p3 = particles[particle_idx + 2];
    int min_x = std::min(std::min(getIndex(p1.pos_x), getIndex(p2.pos_x)), getIndex(p3.pos_x));
    int max_x = std::max(std::max(getIndex(p1.pos_x), getIndex(p2.pos_x)), getIndex(p3.pos_x));
    int min_y = std::min(std::min(getIndex(p1.pos_y), getIndex(p2.pos_y)), getIndex(p3.pos_y));
    int max_y = std::max(std::max(getIndex(p1.pos_y), getIndex(p2.pos_y)), getIndex(p3.pos_y));
    dx = max_x - min_x + grid_px_size;
    dy = max_y - min_y + grid_px_size;
    for (int i = min_x - grid_px_size / 2; i <= max_x + grid_px_size / 2; ++i) {
        for (int j = min_y - grid_px_size / 2; j <= max_y + grid_px_size / 2; ++j) {
            res.push_back(i + grid_num * grid_px_size * j);
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
//        for (int i = 0; i < particles.size(); ++i) {
//            Particle &p = particles[i];
//            drawCircle(image, p, 2);
//        }
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
    unsigned char res = 0;
    if (w < 0) {
        return 0;
    }
    if (w > 255) {
        return 255;
    }
    return (unsigned char) w;
}




