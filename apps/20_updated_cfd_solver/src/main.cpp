#include <cmath>
#include "Configuration.h"

void solidWithAddingWater(Configuration &configuration, int grid_num = 100, int frequency = 50, int first = 1,
                          int px_size = 2);

void circleWater(Configuration &configuration, int grid_num, int radius);

int horizontalLine(Configuration &conf) {
    std::vector<std::pair<int, int>> waterIndices = {};
    int grid_num = 200;
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if ((i < round(grid_num * 0.2) || i > round(grid_num * 0.8) || ((i > round(grid_num * 0.4) && ((i < round(grid_num * 0.6)))))) && j >= round(grid_num * 0.4)) {
                waterIndices.emplace_back(i, j);
            }
        }
    }
    std::vector<std::pair<int, int>> additionalWater = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (j < round(grid_num * 0.3) && j > round(grid_num * 0.1) && i > round(grid_num * 0.7) &&
                i < round(grid_num * 0.8)) {
                additionalWater.emplace_back(i, j);
            }

            if (j < round(grid_num * 0.5) && j > round(grid_num * 0.3) && i > round(grid_num * 0.6) &&
                i < round(grid_num * 0.84)) {
                additionalWater.emplace_back(i, j);
            }
        }
    }

    conf.setGridSize(5).setParticlesPerGrid(5).setGridNum(grid_num)
            .setPxPerCell(3).setSimulationSteps(1000)
            .setWaterIndices(waterIndices).setAdditionalWater(additionalWater, 200, 11000);

}

void basketSolid(Configuration &conf, int grid_num = 30) {
    std::vector<std::pair<int, int>> waterIndices = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (j < round(grid_num * 0.6) && i > round(grid_num * 0.3) && i < round(grid_num * 0.7)) {
                waterIndices.emplace_back(i, j);
            }
        }
    }


    conf.setGridSize(grid_num).setParticlesPerGrid(4).setGridNum(grid_num)
            .setPxPerCell(5).setSimulationSteps(1000)
            .setWaterIndices(waterIndices);
}

void solidWithAddingWater(Configuration &conf, int grid_num, int frequency, int first, int px_size) {
    std::vector<std::pair<int, int>> waterIndices = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (j > round(grid_num * 0.6)) {
                waterIndices.emplace_back(i, j);
            }
        }
    }

    std::vector<std::pair<int, int>> additionalWater = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (j < round(grid_num * 0.3) && j > round(grid_num * 0.1) && i > round(grid_num * 0.7) &&
                i < round(grid_num * 0.8)) {
                additionalWater.emplace_back(i, j);
            }

            if (j < round(grid_num * 0.5) && j > round(grid_num * 0.3) && i > round(grid_num * 0.6) &&
                i < round(grid_num * 0.84)) {
                additionalWater.emplace_back(i, j);
            }
        }
    }

    conf.setGridSize(1).setParticlesPerGrid(3).setGridNum(grid_num)
            .setPxPerCell(px_size).setSimulationSteps(1000)
            .setWaterIndices(waterIndices).setAdditionalWater(additionalWater, frequency, first);
}

int main() {
    Configuration conf = Configuration();


    horizontalLine(conf);

//    basketSolid(conf);
//
//    solidWithAddingWater(conf, 100, 50, 10, 3);
//
//    circleWater(conf, 100, 25);


    conf.start();
    return 0;
}

void circleWater(Configuration &conf, int grid_num, int radius) {
    std::vector<std::pair<int, int>> waterIndices = {};

    int center_x = grid_num / 1.5;
    int center_y = grid_num / 1.5;
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (pow(i - center_x, 2) + pow(j - center_y, 2) <= pow(radius, 2) && j <= center_y) {
                waterIndices.emplace_back(i, j);
            }
//            if (pow((j-center_y), 2) - pow((i - center_x), 2) < 100) {
//                waterIndices.emplace_back(i, j);
//            }
//            if (exp10f(i) * sin(i * j) < i * j) {
//                waterIndices.emplace_back(i, j);
//
//            }
        }
    }
    std::vector<std::pair<int, int>> additionalWater = waterIndices;

    conf.setGridSize(1).setParticlesPerGrid(4).setGridNum(grid_num)
            .setPxPerCell(4).setSimulationSteps(1000)
            .setWaterIndices(waterIndices).setAdditionalWater(additionalWater, 1000, 500);
}

