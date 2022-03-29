#include <cmath>
#include "Configuration.h"

void solidWithAddingWater(Configuration &configuration, int grid_num = 100, int frequency = 50, int first = 1,
                          int px_size = 2);

void circleWater(Configuration &configuration, int grid_num, int radius);

void stair(Configuration &conf, int grid_num);

void smallHoleWall(Configuration &conf, int grid_num);

int twoVerticalLinesWithCenterWall(Configuration &conf, int grid_num = 200);

void circleWall(Configuration &conf, int grid_num = 100);

void circleGridSolid(Configuration &conf, int grid_num);

void basketSolid(Configuration &conf, int grid_num = 50) {
    std::vector<std::pair<int, int>> waterIndices = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (j < round(grid_num * 0.3) && i > round(grid_num * 0.3) && i < round(grid_num * 0.8)) {
                waterIndices.emplace_back(i, j);
            }
        }
    }

    vector<int> solid_indices = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if ((j < round(grid_num * 0.9) && j > round(grid_num * 0.8) && i < round(grid_num * 0.8) &&
                 i > round(grid_num * 0.3))) {
                solid_indices.push_back(i + j * grid_num);
            }
            if (j > round(grid_num * 0.5) && j <= round(grid_num * 0.8) &&
                ((i > round(grid_num * 0.3) && i < round(grid_num * 0.4)) ||
                 ((i > round(grid_num * 0.7) && i < round(grid_num * 0.8))))) {
                solid_indices.push_back(i + j * grid_num);

            }
        }
    }
    std::vector<std::pair<int, int>> addWater = waterIndices;

    conf.setGridSize(10).setParticlesPerGrid(8).setGridNum(grid_num)
            .setPxPerCell(3).setSimulationSteps(1000)
            .setWaterIndices(waterIndices).setAdditionalSolidIndices(solid_indices).setAdditionalWater(addWater, 100,
                                                                                                       50);
}

void solidWithAddingWater(Configuration &conf, int grid_num, int frequency, int first, int px_size) {
    std::vector<std::pair<int, int>> waterIndices = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (j > round(grid_num * 0.6) && (i < round(grid_num * 0.8) && (i > round(grid_num * 0.6)))) {
                waterIndices.emplace_back(i, j);
            }
        }
    }

    std::vector<std::pair<int, int>> additionalWater = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (j < round(grid_num * 0.3) && j > round(grid_num * 0.1) && i > round(grid_num * 0.1) &&
                i < round(grid_num * 0.3)) {
                additionalWater.emplace_back(i, j);
            }

//            if (j < round(grid_num * 0.5) && j > round(grid_num * 0.3) && i > round(grid_num * 0.6) &&
//                i < round(grid_num * 0.84)) {
//                additionalWater.emplace_back(i, j);
//            }
        }
    }

    conf.setGridSize(10).setParticlesPerGrid(12).setGridNum(grid_num)
            .setPxPerCell(px_size).setSimulationSteps(1000)
            .setWaterIndices(waterIndices).setAdditionalWater(additionalWater, frequency, first);
}

int main() {
    Configuration conf = Configuration();

//    twoVerticalLinesWithCenterWall(conf, 200);

//    basketSolid(conf);
//
//    solidWithAddingWater(conf, 100, 50, 10, 3);
//
//    circleWater(conf, 100, 25);

//    stair(conf, 100);

//    smallHoleWall(conf, 100);

//    circleWall(conf, 100);

    circleGridSolid(conf, 300);
    conf.start();
    return 0;
}

void circleGridSolid(Configuration &conf, int grid_num) {
    std::vector<std::pair<int, int>> waterIndices = {};
    double radius = 3;
    double radius_water = 2;
    for (int x = 1; x < grid_num - 1; ++x) {
        for (int y = 1; y < round(grid_num * 0.3); ++y) {
            if (x % 10 == 0 && y % 10 == 0) {
                for (int i = x - radius_water; i <= x + radius_water; ++i) {
                    for (int j = y - radius_water; j <= y + radius_water; ++j) {
                        float r = pow(i - x, 2) + pow(j - y, 2);
                        if (r <= pow(radius_water, 2)) {
                            waterIndices.emplace_back(i, j);
                        }
                    }
                }
            }
        }
    }
    std::vector<std::pair<int, int>> additionalWater = waterIndices;

    std::vector<int> solidIndices = {};
    bool odd = false;
    int z = 10;
    for (int y = round(grid_num * 0.3); y < grid_num - 1; ++y) {
        for (int x = 1; x < grid_num - 1; ++x) {
            if (x == 1) {
                z = odd ? 10 : 15;
            }
            if (x % z == 0 && y % z == 0) {
                if (x == 10 || x == 15) {
                    odd = !odd;
                }
                for (int i = x - radius; i <= x + radius; ++i) {
                    for (int j = y - radius; j <= y + radius; ++j) {
                        float r = pow(i - x, 2) + pow(j - y, 2);
                        if (r <= pow(radius, 2)) {
                            solidIndices.push_back(i + j * grid_num);
                        }
                    }
                }
            }
        }
    }


    conf.setGridSize(1).setParticlesPerGrid(4).setGridNum(grid_num)
            .setPxPerCell(4).setSimulationSteps(1000)
            .setWaterIndices(waterIndices).setAdditionalWater(additionalWater, 50, 100).setAdditionalSolidIndices(
                    solidIndices);
}

void circleWall(Configuration &conf, int grid_num) {
    std::vector<std::pair<int, int>> waterIndices = {};


    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (j >= 2 && j <= round(grid_num * 0.2) && (i >= round(grid_num * 0.3)) && (i < grid_num - 2)) {
                waterIndices.emplace_back(i, j);
            }
        }
    }
    std::vector<std::pair<int, int>> additionalWater = waterIndices;

    std::vector<int> solidIndices = {};
    double radius = 20;
    int center_x = grid_num / 2;
    int center_y = grid_num / 1.5;
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            float r = pow(i - center_x, 2) + pow(j - center_y, 2);
            if (r <= pow(radius, 2) && r >= pow(radius - 3, 2) && j >= center_y - radius + 4) {
                solidIndices.push_back(i + j * grid_num);
            }
        }
    }

    conf.setGridSize(1).setParticlesPerGrid(4).setGridNum(grid_num)
            .setPxPerCell(4).setSimulationSteps(1000)
            .setWaterIndices(waterIndices).setAdditionalWater(additionalWater, 50, 100).setAdditionalSolidIndices(
                    solidIndices);
}

void smallHoleWall(Configuration &conf, int grid_num) {
    std::vector<std::pair<int, int>> waterIndices = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (j <= round(grid_num * 0.3) && (i < round(grid_num * 0.3))) {
                waterIndices.emplace_back(i, j);
            }
        }
    }

    vector<int> solid_indices = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {

            if (j > round(grid_num * 0.4) && i <= round(grid_num * 0.7) && j <= round(grid_num * 0.5) && i <= j) {
                solid_indices.push_back(i + j * grid_num);
            }

            if (j > round(grid_num * 0.4) && i >= round(grid_num * 0.52) && j <= round(grid_num * 0.5)) {
                solid_indices.push_back(i + j * grid_num);
            }
        }
    }
    conf.setGridSize(1).setParticlesPerGrid(4).setGridNum(grid_num)
            .setPxPerCell(3).setSimulationSteps(1000)
            .setWaterIndices(waterIndices).setAdditionalSolidIndices(solid_indices);
}

void stair(Configuration &conf, int grid_num) {
    std::vector<std::pair<int, int>> waterIndices = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (j <= round(grid_num * 0.3) && (i < round(grid_num * 0.3))) {
                waterIndices.emplace_back(i, j);
            }
        }
    }

    vector<int> solid_indices = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {

            if (j > round(grid_num * 0.5) && i <= 0.5 * j) {
                solid_indices.push_back(i + j * grid_num);
            }
        }
    }
    conf.setGridSize(1).setParticlesPerGrid(4).setGridNum(grid_num)
            .setPxPerCell(3).setSimulationSteps(1000)
            .setWaterIndices(waterIndices).setAdditionalSolidIndices(solid_indices);
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

int twoVerticalLinesWithCenterWall(Configuration &conf, int grid_num) {
    std::vector<std::pair<int, int>> waterIndices = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (j >= 2 && j <= grid_num - 3 &&
                ((i >= 2 && i < round(grid_num * 0.45) && (j > round(grid_num * 0.5))))) {
                waterIndices.emplace_back(i, j);
            }
        }
    }

    vector<int> solid_indices = {};
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {

            if (j > round(grid_num * 0.5) && (j < round(grid_num * 0.99)) &&
                ((i > round(grid_num * 0.45) && i < round(grid_num * 0.55)))) {
                solid_indices.push_back(i + j * grid_num);
            }
        }
    }

    std::vector<std::pair<int, int>> additionalIndices = {};

//    for (int i = 1; i < grid_num - 1; ++i) {
//        for (int j = 1; j < grid_num - 1; ++j) {
//
//            if (j < round(grid_num * 0.4) && ((i > round(grid_num * 0.3) && i < round(grid_num * 0.655)))) {
//                additionalIndices.emplace_back(i, j);
//            }
//        }
//    }


    conf.setGridSize(1).setParticlesPerGrid(4).setGridNum(grid_num)
            .setPxPerCell(3).setSimulationSteps(1000)
            .setWaterIndices(waterIndices).setAdditionalSolidIndices(solid_indices).setAdditionalWater(
                    additionalIndices, 50, 50);

}