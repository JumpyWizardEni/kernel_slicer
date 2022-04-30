//
// Created by timofeq on 31.03.2022.
//

#include <cmath>
#include "ConfigurationBuilder.h"

ConfigurationBuilder::ConfigurationBuilder(ConfParams newParams) {
    params = newParams;
    configurations[ConfigurationName::TwoVerticals] = twoVerticalLinesWithCenterWall;
    configurations[ConfigurationName::BasketSolid] = basketSolid;
    configurations[ConfigurationName::AddWater] = solidWithAddingWater;
    configurations[ConfigurationName::CircleWater] = circleWater;
    configurations[ConfigurationName::Stair] = stair;
    configurations[ConfigurationName::SmallHole] = smallHole;
    configurations[ConfigurationName::CircleWall] = circleWall;
    configurations[ConfigurationName::CircleGridSolid] = circleGridSolid;
    configurations[ConfigurationName::Simple] = simple;

}

void ConfigurationBuilder::build(Configuration &conf, ConfigurationName name) {
    configurations[name](conf, params);
}

void ConfigurationBuilder::setParametersToConf(Configuration &conf, vector<std::pair<int, int>> &waterIndices,
                                               vector<std::pair<int, int>> &additionalWaterIndices,
                                               vector<int> &solidIndices, ConfParams &params) {
    conf.setGridSize(params.grid_size).setPxPerCell(params.px_per_cell).setGridNum(params.grid_num).setSimulationSteps(
            params.simulation_steps).setParticlesPerGrid(params.particles_per_grid).
            setWaterIndices(waterIndices).setAdditionalWater(additionalWaterIndices, params.frequency,
                                                             params.first_add_fluid).setAdditionalSolidIndices(
            solidIndices);
}


void ConfigurationBuilder::twoVerticalLinesWithCenterWall(Configuration &conf, ConfParams &params) {
    std::vector<std::pair<int, int>> waterIndices = {};
    int grid_num = params.grid_num;
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (j >= 2 && j <= grid_num - 3 &&
                ((i >= 2 && i < round(grid_num * 0.45) && (j > round(grid_num * 0.5))))) {
                waterIndices.emplace_back(i, j);
            }
        }
    }

    vector<int> solid_indices = {};
//    for (int i = 1; i < grid_num - 1; ++i) {
//        for (int j = 1; j < grid_num - 1; ++j) {
//
//            if (j > round(grid_num * 0.5) && (j < round(grid_num * 0.99)) &&
//                ((i > round(grid_num * 0.45) && i < round(grid_num * 0.55)))) {
//                solid_indices.push_back(i + j * grid_num);
//            }
//        }
//    }

    std::vector<std::pair<int, int>> additionalIndices = {};

//    for (int i = 1; i < grid_num - 1; ++i) {
//        for (int j = 1; j < grid_num - 1; ++j) {
//
//            if (j < round(grid_num * 0.4) && ((i > round(grid_num * 0.3) && i < round(grid_num * 0.655)))) {
//                additionalIndices.emplace_back(i, j);
//            }
//        }
//    }

    setParametersToConf(conf, waterIndices, additionalIndices, solid_indices, params);
}

void ConfigurationBuilder::basketSolid(Configuration &conf, ConfParams &params) {
    std::vector<std::pair<int, int>> waterIndices = {};
    int grid_num = params.grid_num;
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

    std::vector<std::pair<int, int>> additionalWater = {};

    setParametersToConf(conf, waterIndices, additionalWater, solid_indices, params);
}

void ConfigurationBuilder::solidWithAddingWater(Configuration &conf, ConfParams &params) {
    std::vector<std::pair<int, int>> waterIndices = {};
    int grid_num = params.grid_num;
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

    vector<int> solid_indices = {};

    setParametersToConf(conf, waterIndices, additionalWater, solid_indices, params);
}

void ConfigurationBuilder::circleWater(Configuration &conf, ConfParams &params) {
    std::vector<std::pair<int, int>> waterIndices = {};
    int radius = 10;
    int grid_num = params.grid_num;
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
    vector<int> solid_indices = {};
    setParametersToConf(conf, waterIndices, additionalWater, solid_indices, params);
}


void ConfigurationBuilder::stair(Configuration &conf, ConfParams &params) {
    std::vector<std::pair<int, int>> waterIndices = {};
    int grid_num = params.grid_num;
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

    std::vector<std::pair<int, int>> additionalWater = {};
    setParametersToConf(conf, waterIndices, additionalWater, solid_indices, params);

}


void ConfigurationBuilder::smallHole(Configuration &conf, ConfParams &params) {
    std::vector<std::pair<int, int>> waterIndices = {};
    int grid_num = params.grid_num;
    for (int i = 1; i < grid_num - 1; ++i) {
        for (int j = 1; j < grid_num - 1; ++j) {
            if (j <= round(grid_num * 0.3) && (i < round(grid_num * 0.3))) {
                waterIndices.emplace_back(i, j);
            }
        }
    }
    std::vector<std::pair<int, int>> additionalWater = {};
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

    setParametersToConf(conf, waterIndices, additionalWater, solid_indices, params);
}

void ConfigurationBuilder::circleWall(Configuration &conf, ConfParams &params) {
    std::vector<std::pair<int, int>> waterIndices = {};
    int grid_num = params.grid_num;

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

    setParametersToConf(conf, waterIndices, additionalWater, solidIndices, params);
}


void ConfigurationBuilder::circleGridSolid(Configuration &conf, ConfParams &params) {
    std::vector<std::pair<int, int>> waterIndices = {};
    double radius = 3;
    int grid_num = params.grid_num;
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
    setParametersToConf(conf, waterIndices, additionalWater, solidIndices, params);
}

void ConfigurationBuilder::simple(Configuration &conf, ConfParams &params) {
    std::vector<std::pair<int, int>> waterIndices = {};
    int grid_num = params.grid_num;
    for (int x = 1; x < grid_num - 1; ++x) {
        for (int y = 1; y < grid_num - 1; ++y) {
            if (x >= round(grid_num * 0.1) && x <= round (grid_num * 0.9) && y >= round(grid_num * 0.1) && y <= round (grid_num * 0.7))   {
                waterIndices.emplace_back(x, y);
            }
        }
    }
    std::vector<std::pair<int, int>> additionalWater = {};

    std::vector<int> solidIndices = {};
    setParametersToConf(conf, waterIndices, additionalWater, solidIndices, params);
}
