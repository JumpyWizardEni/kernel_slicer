#ifndef TEST_CONFIGURATIONBUILDER_H
#define TEST_CONFIGURATIONBUILDER_H
#include <functional>
#include <map>
#include "Configuration.h"


struct ConfParams {
    int grid_size = 1;
    int px_per_cell = 5;
    int grid_num = 150;
    int simulation_steps = 100;
    int particles_per_grid = 4;
    int frequency = 0;
    int first_add_fluid = 0;

    ConfParams(int grid_size, int px_per_cell, int grid_num, int simulation_steps,
               int particles_per_grid, int frequency,
               int first_add_fluid) : grid_size(grid_size), px_per_cell(px_per_cell),
                                      grid_num(grid_num), simulation_steps(simulation_steps),
                                      particles_per_grid(particles_per_grid), frequency(frequency),
                                      first_add_fluid(first_add_fluid) {
    };

    ConfParams() = default;
};

using f = std::function<void(Configuration &a1, ConfParams &a2)>;

enum class ConfigurationName {
    TwoVerticals, BasketSolid, AddWater, CircleWater, Stair, SmallHole, CircleWall, CircleGridSolid, Simple
};

class ConfigurationBuilder {
private:
    std::map<ConfigurationName, f> configurations = std::map<ConfigurationName, f>();
    ConfParams params = ConfParams();

    static void twoVerticalLinesWithCenterWall(Configuration &conf, ConfParams &params);

    static void basketSolid(Configuration &conf, ConfParams &params);

    static void solidWithAddingWater(Configuration &conf, ConfParams &params);

    static void circleWater(Configuration &conf, ConfParams &params);

    static void stair(Configuration &conf, ConfParams &params);

    static void smallHole(Configuration &conf, ConfParams &params);

    static void circleWall(Configuration &conf, ConfParams &params);

    static void circleGridSolid(Configuration &conf, ConfParams &params);

    static void simple(Configuration &conf, ConfParams &params);

    static void setParametersToConf(Configuration &conf, vector<std::pair<int, int>> &waterIndices,
                                    vector<std::pair<int, int>> &additionalWaterIndices,
                                    vector<int> &solidIndices, ConfParams &params);


public:

    explicit ConfigurationBuilder(ConfParams);

    void build(Configuration &conf, ConfigurationName name);
};


#endif //TEST_CONFIGURATIONBUILDER_H
