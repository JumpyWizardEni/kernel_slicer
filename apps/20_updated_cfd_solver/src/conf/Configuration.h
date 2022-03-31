#ifndef TEST_CONFIGURATION_H
#define TEST_CONFIGURATION_H


#include "src/render/SimpleRenderer.h"

class Configuration {
private:
    int grid_size = 1;
    int px_per_cell = 5;
    int grid_num = 150;
    int simulation_steps = 100;
    int particles_per_grid = 4;
    int frequency = 0;
    bool additional_fluid = false;
    int first = 0;
    vector<std::pair<int, int>> additional_fluid_indices = {};
    vector<int> solid_indices = {};
    vector<std::pair<int, int>> water_indices = {};
    IRenderer *renderer = nullptr;
    Solver *solver = nullptr;
    void fillSolverData();
    void simulate();
    double randfrom(double min, double max);
public:
    void start();
    Configuration &setGridSize(int value);
    Configuration &setPxPerCell(int value);
    Configuration &setGridNum(int value);
    Configuration &setSimulationSteps(int value);
    Configuration &setParticlesPerGrid(int value);

    ~Configuration();

    Configuration &setWaterIndices(vector<std::pair<int, int>> &indices);


    Configuration &setAdditionalSolidIndices(vector<int> &indices);

    Configuration &setAdditionalWater(vector<std::pair<int, int>> &indices, int frequency, int first);
};


#endif //TEST_CONFIGURATION_H
