#include "test_class.h"
#include <string>
#include <vector>
#include <iostream>
#include "SimpleRenderer.h"

using std::vector;

const char GRID_SIZE = 5;
const int GRID_NUM = 64;
const int SIMULATION_STEPS = 10;



double randfrom(double min, double max);

void fillSolverData(Solver &solver);

void simulate(Solver &solver, SimpleRenderer &renderer);

int main() {
    Solver solver = Solver();
    SimpleRenderer renderer = SimpleRenderer(GRID_SIZE, GRID_NUM);

    fillSolverData(solver);

    simulate(solver, renderer);

    return 0;
}

void simulate(Solver &solver, SimpleRenderer &renderer) {
    for (int i = 0; i < SIMULATION_STEPS; ++ i) {
        std::cout << "Current frame: " + std::to_string(i) <<  std::endl;
        solver.performStep();
        renderer.saveImage("images/" + std::to_string(i) + ".jpeg", solver.spaceTypes);
    }
}

//"Случайное" вещ. число в пределах от min до max
double randfrom(double min, double max) {
    double range = (max - min);
    double div = RAND_MAX / range;
    return min + (rand() / div);
}

void fillSolverData(Solver &solver) {
    vector<SpaceType> spaceTypes(GRID_NUM * GRID_NUM);
    for (int i = 0; i < GRID_NUM; ++i) {
        for (int j = 0; j < GRID_NUM; ++j) {
            if (j == 0 || i == 0 || j == GRID_NUM - 1 || i == GRID_NUM - 1) {
                spaceTypes[GRID_NUM * i + j] = SpaceType::Solid; // На границе - стена
            } else {
                double r = randfrom(0, 1);
                //Остальные заполняем случайно
                if (r < 0.4) {
                    spaceTypes[GRID_NUM * i + j] = SpaceType::Empty;
                } else {
                    spaceTypes[GRID_NUM * i + j] = SpaceType::Fluid;
                }
            }
        }
    }

    vector<float> vx((GRID_NUM + 1) * GRID_NUM);
    for (int i = 0; i < GRID_NUM * (GRID_NUM + 1); ++i)
        vx[i] = 0;

    vector<float> vy(GRID_NUM * (GRID_NUM + 1));
    for (int i = 0; i < GRID_NUM * (GRID_NUM + 1); ++i)
        vy[i] = 0;

    vector<float> pressure(GRID_NUM * GRID_NUM);
    for (int i = 0; i < GRID_NUM * (GRID_NUM); ++i)
        pressure[i] = randfrom(-1, 1);

    solver.size = GRID_NUM;
    solver.vx = vx;
    solver.vy = vy;
    solver.spaceTypes = spaceTypes;
    solver.pressure = pressure;
}

