#include "test_class.h"
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include "SimpleRenderer.h"

using std::vector;

const double GRID_SIZE = 1;
const int GRID_NUM = 100;
const int SIMULATION_STEPS = 100;



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
    renderer.saveImage("images/" + std::to_string(0) + ".jpeg", solver.spaceTypes);
    for (int i = 0; i < SIMULATION_STEPS; ++ i) {
        std::cout << "Current frame: " + std::to_string(i) <<  std::endl;
        solver.performStep();
        renderer.saveImage("images/" + std::to_string(i + 1) + ".jpeg", solver.spaceTypes);
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
                //Остальные заполняем случайно
                if (i < round(GRID_NUM * 0.3)) {
                    spaceTypes[GRID_NUM * i + j] = SpaceType::Empty;
                } else {
                    spaceTypes[GRID_NUM * i + j] = SpaceType::Fluid;
                }
            }
        }
    }

    vector<float> vx(GRID_NUM * (GRID_NUM + 1));
    for (int i = 0; i < GRID_NUM * (GRID_NUM + 1); ++i)
        vx[i] = 0;



    vector<float> pressure(GRID_NUM * GRID_NUM);
    for (int i = 0; i < GRID_NUM * (GRID_NUM); ++i)
        pressure[i] = 0;

    solver.size = GRID_NUM;
    vector<float> vy((GRID_NUM + 1) * GRID_NUM);
    for (int i = 1; i < GRID_NUM - 1; ++i) {
        for (int j =  1; j < GRID_NUM; j++) {
            vy[solver.getIdxY(i, j)] = 0;
        }
    }
    solver.dx = (float) GRID_SIZE / GRID_NUM;
    solver.vx = vx;
    solver.vy = vy;
    solver.spaceTypes = spaceTypes;
    solver.spaceTypesOld = spaceTypes;
    solver.pressure = pressure;

    solver.setParameters();
}

