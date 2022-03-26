#include "test_class.h"
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include "SimpleRenderer.h"

using std::vector;

const double GRID_SIZE = 1;
const double PX_PER_CELL = 5;
const int GRID_NUM = 20;
const int SIMULATION_STEPS = 500;


double randfrom(double min, double max);

void fillSolverData(Solver &solver);

void simulate(Solver &solver, SimpleRenderer &renderer);

int main() {
    Solver solver = Solver();

    SimpleRenderer renderer = SimpleRenderer(PX_PER_CELL, GRID_NUM);

    fillSolverData(solver);

    simulate(solver, renderer);

    return 0;
}

void simulate(Solver &solver, SimpleRenderer &renderer) {
    renderer.saveImage("images/" + std::to_string(0) + ".jpeg", solver.spaceTypes, solver.particles,
                       RenderMode::Square);
    for (int frameNum = 1; frameNum < SIMULATION_STEPS; ++frameNum) {
        if (frameNum % 40 == 0) {
            for (int i = 0; i < GRID_NUM; ++i) {
                for (int j = 0; j < GRID_NUM; ++j) {
                    if (!(j == 0 || i == 0 || j == GRID_NUM - 1 || i == GRID_NUM - 1)) {


                        //Остальные заполняем случайно
                        if (i > round(GRID_NUM * 0.2) && i < round(GRID_NUM * 0.4) && j > round(GRID_NUM * 0.2) &&
                            j < round(GRID_NUM * 0.6)) {
                            for (int k = 0; k < 4; ++k) {
                                double r1 = randfrom(0.0, 1.0);
                                double r2 = randfrom(0.0, 1.0);
                                Particle p = Particle();
                                p.pos_x = i * solver.dx + solver.dx * r1;
                                p.pos_y = j * solver.dx + solver.dx * r2;
                                solver.particles.push_back(p);
                            }


                            solver.particles_size += 4;
                        }
                    }
                }
            }
        }
        std::cout << "Current frame: " + std::to_string(frameNum) << std::endl;
        solver.performStep();
        renderer.saveImage("images/" + std::to_string(frameNum + 1) + ".jpeg", solver.spaceTypes, solver.particles,
                           RenderMode::Square);
    }

}

//"Случайное" вещ. число в пределах от min до max
double randfrom(double min, double max) {
    double range = (max - min);
    double div = RAND_MAX / range;
    return min + (rand() / div);
}

void fillSolverData(Solver &solver) {
    float dx = (float) GRID_SIZE / GRID_NUM;
    vector<Particle> particles;
    int particles_size = 0;
    for (int i = 0; i < GRID_NUM; ++i) {
        for (int j = 0; j < GRID_NUM; ++j) {
            if (!(j == 0 || i == 0 || j == GRID_NUM - 1 || i == GRID_NUM - 1 )) {


                //Остальные заполняем случайно
                if (j > round(GRID_NUM * 0.4)) {
                    for (int k = 0; k < 4; ++k) {
                        double r1 = randfrom(0.0, 1.0);
                        double r2 = randfrom(0.0, 1.0);
                        Particle p = Particle();
                        p.pos_x = i * dx + dx * r1;
                        p.pos_y = j * dx + dx * r2;
                        particles.push_back(p);
                    }


                    particles_size += 4;
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
    solver.particles_size = particles_size;
    vector<float> vy((GRID_NUM + 1) * GRID_NUM);
    for (int i = 1; i < GRID_NUM - 1; ++i) {
        for (int j = 1; j < GRID_NUM; j++) {
            vy[solver.getIdxY(i, j)] = 0;
        }
    }
    solver.particles = particles;
    solver.dx = dx;
    solver.vx = vx;
    solver.vy = vy;
    solver.pressure = pressure;

    solver.setParameters();
}

