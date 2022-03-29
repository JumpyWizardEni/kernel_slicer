#include "Configuration.h"
#include <cmath>
#include <iostream>

//"Случайное" вещ. число в пределах от min до max
double Configuration::randfrom(double min, double max) {
    double range = (max - min);
    double div = RAND_MAX / range;
    return min + (rand() / div);
}


void Configuration::start() {
    solver = new Solver();
    renderer = new SimpleRenderer(px_per_cell, grid_num);
    fillSolverData();
    simulate();
}

void Configuration::fillSolverData() {
    double dx = (double) grid_size / grid_num;
    renderer->dx = dx;
    vector<Particle> particles;
    int particles_size = 0;
    for (int i = 0; i < water_indices.size(); ++i) {
        for (int k = 0; k < particles_per_grid; ++k) {
            double r1 = randfrom(0.4, 0.6);
            double r2 = randfrom(0.4, 0.6);
            Particle p = Particle();
            p.pos_x = water_indices[i].first * dx + dx * r1;
            p.pos_y = water_indices[i].second * dx + dx * r2;
            particles.push_back(p);
        }
        particles_size += particles_per_grid;

    }

    vector<double> vx(grid_num * (grid_num + 1), 0);
    vector<double> vy((grid_num + 1) * grid_num, 0);
    vector<double> pressure(grid_num * grid_num, 0);

    solver->size = grid_num;
    solver->particles_size = particles_size;
    solver->particles = particles;
    solver->dx = dx;
    solver->vx = vx;
    solver->vy = vy;
    solver->pressure = pressure;
    solver->solid_indices = std::move(solid_indices);

    solver->setParameters();
}


void Configuration::simulate() {
    double dx = (double) grid_size / grid_num;
    solver->createSpaceTypes();
//    renderer->saveImage("images/" + std::to_string(1) + ".jpeg", solver->spaceTypes, solver->particles,
//                        RenderMode::Blobbies);
    for (int frameNum = 1; frameNum < simulation_steps; ++frameNum) {
//        if (frameNum % 20 == 0) {
//        }
        if (additional_fluid) {

            if (frameNum % frequency == 0 || frameNum == first) {
                for (int i = 0; i < additional_fluid_indices.size(); ++i) {
                    for (int k = 0; k < particles_per_grid; ++k) {
                        double r1 = randfrom(0.0, 1.0);
                        double r2 = randfrom(0.0, 1.0);
                        Particle p = Particle();
                        p.pos_x = additional_fluid_indices[i].first * dx + dx * r1;
                        p.pos_y = additional_fluid_indices[i].second * dx + dx * r2;
                        solver->particles.push_back(p);
                    }
                    solver->particles_size += particles_per_grid;

                }
            }
        }
        std::cout << "Current frame: " + std::to_string(frameNum) << std::endl;
        float t = 0;
        float t_frame = 1.0 / 30;
//        while (t < t_frame) {
            solver->performStep();
//            t += solver->dt;
//        }
//        solver->changeParticlesNum();
        if (frameNum % 30 == 0) {
//            solver->deleteUnnecessaryParticles();
        }
        renderer->saveImage("images/" + std::to_string(frameNum + 1) + ".jpeg", solver->spaceTypes, solver->particles,
                            RenderMode::Square);
    }

}

Configuration::~Configuration() {
    delete solver;
    delete renderer;
}

Configuration &Configuration::setGridSize(int value) {
    grid_size = value;
    return *this;
}

Configuration &Configuration::setPxPerCell(int value) {
    px_per_cell = value;
    return *this;
}

Configuration &Configuration::setGridNum(int value) {
    grid_num = value;
    return *this;
}

Configuration &Configuration::setSimulationSteps(int value) {
    simulation_steps = value;
    return *this;
}

Configuration &Configuration::setParticlesPerGrid(int value) {
    particles_per_grid = value;
    return *this;
}

Configuration &Configuration::setWaterIndices(vector<std::pair<int, int>> &indices) {
    water_indices = std::move(indices);
    return *this;
}

Configuration &Configuration::setAdditionalSolidIndices(vector<int> &indices) {
    solid_indices = std::move(indices);
    return *this;
}

Configuration &Configuration::setAdditionalWater(vector<std::pair<int, int>> &indices, int frequency, int first) {
    additional_fluid = true;
    this->first = first;
    additional_fluid_indices = std::move(indices);
    this->frequency = frequency;
    return *this;
}
