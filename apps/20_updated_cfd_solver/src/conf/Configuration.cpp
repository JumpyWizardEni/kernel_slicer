#include "Configuration.h"
#include <cmath>
#include <iostream>
#include <src/test_class_generated.h>
#include <chrono>

#define TEST_MODE false

//"Случайное" вещ. число в пределах от min до max
double Configuration::randfrom(double min, double max) {
    double range = (max - min);
    double div = RAND_MAX / range;
    return min + (rand() / div);
}

std::shared_ptr<Solver> CreateSolver_Generated(vk_utils::VulkanContext a_ctx, size_t a_maxThreadsGenerated);

void Configuration::start() {
#ifndef NDEBUG
    bool enableValidationLayers = true;
#else
    bool enableValidationLayers = false;
#endif
    renderer = new SimpleRenderer(px_per_cell, grid_num, grid_size);

    if (TEST_MODE) {
        auto ctx = vk_utils::globalContextGet(enableValidationLayers, 0);
        solver = CreateSolver_Generated(ctx,grid_num);
        solver_cpu = std::make_shared<Solver>();
        fillSolverData();
        solver_cpu->particles.reserve(particles_size);
        solver_cpu->setParameters(grid_num, dx, solid_indices);
        solver->CommitDeviceData();
        solver_cpu->CommitDeviceData();

    } else {
        if (mode == GPU) {
            auto ctx = vk_utils::globalContextGet(enableValidationLayers, 0);
            solver = CreateSolver_Generated(ctx, grid_num);
        } else {
            solver = std::make_shared<Solver>();

        }

        fillSolverData();

        solver->CommitDeviceData();

    }
    simulate();

}

void Configuration::fillSolverData() {
    dx = (double) grid_size / grid_num;
    spaceTypes.resize(grid_num * grid_num, 0);
    particles_size = 0;
    for (int i = 0; i < water_indices.size(); ++i) {
        for (int k = 0; k < particles_per_grid; ++k) {
            double r1 = randfrom(0.4, 0.6);
            double r2 = randfrom(0.4, 0.6);
            Solver::Particle p = Solver::Particle();
            p.pos_x = water_indices[i].first * dx + dx * r1;
            p.pos_y = water_indices[i].second * dx + dx * r2;
            particles.push_back(p);
        }
        particles_size += particles_per_grid;

    }

    spaceTypes.resize(grid_num, grid_num);
    solver->particles.reserve(particles_size);
    solver->setParameters(grid_num, dx, solid_indices);
}


void Configuration::simulate() {
    for (int frameNum = 1; frameNum < simulation_steps; ++frameNum) {

//        if (additional_fluid) {
//
//            if (frameNum % frequency == 0 || frameNum == first) {
//                for (int i = 0; i < additional_fluid_indices.size(); ++i) {
                    for (int k = 0; k < particles_per_grid; ++k) {
                        double r1 = randfrom(0.0, 1.0);
                        double r2 = randfrom(0.0, 1.0);
                        Solver::Particle p = Solver::Particle();
                        p.pos_x = 0.5;
                        p.pos_y = 0.5;
//                        p.pos_x = additional_fluid_indices[i].first * dx + dx * r1;
//                        p.pos_y = additional_fluid_indices[i].second * dx + dx * r2;
                        particles.push_back(p);
                    }
                    particles_size += particles_per_grid;

//                }
//            }
//        }
        std::cout << "Current frame: " + std::to_string(frameNum) << std::endl;
        double t = 0;
        double t_frame = 1.0 / 60;
        vector<float> vx_output(grid_num * (grid_num + 1));
        vector<float> vy_output(grid_num * (grid_num + 1));

        while (t < t_frame) {
            if (TEST_MODE) {
                std::cout << "CPU: " << std::endl;
                std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                solver_cpu->performStep(particles_size, particles.data(), particles.data());
                std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
                std::cout << "Perform time  = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
            }
            if (mode == GPU) {
                std::cout << "GPU: " << std::endl;
            } else {
                std::cout << "CPU: " << std::endl;
            }
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            solver->performStep(particles_size, particles.data(), particles.data());
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::cout << "Perform time  = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
            t += solver->dt;
        }
        if (TEST_MODE) {
//            checkValues(particles);
        }
        countSpaceTypes();
        renderer->saveImage("images/" + std::to_string(frameNum + 1) + ".jpeg", spaceTypes, particles,
                            RenderMode::Blobbies);
    }

}

Configuration::~Configuration() {
    solver = nullptr;
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

void Configuration::countSpaceTypes() {
    for (int i = 0; i < grid_num * grid_num; ++i) {
        spaceTypes[i] = Empty;
    }
    for (int i = 0; i < particles_size; ++i) {
        int x = roundValue(0, grid_num - 1, particles[i].pos_x / dx);
        int y = roundValue(0, grid_num - 1, particles[i].pos_y / dx);
        spaceTypes[x + grid_num * y] = Fluid;
    }
    for (int j = 0; j < grid_num; ++j) {
        for (int i = 0; i < grid_num; ++i) {
            if (i == 0 || i == grid_num - 1 || j == 0 || j == grid_num - 1) {
                spaceTypes[i + grid_num * j] = Solid;
            }
        }
    }
    for (auto i: solid_indices) {
        spaceTypes[i] = Solid;
    }
}

int Configuration::roundValue(int from, int to, double value) {
    if (value > to) {
        value = to;
    }
    if (value < from) {
        value = from;
    }

    int f = (int) floor(value);
    if (value - (double) f > 0.5) {
        return (int) ceil(value);
    }
    return f;
}

void Configuration::checkValues(vector<Solver::Particle> &particles) {
    std::vector<Solver::Particle> &cpu_particles = solver_cpu->particles;
    for (int i = 0; i < particles_size; ++i) {
        Particle &p1 = particles[i];
        Particle &p2 = cpu_particles[i];
        if (std::abs(p1.vx - p2.vx) > TOL || std::abs(p1.vy - p2.vy) > TOL || std::abs(p1.pos_x - p2.pos_x) > TOL ||
            std::abs(p1.pos_y - p2.pos_y) > TOL) {
            std::cout << "Check failed: " << "vx: " << p1.vx << ", " << p2.vx<<", vy: " << p1.vy << ", " << p2.vy << ", pos_x: ";
            std::cout << p1.pos_x << ", " << p2.pos_x<< ", pos_y: " << p1.pos_y << ", " << p2.pos_y<<std::endl;
        }
    }
}
