#include "test_class.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <map>

using std::vector;

Solver::Solver() = default;

void Solver::setParameters() {
    pressure.resize(size * size, 0);
    gridInfo.resize(size * size, GridPICInfo());
    spaceTypes.resize(size * size, SpaceType::Empty);
    pressureResidual.resize(size * size, 0);
    rhs.resize(size * size, 0);
    press_diag.resize(size * size, 0);
    pressX.resize(size * size, 0);
    pressY.resize(size * size, 0);
    preconditioner.resize(size * size, 0);
    q.resize(size * size, 0);
    z.resize(size * size, 0);
    s.resize(size * size, 0);
    diff_vx.resize(size * size, 0);
    diff_vy.resize(size * size, 0);
    mask.resize(size * size, 0);
    prev_vx.resize(size * (size + 1), 0);
    prev_vy.resize(size * (size + 1), 0);
}

int Solver::cutValue(double from, double to, double value) {
    if (value > to) {
        value = to;
    }
    if (value < from) {
        value = from;
    }
    return (int) std::round(value);
}


int Solver::roundValue(int from, int to, double value) {
    if (value > to) {
        value = to;
    }
    if (value < from) {
        value = from;
    }

    return (int) std::round(value);
}

double Solver::getVelocityX(double *vx, int i, int j) {
    double res = (vx[getIdxX(i, j)] + vx[getIdxX(i + 1, j)]) / 2;

    return res;
}

double Solver::getVelocityY(double *vy, int i, int j) {
    double res = (vy[getIdxY(i, j)] + vy[getIdxY(i, j + 1)]) / 2;

    return res;

}

double Solver::randfrom1(double min, double max) {
    double range = (max - min);
    double div = RAND_MAX / range;
    return min + (rand() / div);
}

void Solver::performStep() {
    resetParams();
    assembleGridFromParticles();
    countTimeDelta(vx.data(), vy.data());
    prev_vx = vx;
    prev_vy = vy;
    addForces(vy.data(), g);
    dirichleCondition();
//    extrapolateVelocities();
    project();
    checkDivergence();
    dirichleCondition();
    advectParticles();
    createSpaceTypes();
//    int c = 0;
//    for (auto &p: particles) {
//        if (floorValue(1, size - 2, p.pos_x / dx) == 1 && roundValue(1, size - 2, p.pos_y / dx) == size - 2) {
//            c++;
//        }
//    }
//    std::cout << "c: " << c << std::endl;
}


//Считаем dt на текущем шаге, dt <= 5 * dx / max(velocity)
void Solver::countTimeDelta(const double *p_vx, const double *p_vy) {
    //Ищем максимум
    double maximum = -1;
    for (int i = 0; i < size * (size + 1); ++i) {
        double vel = p_vx[i] * p_vx[i] + p_vy[i] * p_vy[i];
        if (vel > maximum) {
            maximum = vel;
        }
    }
    maximum = std::sqrt(maximum);
    maximum += std::sqrt(5 * dx * std::abs(g)); // чтобы не было деления на 0
    dt = 5 * dx / maximum;
}

//Добавляем силу тяжести
void Solver::addForces(double *v, double a) {
    for (int j = 1; j < size; ++j) {
        for (int i = 1; i < size - 1; ++i) {
            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                v[getIdxY(i, j)] += dt * a;
            }
        }
    }
}

double Solver::interpolate(double q, double *q_copy, double x, double y, int i, int j) {
    double w_x = (ceil(x) - x) * dx;
    double w_y = (ceil(y) - y) * dx;

    //Кубическая интерполяция

    //По x
    double w_x_2 = w_x * w_x;
    double w_x_3 = w_x * w_x_2;
    double w0 = -w_x / 3 + w_x_2 / 3 - w_x_3 / 6;
    double w1 = 1 - w_x_2 + (w_x_3 - w_x) / 2;
    double w2 = w_x + (w_x_2 - w_x_3) / 2;
    double w3 = (w_x_3 - w_x) / 6;

    double q_j_1 = w0 * q_copy[getIdx(i - 1, j - 1)] + w1 * q_copy[getIdx(i, j - 1)] +
                   w2 * q_copy[getIdx(i + 1, j - 1)] + w3 * q_copy[getIdx(i + 2, j - 1)];

    double q_j = w0 * q_copy[getIdx(i - 1, j)] + w1 * q_copy[getIdx(i, j)] +
                 w2 * q_copy[getIdx(i + 1, j)] + w3 * q_copy[getIdx(i + 2, j)];

    double q_j1 = w0 * q_copy[getIdx(i - 1, j + 1)] + w1 * q_copy[getIdx(i, j + 1)] +
                  w2 * q_copy[getIdx(i + 1, j + 1)] + w3 * q_copy[getIdx(i + 2, j + 1)];

    double q_j2 = w0 * q_copy[getIdx(i - 1, j + 2)] + w1 * q_copy[getIdx(i, j + 2)] +
                  w2 * q_copy[getIdx(i + 1, j + 2)] + w3 * q_copy[getIdx(i + 2, j + 2)];

    //По y
    double w_y_2 = w_y * w_y;
    double w_y_3 = w_y * w_y_2;
    w0 = -w_y / 3 + w_y_2 / 3 - w_y_3 / 6;
    w1 = 1 - w_y_2 + (w_y_3 - w_y) / 2;
    w2 = w_y + (w_y_2 - w_y_3) / 2;
    w3 = (w_y_3 - w_y) / 6;
    q = w0 * q_j_1 + w1 * q_j + w2 * q_j1 + w3 * q_j2;
    return q;
}


//solve
void Solver::project() {
    //calc rhs for Ap = rhs
    calcNegativeDivergence();
    //set A
    fillPressureMatrix();
    //solve Ap = rhs
    PCG();
    //Compute new velocities
    updateVelocities();

};

void Solver::calcNegativeDivergence() {

    double scale = 1 / dx;
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                rhs[getIdx(i, j)] =
                        -scale *
                        (vx[getIdxX(i + 1, j)] - vx[getIdxX(i, j)] + vy[getIdxY(i, j + 1)] - vy[getIdxY(i, j)]);
                if (spaceTypes[getIdx(i - 1, j)] == SpaceType::Solid) {
                    rhs[getIdx(i, j)] -= scale * vx[getIdxX(i, j)];
                }
                if (spaceTypes[getIdx(i + 1, j)] == SpaceType::Solid) {
                    rhs[getIdx(i, j)] += scale * vx[getIdxX(i + 1, j)];
                }
                if (spaceTypes[getIdx(i, j - 1)] == SpaceType::Solid) {
                    rhs[getIdx(i, j)] -= scale * vy[getIdxY(i, j)];
                }
                if (spaceTypes[getIdx(i, j + 1)] == SpaceType::Solid) {
                    rhs[getIdx(i, j)] += scale * vy[getIdxY(i, j + 1)];
                }
            }
        }
    }
}

void Solver::fillPressureMatrix() {
    double scale = dt / (density * dx * dx);
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                if (i != 0) {
                    if (spaceTypes[getIdx(i - 1, j)] != SpaceType::Solid) {
                        press_diag[getIdx(i, j)] += scale;
                    }
                }
                if (i != size - 1) {
                    if (spaceTypes[getIdx(i + 1, j)] == SpaceType::Fluid) {
                        press_diag[getIdx(i, j)] += scale;
                        pressX[getIdx(i, j)] = -scale;
                    } else if (spaceTypes[getIdx(i + 1, j)] == SpaceType::Empty) {
                        press_diag[getIdx(i, j)] += scale;
                    }
                }
                if (j != size - 1) {
                    if (spaceTypes[getIdx(i, j - 1)] != SpaceType::Solid) {
                        press_diag[getIdx(i, j)] += scale;
                    }
                }
                if (j != 0) {
                    if (spaceTypes[getIdx(i, j + 1)] == SpaceType::Fluid) {
                        press_diag[getIdx(i, j)] += scale;
                        pressY[getIdx(i, j)] = -scale;
                    } else if (spaceTypes[getIdx(i, j + 1)] == SpaceType::Empty) {
                        press_diag[getIdx(i, j)] += scale;
                    }
                }
            }
        }
    }
}


//MIC Preconditioner matrix
void Solver::calcPreconditioner() {
    double safe = 0.25;
    double tau = 0.97;
    for (int j = 1; j < size - 1; ++j) {
        for (int i = 1; i < size - 1; ++i) {
            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                double e = press_diag[getIdx(i, j)]
                           - std::pow(pressX[getIdx(i - 1, j)] * preconditioner[getIdx(i - 1, j)], 2)
                           - std::pow(pressY[getIdx(i, j - 1)] * preconditioner[getIdx(i, j - 1)], 2)
                           - tau * (pressX[getIdx(i - 1, j)] * pressY[getIdx(i - 1, j)] *
                                    std::pow(preconditioner[getIdx(i - 1, j)], 2)
                                    + pressY[getIdx(i, j - 1)] * pressX[getIdx(i, j - 1)] *
                                      std::pow(preconditioner[getIdx(i, j - 1)], 2));
                if (e < safe * press_diag[getIdx(i, j)]) {
                    e = press_diag[getIdx(i, j)];
                }
                preconditioner[getIdx(i, j)] = 1 / std::sqrt(e);
            }
        }
    }
}

//Preconditioned conjugate gradient algorithm
void Solver::PCG() {
    for (int i = 0; i < size * size; ++i) {
        pressure[i] = 0;
    }

    bool isEnd = true;
    for (int i = 0; i < size * size; ++i) {
        if (std::abs(rhs[i]) > TOL) {
            isEnd = false;
            break;
        }
    }

    if (isEnd) {
        return;
    }

    pressureResidual = rhs;

    calcPreconditioner();

    applyPreconditioner();

    s = z;

    double sygma = dotProduct(z.data(), pressureResidual.data());
    for (int i = 0; i < PCG_MAX_ITERS; ++i) {
        applyPressureMatrix();

        double alpha = sygma / dotProduct(z.data(), s.data());

        int endFlag = 1;

        for (int j = 0; j < size * size; ++j) {
            pressure[j] += alpha * s[j];
            pressureResidual[j] -= alpha * z[j];
            if (std::abs(pressureResidual[j]) > TOL) {
                endFlag = 0;
            }
        }
//        printMaxMin(pressureResidual);

        //Вектор давлений найден
        if (endFlag) {
            return;
        }

        applyPreconditioner();

        double sygma_new = dotProduct(z.data(), pressureResidual.data());

        double beta = sygma_new / sygma;

        for (int j = 0; j < size * size; ++j) {
            s[j] = z[j] + beta * s[j];
        }

        sygma = sygma_new;
    }
}

void Solver::applyPreconditioner() {

    for (int j = 1; j < size - 1; ++j) {
        for (int i = 1; i < size - 1; ++i) {
            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                double t = pressureResidual[getIdx(i, j)]
                           - pressX[getIdx(i - 1, j)]
                             * preconditioner[getIdx(i - 1, j)] * q[getIdx(i - 1, j)]
                           - pressY[getIdx(i, j - 1)]
                             * preconditioner[getIdx(i, j - 1)] * q[getIdx(i, j - 1)];

                q[getIdx(i, j)] = t * preconditioner[getIdx(i, j)];
            }
        }
    }
    for (int j = size - 2; j >= 1; --j) {
        for (int i = size - 2; i >= 1; --i) {
            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                double t = q[getIdx(i, j)]
                           - pressX[getIdx(i, j)]
                             * preconditioner[getIdx(i, j)] * z[getIdx(i + 1, j)]
                           - pressY[getIdx(i, j)]
                             * preconditioner[getIdx(i, j)] * z[getIdx(i, j + 1)];

                z[getIdx(i, j)] = t * preconditioner[getIdx(i, j)];
            }
        }
    }
}


void Solver::applyPressureMatrix() {
    for (int j = 1; j < size - 1; ++j) {
        for (int i = 1; i < size - 1; ++i) {
            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                double value = 0.0;
                value += s[getIdx(i - 1, j)] * pressX[getIdx(i - 1, j)];
                value += s[getIdx(i + 1, j)] * pressX[getIdx(i, j)];
                value += s[getIdx(i, j - 1)] * pressY[getIdx(i, j - 1)];
                value += s[getIdx(i, j + 1)] * pressY[getIdx(i, j)];
                value += s[getIdx(i, j)] * press_diag[getIdx(i, j)];

                z[getIdx(i, j)] = value;
            }
        }
    }
}

double Solver::dotProduct(double *first, double *second) {
    double sum = 0.0;
    for (int i = 0; i < size * size; ++i) {
        sum += first[i] * second[i];
    }
    return sum;
}

bool Solver::isFluidVelocityX(int i, int j) {
    return spaceTypes[getIdx(i - 1, j)] == SpaceType::Fluid ||
           spaceTypes[getIdx(i, j)] == SpaceType::Fluid && !(spaceTypes[getIdx(i - 1, j)] == SpaceType::Solid ||
                                                             spaceTypes[getIdx(i, j)] == SpaceType::Solid);
}

bool Solver::isFluidVelocityY(int i, int j) {
    return spaceTypes[getIdx(i, j - 1)] == SpaceType::Fluid ||
           spaceTypes[getIdx(i, j)] == SpaceType::Fluid && !(spaceTypes[getIdx(i, j - 1)] == SpaceType::Solid ||
                                                             spaceTypes[getIdx(i, j)] == SpaceType::Solid);
}

//Обновляем скорости с помощью вычисленного вектора давлений
void Solver::updateVelocities() {
    vector<int> counts = {};
    counts.resize(size * size, 0);
    for (int i = 0; i < particles_size; ++i) {
        int x = cutValue(1, size - 2, particles[i].pos_x / dx);
        int y = cutValue(1, size - 2, particles[i].pos_y / dx);
        counts[getIdx(x, y)]++;
    }
    for (int j = 1; j < size - 1; ++j) {
        for (int i = 1; i < size - 1; ++i) {
            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                int c = counts[getIdx(i, j)];
                if (c > 8) {
                    pressure[getIdx(i, j)] += particles_pressure_coef * (c - 8);
                }
            }
        }
    }

    double scale = dt / (density * dx);
    for (int j = 1; j < size - 1; ++j) {
        for (int i = 1; i < size - 1; ++i) {
            if (spaceTypes[getIdx(i - 1, j)] == SpaceType::Fluid ||
                spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                if (spaceTypes[getIdx(i - 1, j)] == SpaceType::Solid ||
                    spaceTypes[getIdx(i, j)] == SpaceType::Solid) {
                    vx[getIdxX(i, j)] = 0;
                } else {
                    vx[getIdxX(i, j)] -= scale * (pressure[getIdx(i, j)] - pressure[getIdx(i - 1, j)]);
                }
            }
            if (spaceTypes[getIdx(i, j - 1)] == SpaceType::Fluid ||
                spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                if (spaceTypes[getIdx(i, j - 1)] == SpaceType::Solid ||
                    spaceTypes[getIdx(i, j)] == SpaceType::Solid) {
                    vy[getIdxY(i, j)] = 0;

                } else {
                    vy[getIdxY(i, j)] -= scale * (pressure[getIdx(i, j)] - pressure[getIdx(i, j - 1)]);
                }
            }

        }
    }
}

void Solver::fillWithZeros(double *v, int size) {
    for (int i = 0; i < size; ++i) {
        v[i] = 0;
    }
}

int Solver::getIdx(int i, int j) {
//    return i + size * (size - j - 1);
    return i + size * j;
}

int Solver::getIdxY(int i, int j) {
//    return i + size * (size - j);
    return i + size * j;
}

int Solver::getIdxX(int i, int j) {
//    return i + (size + 1) * (size - j - 1);
    return i + (size + 1) * j;
}

void Solver::resetParams() {
    int s = size * size;
    fillWithZeros(rhs.data(), s);
    fillWithZeros(pressureResidual.data(), s);
    fillWithZeros(press_diag.data(), s);
    fillWithZeros(pressX.data(), s);
    fillWithZeros(pressY.data(), s);
    fillWithZeros(z.data(), s);
    fillWithZeros(this->s.data(), s);
    fillWithZeros(diff_vy.data(), s);
    fillWithZeros(diff_vx.data(), s);
}

void Solver::checkDivergence() {
    double max_div = 0;
    int max_i = -1;
    int max_j = -1;
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            double div =
                    (vx[getIdxX(i + 1, j)] - vx[getIdxX(i, j)]) / dx +
                    (vy[getIdxY(i, j + 1)] - vy[getIdxY(i, j)]) / dx;
            if (std::abs(div) > std::abs(max_div) && spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                max_div = div;
                max_i = i;
                max_j = j;
            }
        }
    }
    std::cout << "max_div: " << max_div << ", max_i: " << max_i << ", max_j: " << max_j << std::endl;
}

double roundTo2(double x) {
    return roundf(x * 100) / 100;
}

void Solver::visualise() {
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            char c = 'F';
            if (spaceTypes[getIdx(i, j)] == SpaceType::Solid) {
                c = 'S';
            } else if (spaceTypes[getIdx(i, j)] == SpaceType::Empty) {
                c = 'E';
            }
            std::cout << "vx: " << roundTo2(vx[getIdxX(i, j)]) << ",vy: " << roundTo2(vy[getIdxY(i, j)])
                      << ",pressure: " << roundTo2(pressure[getIdx(i, j)]) << "-" << c << "      ";
            if (i == size - 1) {
                std::cout << "\n\n\n" << std::endl;
            }
        }
    }
}


void Solver::dirichleCondition() {

    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            if (spaceTypes[getIdx(i, j)] != SpaceType::Fluid) {
                pressure[getIdx(i, j)] = 0;
            }
        }
    }

    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size + 1; ++i) {
            if (i > 0) {
                if (spaceTypes[getIdx(i - 1, j)] == SpaceType::Solid && vx[getIdxX(i, j)] < 0) {
                    vx[getIdxX(i, j)] = 0;
                }
            }
            if (i < size) {
                if (spaceTypes[getIdx(i, j)] == SpaceType::Solid && vx[getIdxX(i, j)] > 0) {
                    vx[getIdxX(i, j)] = 0;
                }
            }
        }
    }

    for (int j = 0; j < size + 1; ++j) {
        for (int i = 0; i < size; ++i) {
            if (j > 0) {
                if (spaceTypes[getIdx(i, j - 1)] == SpaceType::Solid && vy[getIdxY(i, j)] < 0) {
                    vy[getIdxY(i, j)] = 0;
                }
            }
            if (j < size) {
                if (spaceTypes[getIdx(i, j)] == SpaceType::Solid && vy[getIdxY(i, j)] > 0) {
                    vy[getIdxY(i, j)] = 0;
                }
            }
        }
    }
}

void Solver::assembleGridFromParticles() {
    createSpaceTypes();
    getVelocitiesFromParticles();
}

void Solver::clearGrid() {
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            spaceTypes[getIdx(i, j)] = SpaceType::Empty;
        }
    }
}

void Solver::createSpaceTypes() {
    clearGrid();
    for (int i = 0; i < particles_size; ++i) {
        int x = roundValue(0, size - 1, particles[i].pos_x / dx);
        int y = roundValue(0, size - 1, particles[i].pos_y / dx);
        spaceTypes[getIdx(x, y)] = SpaceType::Fluid;
    }

    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            if (i == 0 || i == size - 1 || j == 0 || j == size - 1) {
                spaceTypes[getIdx(i, j)] = SpaceType::Solid;
            }
        }
    }

    for (auto &i: solid_indices) {
        spaceTypes[i] = SpaceType::Solid;
    }
}

double Solver::h2(double x) {

    if (x >= -1.5 && x < -0.5) {
        return 0.5 * pow(x + 1.5, 2);
    }
    if (x >= -0.5 && x < 0.5) {
        return 0.75 - x * x;
    }
    if (x >= 0.5 && x < 1.5) {
        return 0.5 * pow(1.5 - x, 2);
    }
    return 0;
};

//kernel function for interpolating particle values
double Solver::kernelFunc(double x, double y) {
    return h2(x / dx) * h2(y / dx);
}

void Solver::getVelocitiesFromParticles() {
    fillWithZeros(vx.data(), size * (size + 1));
    fillWithZeros(vy.data(), size * (size + 1));

    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            GridPICInfo &info = gridInfo[getIdx(i, j)];
            info.sum_vx = 0;
            info.sum_vy = 0;
            info.weight_vx = 0;
            info.weight_vy = 0;
        }
    }

    for (int i = 0; i < particles_size; ++i) {
        Particle &particle = particles[i];

        int x = roundValue(1, size - 2, particle.pos_x / dx);
        int y = roundValue(1, size - 2, particle.pos_y / dx);
        GridPICInfo &info = gridInfo[getIdx(x, y)];
        double vx_k = kernelFunc(particle.pos_x - (x - 0.5) * dx, particle.pos_y - y * dx);
        double vy_k = kernelFunc(particle.pos_x - x * dx, particle.pos_y - (y - 0.5) * dx);
        info.sum_vx += particle.vx * vx_k;
        info.sum_vy += particle.vy * vy_k;
        info.weight_vx += vx_k;
        info.weight_vy += vy_k;
    }

    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            GridPICInfo &info = gridInfo[getIdx(i, j)];
            if (info.sum_vx > 0.0001) {
                vx[getIdxX(i, j)] = info.sum_vx / info.weight_vx;
            }
            if (info.sum_vy > 0.0001) {
                vy[getIdxY(i, j)] = info.sum_vy / info.weight_vy;
            }

        }
    }
}

void Solver::countDiffXY() {
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            diff_vx[getIdx(i, j)] = getVelocityX(vx.data(), i, j) - getVelocityX(prev_vx.data(), i, j);
            diff_vy[getIdx(i, j)] = getVelocityY(vy.data(), i, j) - getVelocityY(prev_vy.data(), i, j);
        }
    }
}


void Solver::advectParticles() {
    countDiffXY();
    vector<double> _vx;
    vector<double> _vy;
    _vx.resize(size * size);
    _vy.resize(size * size);
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            _vx[getIdx(i, j)] = getVelocityX(vx.data(), i, j);
            _vy[getIdx(i, j)] = getVelocityY(vy.data(), i, j);
        }
    }
    double alpha = getAlpha();
    for (int i = 0; i < particles_size; ++i) {
        Particle &particle = particles[i];
        int x = roundValue(0, size - 1, particle.pos_x / dx);
        int y = roundValue(0, size - 1, particle.pos_y / dx);

        double vx_interpolated = interpolate(diff_vx[getIdx(x, y)], diff_vx.data(), particle.pos_x / dx,
                                             particle.pos_y / dx, x, y);
        double vy_interpolated = interpolate(diff_vy[getIdx(x, y)], diff_vy.data(), particle.pos_x / dx,
                                             particle.pos_y / dx, x,
                                             y);
        particle.vx = alpha * getVelocityX(vx.data(), x, y) + (1 - alpha) * (particle.vx + vx_interpolated);
        particle.vy = alpha * getVelocityY(vy.data(), x, y) + (1 - alpha) * (particle.vy + vy_interpolated);

        //По оси X
        particle.pos_x += particle.vx * dt;

        x = roundValue(0, size - 1, particle.pos_x / dx);

        //Границы
        while (x == 0 || particle.pos_x < 0 || x == size - 1 || particle.pos_x > dx * size) {
            particle.pos_x -= particle.vx * dt;
            x = roundValue(0, size - 1, particle.pos_x / dx);
        }

        //Другие твердые клетки
        while(spaceTypes[getIdx(x, y)] == SpaceType::Solid) {
            particle.pos_x -= particle.vx * dt;
            x = roundValue(0, size - 1, particle.pos_x / dx);
        }

        //По оси Y
        particle.pos_y += particle.vy * dt;
        y = roundValue(0, size - 1, particle.pos_y / dx);

        //Границы
        while (y == 0 || particle.pos_y < 0 || y == size - 1 || particle.pos_y > dx * size) {
            particle.pos_y -= particle.vy * dt;
            y = roundValue(0, size - 1, particle.pos_y / dx);
        }
//
        //Другие твердые клетки
        while(spaceTypes[getIdx(x, y)] == SpaceType::Solid) {
            particle.pos_y -= particle.vy * dt;
            y = roundValue(0, size - 1, particle.pos_y / dx);
        }
//
//
        while (spaceTypes[getIdx(x, y - 1)] == SpaceType::Solid && particle.vy <= 1 &&
               spaceTypes[getIdx(x, y + 1)] == SpaceType::Empty) {
            particle.pos_y += dx;
            particle.vy += g * dt;
            y = roundValue(0, size - 1, particle.pos_y / dx);
        }
//

    }
}

double Solver::getAlpha() {
    double res = 6 * dt * viscosity / (dx * dx);
    if (res < 0) {
        res = 0;
    }
    if (res > 1) {
        res = 1;
    }
    std::cout << res << std::endl;
    return res;
}


