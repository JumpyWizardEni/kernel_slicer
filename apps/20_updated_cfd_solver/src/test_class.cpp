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

int Solver::cutValue(float from, float to, float value) {
    if (value > to) {
        value = to;
    }
    if (value < from) {
        value = from;
    }
    return (int) std::round(value);
}

float Solver::getVelocityX(float *vx, int i, int j) {
    float res = (vx[getIdxX(i, j)] + vx[getIdxX(i + 1, j)]) / 2;

    return res;
}

float Solver::getVelocityY(float *vy, int i, int j) {
    float res = (vy[getIdxY(i, j)] + vy[getIdxY(i, j + 1)]) / 2;

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
    dirichleCondition();
    prev_vx = vx;
    prev_vy = vy;
    addForces(vy.data(), g);
//    extrapolateVelocities();
//    float r = randfrom1(0, 1);
//    if (r > 0.9) {
//        addForces(vx.data(), g * 100);
//    }
    project();
    checkDivergence();
    dirichleCondition();
    advectParticles();
    changeParticlesNum();
    checkDivergence();
//    visualise();
}


//Считаем dt на текущем шаге, dt <= 5 * dx / max(velocity)
void Solver::countTimeDelta(const float *p_vx, const float *p_vy) {
    //Ищем максимум
    float maximum = -1;
    for (int i = 0; i < size * (size + 1); ++i) {
        float vel = p_vx[i] * p_vx[i] + p_vy[i] * p_vy[i];
        if (vel > maximum) {
            maximum = vel;
        }
    }
    maximum = std::sqrt(maximum);
    maximum += std::sqrt(5 * dx * std::abs(g)); // чтобы не было деления на 0
    dt = 5 * dx / maximum;
}

//Добавляем силу тяжести
void Solver::addForces(float *v, float a) {
    for (int j = 1; j < size; ++j) {
        for (int i = 1; i < size - 1; ++i) {
            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                v[getIdxY(i, j)] += dt * a;
            }
        }
    }
}

float Solver::interpolate(float q, float *q_copy, float x, float y, int i, int j) {
    float w_x = (ceil(x) - x) * dx;
    float w_y = (ceil(y) - y) * dx;

    //Кубическая интерполяция

    //По x
    float w_x_2 = w_x * w_x;
    float w_x_3 = w_x * w_x_2;
    float w0 = -w_x / 3 + w_x_2 / 3 - w_x_3 / 6;
    float w1 = 1 - w_x_2 + (w_x_3 - w_x) / 2;
    float w2 = w_x + (w_x_2 - w_x_3) / 2;
    float w3 = (w_x_3 - w_x) / 6;

    float q_j_1 = w0 * q_copy[getIdx(i - 1, j - 1)] + w1 * q_copy[getIdx(i, j - 1)] +
                  w2 * q_copy[getIdx(i + 1, j - 1)] + w3 * q_copy[getIdx(i + 2, j - 1)];

    float q_j = w0 * q_copy[getIdx(i - 1, j)] + w1 * q_copy[getIdx(i, j)] +
                w2 * q_copy[getIdx(i + 1, j)] + w3 * q_copy[getIdx(i + 2, j)];

    float q_j1 = w0 * q_copy[getIdx(i - 1, j + 1)] + w1 * q_copy[getIdx(i, j + 1)] +
                 w2 * q_copy[getIdx(i + 1, j + 1)] + w3 * q_copy[getIdx(i + 2, j + 1)];

    float q_j2 = w0 * q_copy[getIdx(i - 1, j + 2)] + w1 * q_copy[getIdx(i, j + 2)] +
                 w2 * q_copy[getIdx(i + 1, j + 2)] + w3 * q_copy[getIdx(i + 2, j + 2)];

    //По y
    float w_y_2 = w_y * w_y;
    float w_y_3 = w_y * w_y_2;
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

//    dirichleCondition();
    //Compute new velocities
    updateVelocities();

};

void Solver::calcNegativeDivergence() {

    float scale = 1 / dx;
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
    float scale = dt / (density * dx * dx);
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
    float safe = 0.25;
    double tau = 0.97;
    for (int j = 1; j < size - 1; ++j) {
        for (int i = 1; i < size - 1; ++i) {
            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                float e = press_diag[getIdx(i, j)]
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

    float sygma = dotProduct(z.data(), pressureResidual.data());
    for (int i = 0; i < PCG_MAX_ITERS; ++i) {
        applyPressureMatrix();

        float alpha = sygma / dotProduct(z.data(), s.data());

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

        float sygma_new = dotProduct(z.data(), pressureResidual.data());

        float beta = sygma_new / sygma;

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
                float t = pressureResidual[getIdx(i, j)]
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
                float t = q[getIdx(i, j)]
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
                float value = 0.0;
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

float Solver::dotProduct(float *first, float *second) {
    float sum = 0.0;
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
    float scale = dt / (density * dx);
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

void Solver::fillWithZeros(float *v, int size) {
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

void Solver::checkNonFluidVelocities() {
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            if (spaceTypes[getIdx(i, j)] != SpaceType::Fluid) {
                std::cout << "vx: " << getVelocityX(vx.data(), i, j) << "vy: " << getVelocityY(vy.data(), i, j)
                          << std::endl;
            }
        }
    }
}

float roundTo2(float x) {
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
            if (!isFluidVelocityX(i, j)) {
                vx[getIdxX(i, j)] = 0;
            }
        }
    }

    for (int j = 0; j < size + 1; ++j) {
        for (int i = 0; i < size; ++i) {
            if (!isFluidVelocityY(i, j)) {
                vy[getIdxY(i, j)] = 0;
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
        int x = cutValue(0, size - 1, particles[i].pos_x / dx);
        int y = cutValue(0, size - 1, particles[i].pos_y / dx);
        spaceTypes[getIdx(x, y)] = SpaceType::Fluid;
    }

    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            if (i == 0 || i == size - 1 || j == 0 || j == size - 1) {
                spaceTypes[getIdx(i, j)] = SpaceType::Solid;
            }
        }
    }
}

float Solver::h2(float x) {

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
float Solver::kernelFunc(float x, float y) {
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

        int x = cutValue(1, size - 2, particle.pos_x / dx);
        int y = cutValue(1, size - 2, particle.pos_y / dx);
        GridPICInfo &info = gridInfo[getIdx(x, y)];
        float vx_k = kernelFunc(particle.pos_x - (x - 0.5) * dx, particle.pos_y - y * dx);
        float vy_k = kernelFunc(particle.pos_x - x * dx, particle.pos_y - (y - 0.5) * dx);
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
    for (int i = 0; i < particles_size; ++i) {
        Particle &particle = particles[i];
        int x = cutValue(1, size - 2, particle.pos_x / dx);
        int y = cutValue(1, size - 2, particle.pos_y / dx);
        float vx_interpolated = interpolate(diff_vx[getIdx(x, y)], diff_vx.data(), particle.pos_x / dx,
                                            particle.pos_y / dx, x, y);
        particle.vx += vx_interpolated;
        particle.vy += interpolate(diff_vy[getIdx(x, y)], diff_vy.data(), particle.pos_x / dx, particle.pos_y / dx, x,
                                   y);

        particle.pos_x += particle.vx * dt;
        particle.pos_y += particle.vy * dt;
//        float substepTaw = 0;
//        bool finished = false;
//        float _vx = getVelocityX(vx.data(), x, y);
//        float _vy = getVelocityY(vy.data(), x, y);
//        float newX = x;
//        float newY = y;
//        while(!finished) {
//            float dTaw = dx / (sqrt(_vx*_vx + _vy*_vy) + 1e-20);
//            if (substepTaw + dTaw > dt) {
//                dTaw = dt - substepTaw;
//                finished = true;
//            } else if (substepTaw + 2 * dTaw >= dt) {
//                dTaw = 0.5 * (dt - substepTaw);
//            }
//            float k1_x = getVelocityX(vx.data(), round(newX), round(newY));
//            float k1_y = getVelocityY(vy.data(), round(newX), round(newY));
//
//            float k2_x = getVelocityX(vx.data(), cutValue(1, size - 2, newX + 0.5 * substepTaw * k1_x),
//                                      cutValue(1, size - 2, newY + 0.5 * substepTaw * k1_y));
//            float k2_y = getVelocityY(vy.data(), cutValue(1, size - 2, newX + 0.5 * substepTaw * k1_x),
//                                      cutValue(1, size - 2, newY + 0.5 * substepTaw * k1_y));
//
//            float k3_x = getVelocityX(vx.data(), cutValue(1, size - 2, newX + 0.75 * substepTaw * k2_x),
//                                      cutValue(1, size - 2, newY + 0.75 * substepTaw * k2_y));
//
//            float k3_y = getVelocityY(vy.data(), cutValue(1, size - 2, newX + 0.75 * substepTaw * k2_x),
//                                      cutValue(1, size - 2, newY + 0.75 * substepTaw * k2_y));
//            _vx = (2.0 / 9 * k1_x + 0.3333 * k2_x + 4.0/9 * k3_x);
//            _vy = (2.0 / 9 * k1_y + 0.3333 * k2_y + 4.0/9 * k3_y);
//            newX += _vx * substepTaw;
//            newY +=  _vy * substepTaw;
//            substepTaw = substepTaw + dTaw;
//        }

//
//        particle.vx = _vx;
//        particle.vy = _vy;
//        particle.pos_x = newX;
//        particle.pos_y = newY;


        float velocityCoef = 1;
        float dxCoef = 1;
        while (particle.pos_x < -2) {
            particle.pos_x += 1;
        }
        while (particle.pos_x > 2) {
            particle.pos_x -= 1;
        }
        while (particle.pos_y < -2) {
            particle.pos_y += 1;
        }
        while (particle.pos_y > 2) {
            particle.pos_y -= 1;
        }
        if (particle.vx > 0) {
            if (particle.pos_x > 1) {
                particle.pos_x = 2 - particle.pos_x - 2 * dxCoef * dx;
                particle.vx = -particle.vx * velocityCoef;
            } else if (particle.pos_x > 1 - dx) {
                particle.pos_x = particle.pos_x - dx;
                particle.vx = -particle.vx * velocityCoef;
            } else if (particle.pos_x < dxCoef * dx) {
                particle.pos_x = dxCoef * dx + particle.pos_x;
                particle.vx = -particle.vx * velocityCoef;
            }
        } else {
            if (particle.pos_x < 0) {
                particle.pos_x = -particle.pos_x + dxCoef * dx;
                particle.vx = -particle.vx * velocityCoef;
            }
        }
        if (particle.vy > 0) {
            if (particle.pos_y > 1) {
                particle.pos_y = 2 - particle.pos_y - 2 * dxCoef * dx;
                particle.vy = -particle.vy * velocityCoef;
            } else if (particle.pos_y > 1 - dx) {
                particle.pos_y = particle.pos_y - dx;
                particle.vy = -particle.vy * velocityCoef;
            } else if (particle.pos_y < dxCoef * dx) {
                particle.pos_y = dxCoef * dx + particle.pos_y;
                particle.vy = -particle.vy * velocityCoef;
            }
        } else {
            if (particle.pos_y < dxCoef * dx) {
                particle.pos_y = dxCoef * dx - particle.pos_y;
                particle.vy = -particle.vy * velocityCoef;
            }
        }
    }
}

//Удаляем, если в ячейке > 8 частиц. Добавляем, если в ячейке <4 частиц
void Solver::changeParticlesNum() {
    std::map<int, std::vector<int>> counts = std::map<int, std::vector<int>>();
    for (int i = 0; i < particles_size; ++i) {
        int x = cutValue(1, size - 2, particles[i].pos_x * size);
        int y = cutValue(1, size - 2, particles[i].pos_y * size);
        int ind = getIdx(x, y);
        if (counts.count(ind)) {
            counts[ind].push_back(i);
        } else {
            counts[ind] = {i};
        }
    }
    for (auto &count: counts) {
        int c = count.second.size();
        if (c > 12) {
            for (int i = c - 1; i >= 12; --i) {
                particles.erase(particles.begin() + count.second[i]);
                particles_size--;
            }
        }

        if (c < 4) {
            for (int i = c; i < 4; ++i) {
                particles.push_back(getMeanParticle(count.second));
                particles_size++;
            }
        }
    }
}

Particle Solver::getMeanParticle(vector<int> &particlesIndices) {
    float pos_x = 0;
    float pos_y = 0;
    float vx = 0;
    float vy = 0;
    float s = particles.size();
    for (auto i: particlesIndices) {
        Particle &p = particles[i];
        pos_x += p.pos_x;
        pos_y += p.pos_y;
        vx += p.vx;
        vy += p.vy;
    }
    pos_x /= s;
    pos_y /= s;
    vx /= s;
    vy /= s;
    return Particle(vx, vy, pos_x, pos_y);
}

void Solver::extrapolateVelocities() {
    // Маркируем известные значения
    wavefront.clear();
    fillWithZeros(reinterpret_cast<float *>(mask.data()), size * size);
    for (int j = 1; j < size; ++j) {
        for (int i = 1; i < size; ++i) {
            if (spaceTypes[getIdx(i - 1, j)] != SpaceType::Fluid && spaceTypes[getIdx(i, j)] != SpaceType::Fluid) {
                mask[i] = INT32_MAX;
            }
        }
    }

    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            if (mask[getIdx(i, j)] != 0 &&
                (mask[getIdx(i - 1, j)] == 0 || mask[getIdx(i + 1, j)] == 0)) {
                mask[getIdx(i, j)] = 1;
                wavefront.push_back(getIdx(i, j));
            }
        }
    }
    int t = 0;
    while (t < wavefront.size()) {
        int ind = wavefront[t];
        int i = ind % size;
        int j = ind / size;
        float vx_sum = 0;
        int c = 0;
        for (int i1 = i - 1; i1 <= i + 1; i1 += 2) {
            if (i1 >= 0) {
                if (mask[getIdx(i1, j)] < mask[getIdx(i, j)]) {
                    c++;
                    vx_sum += vx[getIdxX(i1, j)];
                } else if (mask[getIdx(i1, j)] == INT32_MAX) {
                    mask[getIdx(i1, j)] = mask[getIdx(i, j)] + 1;
                    wavefront.push_back(getIdx(i1, j));
                }
            }
        }
        if (c > 0) {
            vx_sum /= c;
        }
        vx[getIdxX(i, j)] = vx_sum;
        t = t + 1;
    }

    wavefront.clear();
    fillWithZeros(reinterpret_cast<float *>(mask.data()), size * size);
    for (int j = 1; j < size; ++j) {
        for (int i = 1; i < size; ++i) {
            if (spaceTypes[getIdx(i, j - 1)] != SpaceType::Fluid && spaceTypes[getIdx(i, j)] != SpaceType::Fluid) {
                mask[i] = INT32_MAX;
            }
        }
    }

    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            if (mask[getIdx(i, j)] != 0 &&
                (mask[getIdx(i, j - 1)] == 0 || mask[getIdx(i, j + 1)] == 0)) {
                mask[getIdx(i, j)] = 1;
                wavefront.push_back(getIdx(i, j));
            }
        }
    }
    t = 0;
    while (t < wavefront.size()) {
        int ind = wavefront[t];
        int i = ind % size;
        int j = ind / size;
        float vy_sum = 0;
        int c = 0;
        for (int j1 = j - 1; j1 <= j + 1; j1 += 2) {
            if (j1 >= 0) {
                if (mask[getIdx(i, j1)] < mask[getIdx(i, j)]) {
                    c++;
                    vy_sum += vy[getIdxY(i, j1)];
                } else if (mask[getIdx(i, j1)] == INT32_MAX) {
                    mask[getIdx(i, j1)] = mask[getIdx(i, j)] + 1;
                    wavefront.push_back(getIdx(i, j1));
                }
            }
        }
        if (c > 0) {
            vy_sum /= c;
        }
        vy[getIdxY(i, j)] = vy_sum;
        t = t + 1;
    }

}

