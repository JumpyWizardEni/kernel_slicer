#include "test_class.h"
#include <cmath>
#include <vector>
#include <cstring>
#include <algorithm>
#include <iostream>

using std::vector;

Solver::Solver() = default;

void Solver::setParameters() {
    pressure.resize(size * size, 0);
    pressureResidual.resize(size * size, 0);
    rhs.resize(size * size, 0);
    press_diag.resize(size * size, 0);
    pressX.resize(size * size, 0);
    pressY.resize(size * size, 0);
    preconditioner.resize(size * size, 0);
    q.resize(size * size, 0);
    z.resize(size * size, 0);
    s.resize(size * size, 0);

    velocityExtra.resize(size * (size + 1), 0);
    velocityExtra2.resize(size * (size + 1), 0);
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

float Solver::getVelocityX(int i, int j) {
    return (vx[getIdxX(i, j)] + vx[getIdxX(i + 1, j)]) / 2;
}

float Solver::getVelocityY(int i, int j) {
    return (vy[getIdxY(i, j)] + vx[getIdxY(i, j + 1)]) / 2;

}


void Solver::performStep() {
    resetParams();
//    visualise();
    countTimeDelta(vx.data(), vy.data());
    velocityExtra = vx;
    velocityExtra2 = vx;
    advect(vx.data(), vy.data(), velocityExtra.data(), velocityExtra2.data(), 'x');
    vx = velocityExtra2;
    velocityExtra = vy;
    velocityExtra2 = vy;
    advect(vx.data(), vy.data(), velocityExtra.data(), velocityExtra2.data(), 'y');
    dirichleCondition();

    vy = velocityExtra2;
    spaceTypesOld = spaceTypes;
    addForces(vy.data());
    project();
    checkDivergence();
//    visualise();
    moveCells(spaceTypesOld.data(), spaceTypes.data(), velocityExtra.data(), velocityExtra2.data());
    vx = velocityExtra;
    vy = velocityExtra2;
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
void Solver::addForces(float *v) {
    for (int j = 1; j < size - 1; ++j) {
        for (int i = 1; i < size; ++i) {
            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                v[getIdxY(i, j)] += dt * g;
            }
        }
    }
}

void Solver::advect(float *vx, float *vy, float *q_copy, float *q, char c) {
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            //Semi-Lagrangian метод: ищем точку, в которой текущая величина была dt времени назад
            double _vx = vx[getIdxX(i, j)];
            double _vy = vy[getIdxY(i, j)];
            int last_x = std::round(i - dt * _vx / dx);
            int last_y = std::round(j - dt * _vy / dx);
            if (c == 'x') {
                q[getIdxX(i, j)] = q_copy[getIdxX(last_x, last_y)];
            } else {
                q[getIdxY(i, j)] = q_copy[getIdxY(last_x, last_y)];
            }
        }
    }
}


//solve
void Solver::project() {
    //calc rhs for Ap = rhs
    calcNegativeDivergence();
    //set A
    fillPressureMatrix();
    //solve Ap = rhs
    PCG();

    dirichleCondition();
    //Compute new velocities
    updateVelocities();

};

void Solver::calcNegativeDivergence() {

    float scale = 1 / dx;
    for (int j = 1; j < size - 1; ++j) {
        for (int i = 1; i < size - 1; ++i) {
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
    float scale = dt / (density * dx * dx);
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

//TODO по хорошему интерполировать и делить на 2-3-... частей
void Solver::moveCells(SpaceType *old_s, SpaceType *new_s, float *new_vx, float *new_vy) {
    fillWithZeros(new_vx, size * (size + 1));
    fillWithZeros(new_vy, size * (size + 1));
    for (int i = 0; i < size * size; ++i) {
        if (new_s[i] == SpaceType::Fluid) {
            new_s[i] = SpaceType::Empty;
        }
    }
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            if (old_s[getIdx(i, j)] == SpaceType::Fluid) {
                double _vx = getVelocityX(i, j);
                double _vy = getVelocityY(i, j);
                int new_i = cutValue(1, size - 2, i + dt * _vx / dx);
                int new_j = cutValue(1, size - 2, j + dt * _vy / dx);
                new_s[getIdx(new_i, new_j)] = SpaceType::Fluid;
            }
        }
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

int Solver::getNormalIdx(int i, int j) {
    return i + size * j;
}

void Solver::printMaxMin(vector<float> &v) {
    double max = *std::max_element(std::begin(v), std::end(v));
    double min = *std::min_element(std::begin(v), std::end(v));
    std::cout << max << " " << min << std::endl;
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
                std::cout << "vx: " << getVelocityX(i, j) << "vy: " << getVelocityY(i, j) << std::endl;
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

void Solver::visualiseVx() {
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size + 1; ++i) {
            std::cout << vx[getIdxX(i, j)] << " ";
            if (i == size) {
                std::cout << std::endl;
            }
        }
    }
}

void Solver::visualiseSpaceTypes() {
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            char c = 'F';
            if (spaceTypes[getIdx(i, j)] == SpaceType::Solid) {
                c = 'S';
            } else if (spaceTypes[getIdx(i, j)] == SpaceType::Empty) {
                c = 'E';
            }
            std::cout << c << " ";
            if (i == size - 1) {
                std::cout << std::endl;
            }
        }
    }
}

void Solver::visualiseVy() {

}

void Solver::visualisePressure() {
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            char c = 'F';
            if (spaceTypes[getIdx(i, j)] == SpaceType::Solid) {
                c = 'S';
            } else if (spaceTypes[getIdx(i, j)] == SpaceType::Empty) {
                c = 'E';
            }
            std::cout << pressure[getIdx(i, j)] << "-" << c << " ";
            if (i == size - 1) {
                std::cout << std::endl;
            }
        }
    }
}
