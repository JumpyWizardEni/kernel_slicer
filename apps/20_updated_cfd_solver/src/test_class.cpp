#include "test_class.h"
#include <cmath>
#include <vector>
#include <cstring>

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

    velocityExtra.resize(size * size, 0);
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
    countTimeDelta(vx.data(), vy.data());
    addForces(vy.data());
    velocityExtra = vx;
    velocityExtra2 = vx;
    advect(vx.data(), vy.data(), velocityExtra.data(), velocityExtra2.data());
    vx = velocityExtra2;
    velocityExtra = vy;
    velocityExtra2 = vy;
    advect(vx.data(), vy.data(), velocityExtra.data(), velocityExtra2.data());
    vy = velocityExtra2;
    spaceTypesOld = spaceTypes;
    project();
    moveCells(spaceTypesOld.data(), spaceTypes.data());
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
    for (int i = 0; i < size * (size + 1); ++i) {
        v[i] += dt * g;
    }
}

void Solver::advect(float *vx, float *vy, float *q_copy, float *q) {
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            //Semi-Lagrangian метод: ищем точку, в которой текущая величина была dt времени назад
            //Метод Рунге-Кутта 2 порядка
            int mid_x = std::round(i - 0.5 * dt * vx[getIdxX(i, j)] / dx);
            int mid_y = std::round(j - 0.5 * dt * vy[getIdxY(i, j)] / dx);

            float past_x = i - dt * vx[getIdxX(mid_x, mid_y)] / dx;
            float past_y = j - dt * vy[getIdxY(mid_x, mid_y)] / dx;
            //коэффициент интерполяции
            float w_x = (past_x - std::round(past_x)) / dx;
            float w_y = (past_y - std::round(past_y)) / dx;

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
            q[size * i + j] = w0 * q_j_1 + w1 * q_j + w2 * q_j1 + w3 * q_j2;
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
    for (int j = 1; j < size - 1; ++j) {
        for (int i = 1; i < size - 1; ++i) {
            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                float e = press_diag[getIdx(i, j)]
                          - std::pow(pressX[getIdx(i - 1, j)] * preconditioner[getIdx(i - 1, j)], 2)
                          - std::pow(pressY[getIdx(i, j - 1)] * preconditioner[getIdx(i, j - 1)], 2)
                          - pressX[getIdx(i - 1, j)] * pressY[getIdx(i - 1, j)] *
                            std::pow(preconditioner[getIdx(i - 1, j)], 2)
                          - pressY[getIdx(i, j - 1)] * pressX[getIdx(i, j - 1)] *
                            std::pow(preconditioner[getIdx(i, j - 1)], 2);
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
            float additionalCoef = alpha * s[j];
            pressure[j] += additionalCoef;
            pressureResidual[j] -= additionalCoef;
            if (pressureResidual[j] > TOL) {
                endFlag = 0;
            }
        }

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
    for (int j = size - 2; j >= 1; --j) {
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
    for (int j = 1; j <= size - 2; ++j) {
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
    float scale = dt / (density*dx*dx);
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            float value = 0.0;
            if (i != 0) {
                value += s[getIdx(i - 1, j)];
            }
            if (i != size - 1) {
                value += s[getIdx(i + 1, j)];
            }

            if (j != size - 1) {
                value += s[getIdx(i, j - 1)];
            }
            if (j != 0) {
                value += s[getIdx(i, j + 1)];
            }

            value *= -scale;

            value += press_diag[getIdx(i, j)] * s[getIdx(i, j)];

            z[getIdx(i, j)] = value;
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

//TODO по хорошему интерполировать и делить на 2-3-... частей
void Solver::moveCells(SpaceType *old_s, SpaceType *new_s) {
    for (int i = 0; i < size * size; ++i) {
        if (new_s[i] == SpaceType::Fluid) {
            new_s[i] = SpaceType::Empty;
        }
    }
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (old_s[getIdx(i, j)] == SpaceType::Fluid) {
                int new_i = cutValue(0.1, size - 1, i + dt * getVelocityX(i, j) / dx);
                int new_j = cutValue(0.1, size - 1, j + dt * getVelocityY(i, j) / dx);
                new_s[getIdx(new_i, new_j)] = SpaceType::Fluid;
            }
        }
    }
}

int Solver::getIdx(int i, int j) {
    return i + size * (size - j - 1);
}

int Solver::getIdxY(int i, int j) {
    return i + size * (size - j);
}

int Solver::getIdxX(int i, int j) {
    return i + (size + 1) * (size - j - 1);
}
