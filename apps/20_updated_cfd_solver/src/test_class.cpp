#include "test_class.h"
#include <cmath>
#include <vector>
#include <cstring>

using std::vector;

Solver::Solver() = default;

void Solver::performStep() {
    countTimeDelta(vx.data(), vy.data());
    addForces(vy.data());
    velocityExtra = vx;
    advect(vx.data(), vy.data(), velocityExtra.data());
    vx = velocityExtra;
    velocityExtra = vy;
    advect(vx.data(), vy.data(), velocityExtra.data());
    vy = velocityExtra;
    spaceTypesOld = spaceTypes;
    calcNegativeDivergence();
//    memcpy(spaceTypesOld.data(), spaceTypes.data(), copy_size);
    moveCells(spaceTypesOld.data(), spaceTypes.data());
}

//solve
void Solver::project() {
    //calc rhs for Ap = rhs
    calcNegativeDivergence();
    //set A
    fillPressureMatrix();
    //preconditioner
    calcPreconditioner();
    //solve Ap = rhs
    PCG();
    //Compute new velocities
    updateVelocities();
};


//Считаем dt на текущем шаге, dt <= 5 * dx / max(velocity)
void Solver::countTimeDelta(const float *p_vx, const float *p_vy) {
    //Ищем максимум
    float maximum = -1;
    for (int i = 0; i < size * size; ++i) {
        float vel = std::sqrt(p_vx[i] * p_vx[i] + p_vy[i] * p_vy[i]);
        if (vel > maximum) {
            maximum = vel;
        }
    }
    maximum += std::sqrt(5 * dx * g); // чтобы не было деления на 0
    dt = 5 * dx / maximum;
}

//Обновляем скорости с помощью вычесленного вектора давлений
void Solver::updateVelocities() {
    float scale = dt / (density * dx);
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (spaceTypes[size * (i - 1) + j] == SpaceType::Fluid ||
                spaceTypes[size * i + j] == SpaceType::Fluid) {
                if (spaceTypes[size * (i - 1) + j] == SpaceType::Solid ||
                    spaceTypes[size * i + j] == SpaceType::Solid) {
                    vx[size * i + j] = 0;
                } else {
                    vx[size * i + j] -= scale * (pressure[size * i + j] - pressure[size * (i - 1) + j]);
                }
            }
            if (spaceTypes[size * i + j - 1] == SpaceType::Fluid ||
                spaceTypes[size * i + j] == SpaceType::Fluid) {
                if (spaceTypes[size * i + j] == SpaceType::Solid ||
                    spaceTypes[size * i + j] == SpaceType::Solid) {
                    vy[size * i + j] = 0;

                } else {
                    vy[size * i + j] -= scale * (pressure[size * i + j] - pressure[size * i + j - 1]);
                }
            }

        }
    }
}


void Solver::calcNegativeDivergence() {

    float scale = 1 / dx;
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (spaceTypes[size * i + j] == SpaceType::Fluid) {
                rhs[size * i + j] =
                        -scale * (vx[size * (i + 1) + j] - vx[size * i + j] + vy[size * i + j + 1] - vy[size * i + j]);
                if (spaceTypes[size * (i - 1) + j] == SpaceType::Solid) {
                    rhs[size * i + j] -= scale * vx[size * i + j];
                }
                if (spaceTypes[size * (i + 1) + j] == SpaceType::Solid) {
                    rhs[size * i + j] += scale * vx[size * (i + 1) + j];
                }
                if (spaceTypes[size * i + j - 1] == SpaceType::Solid) {
                    rhs[size * i + j] -= scale * vy[size * i + j];
                }
                if (spaceTypes[size * i + j + 1] == SpaceType::Solid) {
                    rhs[size * i + j] += scale * vy[size * i + j + 1];
                }
            }
        }
    }
}

void Solver::fillPressureMatrix() {
    float scale = dt / (density * dx * dx);
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (spaceTypes[size * i + j] == SpaceType::Fluid) {
                if (spaceTypes[size * (i - 1) + j] == SpaceType::Fluid) {
                    press_diag[size * i + j] += scale;
                }
                if (spaceTypes[size * (i + 1) + j] == SpaceType::Fluid) {
                    press_diag[size * i + j] += scale;
                    pressX[size * i + j] = -scale;
                } else if (spaceTypes[size * (i + 1) + j] == SpaceType::Empty) {
                    press_diag[size * i + j] += scale;
                }

                if (spaceTypes[size * i + j - 1] == SpaceType::Fluid) {
                    press_diag[size * i + j] += scale;
                }
                if (spaceTypes[size * i + j + 1] == SpaceType::Fluid) {
                    press_diag[size * i + j] += scale;
                    pressY[size * i + j] = -scale;
                } else if (spaceTypes[size * i + j + 1] == SpaceType::Empty) {
                    press_diag[size * i + j] += scale;
                }
            }
        }
    }
}

//Preconditioned conjugate gradient algorithm
void Solver::PCG() {
    for (int i = 0; i < size * size; ++i) {
        pressure[i] = 0;
    }
    for (int i = 0; i < size * size; ++i) {
        pressureResult[i] = rhs[i];
    }

    calcPreconditioner();

    applyPreconditioner();

    for (int i = 0; i < size * size; ++i) {
        s[i] = z[i];
    }

    dotProduct(z.data(), pressureResult.data(), sygma.data());
    for (int i = 0; i < PCG_MAX_ITERS; ++i) {
    }
}

void Solver::applyPreconditioner() {
    for (int i = 1; i < size - 1; ++i) {
        for (int j = 1; j < size - 1; ++j) {
            if (spaceTypes[size * i + j] == SpaceType::Fluid) {
                float t = pressureResult[size * i + j]
                          - pressX[size * (i - 1) + j]
                            * preconditioner[size * (i - 1) + j] * q[size * (i - 1) + j]
                          - pressY[size * i + j - 1]
                            * preconditioner[size * i + j - 1] * q[size * i + j - 1];

                q[size * i + j] = t * preconditioner[size * i + j];
            }
        }
    }
    for (int i = size - 2; i >= 1; --i) {
        for (int j = size - 2; j >= 1; --j) {
            if (spaceTypes[size * i + j] == SpaceType::Fluid) {
                float t = q[size * i + j]
                          - pressX[size * i + j]
                            * preconditioner[size * i + j] * z[size * (i + 1) + j]
                          - pressY[size * i + j]
                            * preconditioner[size * i + j] * z[size * i + j + 1];

                z[size * i + j] = t * preconditioner[size * i + j];
            }
        }
    }
}

//MIC Preconditioner matrix (size + 1) X (size + 1)
void Solver::calcPreconditioner() {
    float safe = 0.25;
    for (int i = 1; i < size - 1; ++i) {
        for (int j = 1; j < size - 1; ++j) {
            if (spaceTypes[size * i + j] == SpaceType::Fluid) {
                float e = press_diag[size * i + j]
                          - std::pow(pressX[size * (i - 1) + j] * preconditioner[size * (i - 1) + j], 2)
                          - std::pow(pressY[size * i + j - 1] * preconditioner[size * i + j - 1], 2)
                          - pressX[size * (i - 1) + j] * pressY[size * (i - 1) + j] *
                            std::pow(preconditioner[size * (i - 1) + j], 2)
                          - pressY[size * i + j - 1] * pressX[size * i + j - 1] *
                            std::pow(preconditioner[size * (i - 1) + j], 2);
                if (e < safe * press_diag[size * i + j]) {
                    e = press_diag[size * i + j];
                }
                preconditioner[size * i + j] = 1 / std::sqrt(e);
            }
        }
    }
}

//Добавляем силу тяжести
void Solver::addForces(float *v) {
    for (int i = 0; i < size * (size + 1); ++i) {
        v[i] += dt * g;
    }
}

void Solver::advect(float *vx, float *vy, float *q) {
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            //Semi-Lagrangian метод: ищем точку, в которой текущая величина была dt времени назад
            //Метод Рунге-Кутта 2 порядка
            int mid_x = std::round(i - 0.5 * dt * vx[size * i + j]);
            int mid_y = std::round(j - 0.5 * dt * vy[size * i + j]);

            float past_x = i - dt * vx[size * mid_x + mid_y];
            float past_y = j - dt * vy[size * mid_x + mid_y];
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

            float q_j_1 = w0 * q[size * (i - 1) + j - 1] + w1 * q[size * (i) + j - 1] +
                          w2 * q[size * (i + 1) + j - 1] + w3 * q[size * (i + 2) + j - 1];

            float q_j = w0 * q[size * (i - 1) + j] + w1 * q[size * (i) + j] +
                        w2 * q[size * (i + 1) + j] + w3 * q[size * (i + 2) + j];

            float q_j1 = w0 * q[size * (i - 1) + j + 1] + w1 * q[size * (i) + j + 1] +
                         w2 * q[size * (i + 1) + j + 1] + w3 * q[size * (i + 2) + j + 1];

            float q_j2 = w0 * q[size * (i - 1) + j + 2] + w1 * q[size * (i) + j + 2] +
                         w2 * q[size * (i + 1) + j + 2] + w3 * q[size * (i + 2) + j + 2];

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


int Solver::cutValue(float from, float to, float value) {
    if (value > to) {
        value = to;
    }
    if (value < from) {
        value = from;
    }
    return (int) std::round(value);
}

float Solver::getVelocityX(float i, float j) {
    return (vx[size * i + j] + vx[size * (i + 1) + j]) / 2;
}

float Solver::getVelocityY(float i, float j) {
    return (vy[size * i + j] + vx[size * i + j + 1]) / 2;

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
            if (old_s[size * i + j] == SpaceType::Fluid) {
                int new_i = cutValue(0.1, size - 1, i + dt * getVelocityY(i, j));
                int new_j = cutValue(0.1, size - 1, j + dt * getVelocityX(i, j));
                new_s[size * new_i + new_j] = SpaceType::Fluid;
            }
        }
    }
}

void Solver::dotProduct(float *first, float *second, float *result) {
    for (int i = 0; i < size * size; ++i) {
        result[i] = first[i] * second[i];
    }
}

void Solver::dotDiv(float *first, float *second, float *result) {
    for (int i = 0; i < size * size; ++i) {
        float sec = second[i];
        if (std::abs(sec) < 1e-3) {
            sec = sec + 1e-3;
        }
        result[i] = first[i] / sec;
    }
}

//Умножаем матрицу с 3 диагоналями на вектор
//void Solver::multPressMatrix(float *diag, float *upper_diag, float *under_diag, float *vec, float *result) {
//    for (int i = 0; i < size; ++i) {
//        for (int j = 0; j < size; ++j) {
//            result[size * i + j] = 0;
//            for (int k = 0; k < size; ++k) {
//                result[size * i + j] += first[size * i + k] * second[size * k + j];
//            }
//        }
//    }
//}


