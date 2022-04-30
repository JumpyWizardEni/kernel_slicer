#include "test_class.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cstring>

using std::vector;
using std::max;

Solver::Solver() = default;

void Solver::setParameters(int grid_num, float _dx, vector<int> &_solid_indices, int pSize) {
    size = grid_num;
    subgrid_size = size / sub_domains;
    pressure.resize(size * size, 0);
    particles.resize(pSize);
    gridInfo.resize(size * size, GridPICInfo());
    spaceTypes.resize(size * size, Empty);
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
    prev_vx.resize(size * (size + 1), 0);
    prev_vy.resize(size * (size + 1), 0);
    counts.resize(size * size, 0);
    vx.resize(size * (size + 1), 0);
    vy.resize(size * (size + 1), 0);
    isEnd.resize(1, 0);
    dx = _dx;
    solid_indices = _solid_indices;
    if (solid_indices.size() == 0) {
        solid_indices.push_back(1);
    }


}

int Solver::roundValue(int from, int to, float value) {
    if (value > to) {
        value = to;
    }
    if (value < from) {
        value = from;
    }

    int f = (int) floor(value);
    if (value - (float) f > 0.5) {
        return (int) ceil(value);
    }
    return f;
}

float Solver::getVelocityX(float *vx, int i, int j) {
    float res = (vx[getIdxX(i, j)] + vx[getIdxX(i + 1, j)]) / 2;

    return res;
}

float Solver::getVelocityY(float *vy, int i, int j) {
    float res = (vy[getIdxY(i, j)] + vy[getIdxY(i, j + 1)]) / 2;

    return res;

}

void Solver::performStep(int pSize, const Particle *input, Particle *output) {
    //Обнулить все нужные массивы
    int _s = size * size;
    kernel1D_fillWithZeros_float(_s, rhs.data());
    kernel1D_fillWithZeros_float(_s, pressureResidual.data());
    kernel1D_fillWithZeros_float(_s, press_diag.data());
    kernel1D_fillWithZeros_float(_s, pressX.data());
    kernel1D_fillWithZeros_float(_s, pressY.data());
    kernel1D_fillWithZeros_float(_s, z.data());
    kernel1D_fillWithZeros_float(_s, s.data());
    kernel1D_fillWithZeros_float(_s, diff_vy.data());
    kernel1D_fillWithZeros_float(_s, diff_vx.data());
    kernel1D_fillWithZeros_float(_s, q.data());
    kernel1D_fillWithZeros(_s, counts.data());
    //Перенести частицы в буферы
    int pCopySize = sizeof(Particle) * pSize;
    memcpy(particles.data(), input, pCopySize);

    //Создаем MAC сетку по частицам
    kernel1D_clearSpaceTypes(_s, spaceTypes.data());
    kernel1D_createFluidFromParticles(pSize);
    kernel2D_createSolid(size, size, spaceTypes.data());
    kernel1D_createAdditionalSolid(solid_indices.size(), solid_indices.data(), spaceTypes.data());
//

//    //Переносим скорости
    int _s_vel = size * (size + 1);
    kernel1D_fillWithZeros_float(_s_vel, vx.data());
    kernel1D_fillWithZeros_float(_s_vel, vy.data());
    kernel1D_particlesToGridVelocity(pSize);
    kernel2D_meanVelocities(size, size, vx.data(), vy.data());
//
//    //Считаем новый dt
//    maximum_vel = 0;
//    kernel1D_countTimeDelta(size, vx.data(), vy.data());
//    maximum_vel = std::sqrt(maximum_vel);
//    maximum_vel += std::sqrt(5 * dx * std::abs(g)); // чтобы не было деления на 0
//    dt = 5 * dx / maximum_vel;
    //Сохраняем старые значения скоростей
//
    int velCopySize = sizeof(float) * _s_vel;
    memcpy(prev_vx.data(), vx.data(), velCopySize);
    memcpy(prev_vy.data(), vy.data(), velCopySize);
//
//    //Добавляем силу притяжиния
    kernel2D_addForces(size, size - 1, vy.data(), g, spaceTypes.data());
//    //Зануляем скорости на границах
    kernel2D_dirichleCondition(size + 1, size + 1, spaceTypes.data(), pressure.data(), vx.data(), vy.data());
//
//project
    //Считаем вектор производных скоростей
    kernel2D_calcNegativeDivergence(size, size, spaceTypes.data(), rhs.data(), vx.data(), vy.data());
//    Заполняем матрицу коэффициентов
    kernel2D_fillPressureMatrix(size, size, spaceTypes.data(), press_diag.data(), pressX.data(), pressY.data());
//    Основной алгоритм

    kernel1D_fillWithZeros_float(_s, pressure.data());

    kernel1D_checkZeroRhs(_s);
//
    int d_copy = sizeof(float) * _s;
//    if (!isEnd[0]) {
    memcpy(pressureResidual.data(), rhs.data(), d_copy);
    int preconKernel = sub_domains * sub_domains;
    kernel1D_calcPreconditioner(preconKernel);
    kernel1D_applyPreconditionerForward(preconKernel);
    kernel1D_applyPreconditionerBackward(preconKernel);
//
    memcpy(s.data(), z.data(), d_copy);
//
    dotResult = 0.0;
    kernel1D_dotProduct(_s, z.data(), pressureResidual.data());

    float sygma = dotResult;
    for (int i = 0; i < PCG_MAX_ITERS; ++i) {


        kernel2D_applyPressureMatrix(size, size);
        dotResult = 0.0;

        kernel1D_dotProduct(_s, z.data(), s.data());
        float alpha = dotResult;

        alpha = sygma / alpha;

        kernel1D_changePressure(_s, alpha);

        kernel1D_applyPreconditionerForward(preconKernel);
        kernel1D_applyPreconditionerBackward(preconKernel);
        dotResult = 0.0;

        kernel1D_dotProduct(_s, z.data(), pressureResidual.data());
        float sygma_new = dotResult;

        float beta = sygma_new / sygma;

        kernel1D_changeSearchVector(_s, beta);

        sygma = sygma_new;
    }
//    }
//
//
//
////    //Обновить скорости с помощью давлений
////
    kernel1D_countParticlesNum(pSize);
    kernel2D_changePressureWithParticles(size, size, spaceTypes.data(), counts.data(), pressure.data());
////
    kernel2D_updateVelocities(size - 1, size - 1, spaceTypes.data(), pressure.data(), vx.data(), vy.data());
//    checkDivergence();
////
////    //Снова зануляем во избежании ошибок
    kernel2D_dirichleCondition(size + 1, size + 1, spaceTypes.data(), pressure.data(), vx.data(), vy.data());
////
////    // перенос частиц
    kernel2D_countDiffXY(size, size, vx.data(), vy.data(), prev_vx.data(), prev_vy.data(), diff_vx.data(),
                         diff_vy.data());
    kernel1D_advectParticles(pSize, diff_vx.data(), diff_vy.data(), vx.data(), vy.data(), spaceTypes.data());
    memcpy(output, particles.data(), pCopySize);
}

void Solver::kernel1D_changePressure(int _size, float alpha) {
    for (int j = 0; j < _size; ++j) {
        if (isEnd[0] == 0) {
            pressure[j] += alpha * s[j];
            pressureResidual[j] -= alpha * z[j];
            if (std::abs(pressureResidual[j]) > TOL) {
                isEnd[0] = 1;
            }
        }
    }
}

void Solver::kernel1D_checkZeroRhs(int _size) {
    for (int i = 0; i < _size; ++i) {
        if (std::abs(rhs[i]) > TOL) {
            isEnd[0] = 0;
        }
    }
}

void Solver::kernel1D_changeSearchVector(int _size, float beta) {
    for (int j = 0; j < _size; ++j) {
        s[j] = z[j] + beta * s[j];
    }
}


//Считаем dt на текущем шаге, dt <= 5 * dx / max(velocity)
void Solver::kernel1D_countTimeDelta(int _size, float *p_vx, float *p_vy) {
    //Ищем максимум
    for (int i = 0; i < _size; ++i) {
        maximum_vel = std::max(maximum_vel, p_vx[i] * p_vx[i] + p_vy[i] * p_vy[i]);
    }
}

//Добавляем силу тяжести
void Solver::kernel2D_addForces(int h, int w, float *v, float a, int *_spaceTypes) {
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            if (_spaceTypes[getIdx(i, j)] == Fluid) {
                v[getIdxY(i, j)] += dt * a;
            }
        }
    }
}

float Solver::interpolate(float _q, float *q_copy, float x, float y, int i, int j, float _dx, int _size) {
    float w_x = (ceil(x) - x) * _dx;
    float w_y = (ceil(y) - y) * _dx;

    //Кубическая интерполяция

    //По x
    float w_x_2 = w_x * w_x;
    float w_x_3 = w_x * w_x_2;
    float w0 = -w_x / 3 + w_x_2 / 3 - w_x_3 / 6;
    float w1 = 1 - w_x_2 + (w_x_3 - w_x) / 2;
    float w2 = w_x + (w_x_2 - w_x_3) / 2;
    float w3 = (w_x_3 - w_x) / 6;

    float q_j_1 = w0 * q_copy[i - 1 + _size * (j - 1)] + w1 * q_copy[i + _size * (j - 1)] +
                  w2 * q_copy[i + 1 + _size * (j - 1)] + w3 * q_copy[i + 2 + _size * (j - 1)];

    float q_j = w0 * q_copy[i - 1 + _size * j] + w1 * q_copy[i + _size * j] +
                w2 * q_copy[i + 1 + _size * j] + w3 * q_copy[i + 2 + _size * j];

    float q_j1 = w0 * q_copy[i - 1 + _size * (j + 1)] + w1 * q_copy[i + _size * (j + 1)] +
                 w2 * q_copy[i + 1 + _size * (j + 1)] + w3 * q_copy[i + 2 + _size * (j + 1)];

    float q_j2 = w0 * q_copy[i - 1 + _size * (j + 2)] + w1 * q_copy[i + _size * (j + 2)] +
                 w2 * q_copy[i + 1 + _size * (j + 2)] + w3 * q_copy[i + 2 + _size * (j + 2)];

    //По y
    float w_y_2 = w_y * w_y;
    float w_y_3 = w_y * w_y_2;
    w0 = -w_y / 3 + w_y_2 / 3 - w_y_3 / 6;
    w1 = 1 - w_y_2 + (w_y_3 - w_y) / 2;
    w2 = w_y + (w_y_2 - w_y_3) / 2;
    w3 = (w_y_3 - w_y) / 6;
    _q = w0 * q_j_1 + w1 * q_j + w2 * q_j1 + w3 * q_j2;
    return _q;
}

void
Solver::kernel2D_calcNegativeDivergence(int h, int w, int *_spaceTypes, float *_rhs, float *_vx, float *_vy) {

    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            float scale = 1 / dx;
            if (_spaceTypes[getIdx(i, j)] == Fluid) {
                _rhs[getIdx(i, j)] =
                        -scale *
                        (_vx[getIdxX(i + 1, j)] - _vx[getIdxX(i, j)] + _vy[getIdxY(i, j + 1)] - _vy[getIdxY(i, j)]);
                if (_spaceTypes[getIdx(i - 1, j)] == Solid) {
                    _rhs[getIdx(i, j)] -= scale * _vx[getIdxX(i, j)];
                }
                if (_spaceTypes[getIdx(i + 1, j)] == Solid) {
                    _rhs[getIdx(i, j)] += scale * _vx[getIdxX(i + 1, j)];
                }
                if (_spaceTypes[getIdx(i, j - 1)] == Solid) {
                    _rhs[getIdx(i, j)] -= scale * _vy[getIdxY(i, j)];
                }
                if (_spaceTypes[getIdx(i, j + 1)] == Solid) {
                    _rhs[getIdx(i, j)] += scale * _vy[getIdxY(i, j + 1)];
                }
            }
        }
    }
}

void Solver::kernel2D_fillPressureMatrix(int h, int w, int *_spaceTypes, float *_press_diag, float *_pressX,
                                         float *_pressY) {
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            float scale = dt / (density * dx * dx);
            if (_spaceTypes[getIdx(i, j)] == Fluid) {
                if (i != 0) {
                    if (_spaceTypes[getIdx(i - 1, j)] != Solid) {
                        _press_diag[getIdx(i, j)] += scale;
                    }
                }
                if (i != size - 1) {
                    if (_spaceTypes[getIdx(i + 1, j)] == Fluid) {
                        _press_diag[getIdx(i, j)] += scale;
                        _pressX[getIdx(i, j)] = -scale;
                    } else if (_spaceTypes[getIdx(i + 1, j)] == Empty) {
                        _press_diag[getIdx(i, j)] += scale;
                    }
                }
                if (j != size - 1) {
                    if (_spaceTypes[getIdx(i, j - 1)] != Solid) {
                        _press_diag[getIdx(i, j)] += scale;
                    }
                }
                if (j != 0) {
                    if (_spaceTypes[getIdx(i, j + 1)] == Fluid) {
                        _press_diag[getIdx(i, j)] += scale;
                        _pressY[getIdx(i, j)] = -scale;
                    } else if (_spaceTypes[getIdx(i, j + 1)] == Empty) {
                        _press_diag[getIdx(i, j)] += scale;
                    }
                }
            }
        }
    }
}


//MIC Preconditioner matrix
void Solver::kernel1D_calcPreconditioner(int _size) {

    for (int k = 0; k < _size; k++) {
        int k_x = k % sub_domains;
        int k_y = k / sub_domains;
        for (int j = k_y * subgrid_size - overlap; j < (k_y + 1) * subgrid_size + overlap; ++j) {
            for (int i = k_x * subgrid_size - overlap; i < (k_x + 1) * subgrid_size + overlap; ++i) {
//    for (int j = 0; j < size; ++j) {
//        for (int i = 0; i < size; ++i) {
                if (i >= 0 && j >= 0 && i < size && j < size) {
                    float safe = 0.25;
                    float tau = 0.97;
                    if (spaceTypes[getIdx(i, j)] == Fluid) {
                        float e = press_diag[getIdx(i, j)]
                                  - pow(pressX[getIdx(i - 1, j)] * preconditioner[getIdx(i - 1, j)])
                                  - pow(pressY[getIdx(i, j - 1)] * preconditioner[getIdx(i, j - 1)])
                                  - tau * (pressX[getIdx(i - 1, j)] * pressY[getIdx(i - 1, j)] *
                                           pow(preconditioner[getIdx(i - 1, j)])
                                           + pressY[getIdx(i, j - 1)] * pressX[getIdx(i, j - 1)] *
                                             pow(preconditioner[getIdx(i, j - 1)]));
                        if (e < safe * press_diag[getIdx(i, j)]) {
                            e = press_diag[getIdx(i, j)];
                        }
                        preconditioner[getIdx(i, j)] = 1 / std::sqrt(e);
                    }
                }
            }
        }
    }
}

void Solver::kernel1D_applyPreconditionerBackward(int _size) {

//#pragma omp parallel for num_threads(sub_domains) schedule(dynamic) default(shared)
    for (int k = 0; k < _size; k++) {
        int k_x = k % sub_domains;
        int k_y = k / sub_domains;
        for (int j = (k_y + 1) * subgrid_size + overlap - 1; j >= k_y * subgrid_size - overlap; --j) {
            for (int i = (k_x + 1) * subgrid_size + overlap - 1; i >= (k_x) * subgrid_size - overlap; --i) {
//    for (int j = size - 1; j >= 0; --j) {
//        for (int i = size - 1; i >= 0; --i) {
                if (i >= 0 && j >= 0 && i < size && j < size) {
                    if (spaceTypes[getIdx(i, j)] == Fluid) {
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
    }
}

void Solver::kernel1D_applyPreconditionerForward(
        int _size) {
    //#pragma omp parallel for num_threads(sub_domains) schedule(dynamic) default(shared)
    for (int k = 0; k < _size; k++) {
        int k_x = k % sub_domains;
        int k_y = k / sub_domains;
        for (int j = k_y * subgrid_size - overlap; j < (k_y + 1) * subgrid_size + overlap; ++j) {
            for (int i = k_x * subgrid_size - overlap; i < (k_x + 1) * subgrid_size + overlap; ++i) {
//    for (int j = 0; j < size; ++j) {
//        for (int i = 0; i < size; ++i) {
                if (i >= 0 && j >= 0 && i < size && j < size) {
                    if (spaceTypes[getIdx(i, j)] == Fluid) {
                        float t = pressureResidual[getIdx(i, j)]
                                  - pressX[getIdx(i - 1, j)]
                                    * preconditioner[getIdx(i - 1, j)] * q[getIdx(i - 1, j)]
                                  - pressY[getIdx(i, j - 1)]
                                    * preconditioner[getIdx(i, j - 1)] * q[getIdx(i, j - 1)];
                        q[getIdx(i, j)] = t * preconditioner[getIdx(i, j)];
                    }
                }
            }
        }
    }
}


void Solver::kernel2D_applyPressureMatrix(int h, int w) {
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            if (spaceTypes[getIdx(i, j)] == Fluid) {
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

void Solver::kernel1D_dotProduct(int size, float *first, float *second) {
    for (int i = 0; i < size; ++i) {
        dotResult += first[i] * second[i];
    }
}

//Обновляем скорости с помощью вычисленного вектора давлений
void Solver::kernel2D_updateVelocities(int h, int w, int *_spaceTypes, float *_pressure, float *_vx, float *_vy) {
    for (int j = 1; j < h; ++j) {
        for (int i = 1; i < w; ++i) {
            float scale = dt / (density * dx);
            if (_spaceTypes[getIdx(i - 1, j)] == Fluid ||
                _spaceTypes[getIdx(i, j)] == Fluid) {
                if (_spaceTypes[getIdx(i - 1, j)] == Solid ||
                    _spaceTypes[getIdx(i, j)] == Solid) {
                    _vx[getIdxX(i, j)] = 0;
                } else {
                    _vx[getIdxX(i, j)] -= scale * (_pressure[getIdx(i, j)] - _pressure[getIdx(i - 1, j)]);
                }
            }
            if (_spaceTypes[getIdx(i, j - 1)] == Fluid ||
                _spaceTypes[getIdx(i, j)] == Fluid) {
                if (_spaceTypes[getIdx(i, j - 1)] == Solid ||
                    _spaceTypes[getIdx(i, j)] == Solid) {
                    _vy[getIdxY(i, j)] = 0;

                } else {
                    _vy[getIdxY(i, j)] -= scale * (_pressure[getIdx(i, j)] - _pressure[getIdx(i, j - 1)]);
                }
            }

        }
    }
}

void Solver::kernel2D_changePressureWithParticles(int h, int w, int *spaceTypes, int *counts, float *pressure) {
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            if (spaceTypes[getIdx(i, j)] == Fluid) {
                int c = counts[getIdx(i, j)];
                if (c > 8) {
                    pressure[getIdx(i, j)] += particles_pressure_coef * (c - 8);
                }
            }
        }
    }
}

void Solver::kernel1D_countParticlesNum(int _size) {
    for (int i = 0; i < _size; ++i) {
        int x = roundValue(1, Solver::size - 2, particles[i].pos_x / dx);
        int y = roundValue(1, Solver::size - 2, particles[i].pos_y / dx);
        counts[getIdx(x, y)]++;
    }
}

void Solver::kernel1D_fillWithZeros_float(int _size, float *v) {
    for (int i = 0; i < _size; ++i) {
        v[i] = 0.0f;
    }
}

void Solver::kernel1D_fillWithZeros(int _size, int *v) {
    for (int i = 0; i < _size; ++i) {
        v[i] = 0;
    }
}

int Solver::getIdx(int i, int j) {
    return i + size * j;
}

int Solver::getIdxY(int i, int j) {
    return i + size * j;
}

int Solver::getIdxX(int i, int j) {
    return i + (size + 1) * j;
}

void Solver::checkDivergence() {
    float max_div = 0;
    int max_i = -1;
    int max_j = -1;
    for (int j = 0; j < size; ++j) {
        for (int i = 0; i < size; ++i) {
            float div =
                    (vx[getIdxX(i + 1, j)] - vx[getIdxX(i, j)]) / dx +
                    (vy[getIdxY(i, j + 1)] - vy[getIdxY(i, j)]) / dx;
            if (std::abs(div) > std::abs(max_div) && spaceTypes[getIdx(i, j)] == Fluid) {
                max_div = div;
                max_i = i;
                max_j = j;
            }
        }
    }
    std::cout << "max_div: " << max_div << ", max_i: " << max_i << ", max_j: " << max_j << std::endl;
}

void
Solver::kernel2D_dirichleCondition(int h, int w, int *_spaceTypes, float *_pressure, float *_vx, float *_vy) {

    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            if (j < size && i < size && _spaceTypes[getIdx(i, j)] != Fluid) {
                _pressure[getIdx(i, j)] = 0;
            }
            if (j < size) {
                if (i > 0) {
                    if (_spaceTypes[getIdx(i - 1, j)] == Solid && _vx[getIdxX(i, j)] < 0) {
                        _vx[getIdxX(i, j)] = 0;
                    }
                }
                if (i < size) {
                    if (_spaceTypes[getIdx(i, j)] == Solid && _vx[getIdxX(i, j)] > 0) {
                        _vx[getIdxX(i, j)] = 0;
                    }
                }
            }
            if (i < size) {
                if (j > 0) {
                    if (_spaceTypes[getIdx(i, j - 1)] == Solid && _vy[getIdxY(i, j)] < 0) {
                        _vy[getIdxY(i, j)] = 0;
                    }
                }
                if (j < size) {
                    if (_spaceTypes[getIdx(i, j)] == Solid && _vy[getIdxY(i, j)] > 0) {
                        _vy[getIdxY(i, j)] = 0;
                    }
                }
            }

        }
    }
}

float Solver::pow(float value) {
    return value * value;
}

void Solver::kernel1D_clearSpaceTypes(int _size, int *_spaceTypes) {
    for (int i = 0; i < _size; ++i) {
        _spaceTypes[i] = Empty;
    }
}

void Solver::kernel2D_createSolid(int h, int w, int *_spaceTypes) {
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            if (i == 0 || i == size - 1 || j == 0 || j == size - 1) {
                _spaceTypes[getIdx(i, j)] = Solid;
            }
        }
    }
}

void Solver::kernel1D_createFluidFromParticles(int _size) {
    for (int i = 0; i < _size; ++i) {
        int x = roundValue(0, size - 1, particles[i].pos_x / dx);
        int y = roundValue(0, size - 1, particles[i].pos_y / dx);
        spaceTypes[getIdx(x, y)] = Fluid;
    }
}

float Solver::h2(float x) {

    if (x >= -1.5f && x < -0.5f) {
        return 0.5f * pow(x + 1.5f);
    }
    if (x >= -0.5f && x < 0.5f) {
        return 0.75f - x * x;
    }
    if (x >= 0.5f && x < 1.5f) {
        return 0.5f * pow(1.5f - x);
    }
    return 0;
};

//kernel function for interpolating particle values
float Solver::kFunc(float x, float y) {
    return h2(x / dx) * h2(y / dx);
}

void Solver::kernel2D_meanVelocities(int h, int w, float *_vx, float *_vy) {
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            if (gridInfo[getIdx(i, j)].sum_vx > 0.0001) {
                _vx[getIdxX(i, j)] = gridInfo[getIdx(i, j)].sum_vx / gridInfo[getIdx(i, j)].weight_vx;
            }
            if (gridInfo[getIdx(i, j)].sum_vy > 0.0001) {
                _vy[getIdxY(i, j)] = gridInfo[getIdx(i, j)].sum_vy / gridInfo[getIdx(i, j)].weight_vy;
            }

        }
    }
}

void Solver::kernel1D_particlesToGridVelocity(int _size) {
    for (int i = 0; i < _size; ++i) {
        int x = roundValue(1, size - 2, particles[i].pos_x / dx);
        int y = roundValue(1, size - 2, particles[i].pos_y / dx);
        float vx_k = kFunc(particles[i].pos_x - (x - 0.5f) * dx, particles[i].pos_y - y * dx);
        float vy_k = kFunc(particles[i].pos_x - x * dx, particles[i].pos_y - (y - 0.5f) * dx);
        gridInfo[getIdx(x, y)].sum_vx += particles[i].vx * vx_k;
        gridInfo[getIdx(x, y)].sum_vy += particles[i].vy * vy_k;
        gridInfo[getIdx(x, y)].weight_vx += vx_k;
        gridInfo[getIdx(x, y)].weight_vy += vy_k;
    }
}

void
Solver::kernel2D_countDiffXY(int h, int w, float *_vx, float *_vy, float *_prev_vx, float *_prev_vy,
                             float *_diff_vx,
                             float *_diff_vy) {
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            _diff_vx[getIdx(i, j)] = (_vx[getIdxX(i, j)] + _vx[getIdxX(i + 1, j)]) / 2 -
                                     (_prev_vx[getIdxX(i, j)] + _prev_vx[getIdxX(i + 1, j)]) / 2;
            _diff_vy[getIdx(i, j)] = (_vy[getIdxY(i, j)] + _vy[getIdxY(i, j + 1)]) / 2 -
                                     (_prev_vy[getIdxY(i, j)] + _prev_vy[getIdxY(i, j + 1)]) / 2;;
        }
    }
}

void
Solver::kernel1D_advectParticles(int _size, float *_diff_vx, float *_diff_vy, float *_vx, float *_vy,
                                 int *_spaceTypes) {
    for (int i = 0; i < _size; ++i) {
        float alpha = getAlpha();
        float _dx = dx;

        int x = roundValue(0, size - 1, particles[i].pos_x / _dx);
        int y = roundValue(0, size - 1, particles[i].pos_y / _dx);


        float vx_interpolated = interpolate(_diff_vx[getIdx(x, y)], _diff_vx, particles[i].pos_x / _dx,
                                            particles[i].pos_y / _dx, x, y, _dx, size);
        float vy_interpolated = interpolate(_diff_vy[getIdx(x, y)], _diff_vy, particles[i].pos_x / _dx,
                                            particles[i].pos_y / _dx, x,
                                            y, _dx, size);
        particles[i].vx = alpha * (_vx[getIdxX(x, y)] + _vx[getIdxX(x + 1, y)]) / 2 +
                          (1 - alpha) * (particles[i].vx + vx_interpolated);
        particles[i].vy =
                alpha * (_vy[getIdxY(x, y)] + _vy[getIdxY(x, y + 1)]) / 2 +
                (1 - alpha) * (particles[i].vy + vy_interpolated);

        //По оси X
        particles[i].pos_x += particles[i].vx * dt;

        x = roundValue(0, size - 1, particles[i].pos_x / _dx);

        //Границы
        while (x == 0 || particles[i].pos_x < 0 || x == size - 1 || particles[i].pos_x > _dx * size) {
            particles[i].pos_x -= particles[i].vx * dt;
            x = roundValue(0, size - 1, particles[i].pos_x / _dx);
        }

        //Другие твердые клетки
        while (_spaceTypes[getIdx(x, y)] == Solid) {
            particles[i].pos_x -= particles[i].vx * dt;
            x = roundValue(0, size - 1, particles[i].pos_x / _dx);
        }

        //По оси Y
        particles[i].pos_y += particles[i].vy * dt;
        y = roundValue(0, size - 1, particles[i].pos_y / _dx);

        //Границы
        while (y == 0 || particles[i].pos_y < 0 || y == size - 1 || particles[i].pos_y > _dx * size) {
            particles[i].pos_y -= particles[i].vy * dt;
            y = roundValue(0, size - 1, particles[i].pos_y / _dx);
        }
//
        //Другие твердые клетки
        while (_spaceTypes[getIdx(x, y)] == Solid) {
            particles[i].pos_y -= particles[i].vy * dt;
            y = roundValue(0, size - 1, particles[i].pos_y / _dx);
        }
//
//
        while (_spaceTypes[getIdx(x, y - 1)] == Solid && particles[i].vy <= 1 &&
               _spaceTypes[getIdx(x, y + 1)] == Empty) {
            particles[i].pos_y += _dx;
            particles[i].vy += g * dt;
            y = roundValue(0, size - 1, particles[i].pos_y / _dx);
        }
//

    }
}

float Solver::getAlpha() {
    float res = 6 * dt * viscosity / (dx * dx);
    if (res < 0) {
        res = 0;
    }
    if (res > 1) {
        res = 1;
    }
    return res;
}

void Solver::kernel1D_createAdditionalSolid(int _size, int *indices, int *_spaceTypes) {
    for (int i = 0; i < _size; ++i) {
        _spaceTypes[indices[i]] = Solid;
    }
}

void Solver::kernel1D_changeParticlesSize(int t) {
    particles.resize(0);
    for (int i = 0; i < t; ++i) {
        Particle p;
        particles.push_back(p);
    }
}
