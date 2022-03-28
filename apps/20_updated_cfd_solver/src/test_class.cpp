#include "test_class.h"
#include <cmath>
#include <algorithm>
#include <cstring>

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
    prev_vx.resize(size * (size + 1), 0);
    prev_vy.resize(size * (size + 1), 0);
}
//
//int Solver::roundValue(int from, int to, double value) {
//    if (value > to) {
//        value = to;
//    }
//    if (value < from) {
//        value = from;
//    }
//    return (int) std::round(value);
//}
////
//double Solver::getVelocityX(double *vx, int i, int j) {
//    double res = (vx[getIdxX(i, j)] + vx[getIdxX(i + 1, j)]) / 2;
//
//    return res;
//}
//
//double Solver::getVelocityY(double *vy, int i, int j) {
//    double res = (vy[getIdxY(i, j)] + vy[getIdxY(i, j + 1)]) / 2;
//
//    return res;
//
//}

void Solver::performStep(double *output) {
    //Зануляем необходимые массивы
//    resetParams();

    //Создаем MAC сетку по частицам
//    kernel1D_clearSpaceTypes(size * size, spaceTypes.data());
//    kernel1D_createFluidFromParticles(particles_size, particles.data(), spaceTypes.data());
//    kernel2D_createSolid(size, size, spaceTypes.data());

    //Переносим скорости
//    fillWithValue(vx.data(), size * (size + 1));
//    fillWithValue(vy.data(), size * (size + 1));
//    kernel1D_particlesToGridVelocity(particles_size, particles.data(), gridInfo.data());
//    kernel2D_meanVelocities(0, 0, nullptr, nullptr, nullptr);

    //Считаем новый dt
//    countTimeDelta(vx.data(), vy.data());

    //Сохраняем старые значения скоростей
//    memcpy(prev_vx.data(), vx.data(), sizeof(double) * size * (size + 1));
//    memcpy(prev_vy.data(), vy.data(), sizeof(double) * size * (size + 1));

    //Добавляем силу притяжиния
    kernel2D_addForces(size, size - 1, vy.data(), g, spaceTypes.data());

    //Зануляем скорости на границах
//    kernel2D_dirichleCondition(size + 1, size + 1, spaceTypes.data(), pressure.data(), vx.data(), vy.data());
//    extrapolateVelocities();

//project
    //Считаем вектор производных скоростей
//    kernel2D_calcNegativeDivergence(size, size, spaceTypes.data(), rhs.data(), vx.data(), vy.data());
    //Заполняем матрицу коэффициентов
//    kernel2D_fillPressureMatrix(size, size, spaceTypes.data(), press_diag.data(), pressX.data(), pressY.data());
    //Основной алгоритм

//    fillWithValue(pressure.data(), size * size);

//    bool isEnd = true;
//    for (int i = 0; i < size * size; ++i) {
//        if (std::abs(rhs[i]) > TOL) {
//            isEnd = false;
//            break;
//        }
//    }
//
//    if (isEnd) {
//        return;
//    }

//    pressureResidual = rhs;

//    calcPreconditioner();

//    applyPreconditioner();

//    s = z;

//    double sygma = dotProduct(z.data(), pressureResidual.data());
//    for (int i = 0; i < PCG_MAX_ITERS; ++i) {


//        applyPressureMatrix();
//        double alpha = sygma / dotProduct(z.data(), s.data());

//        int endFlag = 1;

//        for (int j = 0; j < size * size; ++j) {
//            pressure[j] += alpha * s[j];
//            pressureResidual[j] -= alpha * z[j];
//            if (std::abs(pressureResidual[j]) > TOL) {
//                endFlag = 0;
//            }
//        }

        //Вектор давлений найден
//        if (endFlag) {
//            return;
//        }
//
//        applyPreconditioner();
//
//        double sygma_new = dotProduct(z.data(), pressureResidual.data());
//
//        double beta = sygma_new / sygma;
//
//        for (int j = 0; j < size * size; ++j) {
//            s[j] = z[j] + beta * s[j];
//        }
//
//        sygma = sygma_new;
//    }

//    //Обновить скорости с помощью давлений
//    kernel2D_updateVelocities(size - 1, size - 1, spaceTypes.data(), pressure.data(), vx.data(), vy.data());
//    //Снова зануляем во избежании ошибок
//    kernel2D_dirichleCondition(size + 1, size + 1, spaceTypes.data(), pressure.data(), vx.data(), vy.data());
//
//    // перенос частиц
//    kernel2D_countDiffXY(size, size, vx.data(), vy.data(), prev_vx.data(), prev_vy.data(), diff_vx.data(),
//                         diff_vy.data());
//
//    kernel1D_advectParticles(particles_size, particles.data(), diff_vx.data(), diff_vy.data(), spaceTypes.data());
//    int copy_size = sizeof(float) * size * size;
//    memcpy((float *)output, (float *)pressure.data(), copy_size);
//    checkDivergence();
}


//Считаем dt на текущем шаге, dt <= 5 * dx / max(velocity)
//void Solver::countTimeDelta(const double *p_vx, const double *p_vy) {
//    //Ищем максимум
//    double maximum = -1;
//    for (int i = 0; i < size * (size + 1); ++i) {
//        double vel = p_vx[i] * p_vx[i] + p_vy[i] * p_vy[i];
//        if (vel > maximum) {
//            maximum = vel;
//        }
//    }
//    maximum = std::sqrt(maximum);
//    maximum += std::sqrt(5 * dx * std::abs(g)); // чтобы не было деления на 0
//    dt = 3 * dx / maximum;
//}

//Добавляем силу тяжести
void Solver::kernel2D_addForces(int h, int w, double *v, double a, SpaceType *_spaceTypes) {
    for (int j = 0; j < h; ++j) {
        for (int i = 0; i < w; ++i) {
            if (_spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
                v[getIdxY(i, j)] += dt * a;
            }
        }
    }
}
//
//double Solver::interpolate(double q, double *q_copy, double x, double y, int i, int j) {
//    double w_x = (ceil(x) - x) * dx;
//    double w_y = (ceil(y) - y) * dx;
//
//    //Кубическая интерполяция
//
//    //По x
//    double w_x_2 = w_x * w_x;
//    double w_x_3 = w_x * w_x_2;
//    double w0 = -w_x / 3 + w_x_2 / 3 - w_x_3 / 6;
//    double w1 = 1 - w_x_2 + (w_x_3 - w_x) / 2;
//    double w2 = w_x + (w_x_2 - w_x_3) / 2;
//    double w3 = (w_x_3 - w_x) / 6;
//
//    double q_j_1 = w0 * q_copy[getIdx(i - 1, j - 1)] + w1 * q_copy[getIdx(i, j - 1)] +
//                   w2 * q_copy[getIdx(i + 1, j - 1)] + w3 * q_copy[getIdx(i + 2, j - 1)];
//
//    double q_j = w0 * q_copy[getIdx(i - 1, j)] + w1 * q_copy[getIdx(i, j)] +
//                 w2 * q_copy[getIdx(i + 1, j)] + w3 * q_copy[getIdx(i + 2, j)];
//
//    double q_j1 = w0 * q_copy[getIdx(i - 1, j + 1)] + w1 * q_copy[getIdx(i, j + 1)] +
//                  w2 * q_copy[getIdx(i + 1, j + 1)] + w3 * q_copy[getIdx(i + 2, j + 1)];
//
//    double q_j2 = w0 * q_copy[getIdx(i - 1, j + 2)] + w1 * q_copy[getIdx(i, j + 2)] +
//                  w2 * q_copy[getIdx(i + 1, j + 2)] + w3 * q_copy[getIdx(i + 2, j + 2)];
//
//    //По y
//    double w_y_2 = w_y * w_y;
//    double w_y_3 = w_y * w_y_2;
//    w0 = -w_y / 3 + w_y_2 / 3 - w_y_3 / 6;
//    w1 = 1 - w_y_2 + (w_y_3 - w_y) / 2;
//    w2 = w_y + (w_y_2 - w_y_3) / 2;
//    w3 = (w_y_3 - w_y) / 6;
//    q = w0 * q_j_1 + w1 * q_j + w2 * q_j1 + w3 * q_j2;
//    return q;
//}
//
//void
//Solver::kernel2D_calcNegativeDivergence(int h, int w, SpaceType *_spaceTypes, double *_rhs, double *_vx, double *_vy) {
//
//    double scale = 1 / dx;
//    for (int j = 0; j < h; ++j) {
//        for (int i = 0; i < w; ++i) {
//            if (_spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
//                _rhs[getIdx(i, j)] =
//                        -scale *
//                        (_vx[getIdxX(i + 1, j)] - _vx[getIdxX(i, j)] + _vy[getIdxY(i, j + 1)] - _vy[getIdxY(i, j)]);
//                if (_spaceTypes[getIdx(i - 1, j)] == SpaceType::Solid) {
//                    _rhs[getIdx(i, j)] -= scale * _vx[getIdxX(i, j)];
//                }
//                if (_spaceTypes[getIdx(i + 1, j)] == SpaceType::Solid) {
//                    _rhs[getIdx(i, j)] += scale * _vx[getIdxX(i + 1, j)];
//                }
//                if (_spaceTypes[getIdx(i, j - 1)] == SpaceType::Solid) {
//                    _rhs[getIdx(i, j)] -= scale * _vy[getIdxY(i, j)];
//                }
//                if (_spaceTypes[getIdx(i, j + 1)] == SpaceType::Solid) {
//                    _rhs[getIdx(i, j)] += scale * _vy[getIdxY(i, j + 1)];
//                }
//            }
//        }
//    }
//}
//
//void Solver::kernel2D_fillPressureMatrix(int h, int w, SpaceType *_spaceTypes, double *_press_diag, double *_pressX,
//                                         double *_pressY) {
//    double scale = dt / (density * dx * dx);
//    for (int j = 0; j < h; ++j) {
//        for (int i = 0; i < w; ++i) {
//            if (_spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
//                if (i != 0) {
//                    if (_spaceTypes[getIdx(i - 1, j)] != SpaceType::Solid) {
//                        _press_diag[getIdx(i, j)] += scale;
//                    }
//                }
//                if (i != size - 1) {
//                    if (_spaceTypes[getIdx(i + 1, j)] == SpaceType::Fluid) {
//                        _press_diag[getIdx(i, j)] += scale;
//                        _pressX[getIdx(i, j)] = -scale;
//                    } else if (_spaceTypes[getIdx(i + 1, j)] == SpaceType::Empty) {
//                        _press_diag[getIdx(i, j)] += scale;
//                    }
//                }
//                if (j != size - 1) {
//                    if (_spaceTypes[getIdx(i, j - 1)] != SpaceType::Solid) {
//                        _press_diag[getIdx(i, j)] += scale;
//                    }
//                }
//                if (j != 0) {
//                    if (_spaceTypes[getIdx(i, j + 1)] == SpaceType::Fluid) {
//                        _press_diag[getIdx(i, j)] += scale;
//                        _pressY[getIdx(i, j)] = -scale;
//                    } else if (_spaceTypes[getIdx(i, j + 1)] == SpaceType::Empty) {
//                        _press_diag[getIdx(i, j)] += scale;
//                    }
//                }
//            }
//        }
//    }
//}
//
//
////MIC Preconditioner matrix
//void Solver::calcPreconditioner() {
//    double safe = 0.25;
//    double tau = 0.97;
//    for (int j = 1; j < size - 1; ++j) {
//        for (int i = 1; i < size - 1; ++i) {
//            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
//                double e = press_diag[getIdx(i, j)]
//                           - std::pow(pressX[getIdx(i - 1, j)] * preconditioner[getIdx(i - 1, j)], 2)
//                           - std::pow(pressY[getIdx(i, j - 1)] * preconditioner[getIdx(i, j - 1)], 2)
//                           - tau * (pressX[getIdx(i - 1, j)] * pressY[getIdx(i - 1, j)] *
//                                    std::pow(preconditioner[getIdx(i - 1, j)], 2)
//                                    + pressY[getIdx(i, j - 1)] * pressX[getIdx(i, j - 1)] *
//                                      std::pow(preconditioner[getIdx(i, j - 1)], 2));
//                if (e < safe * press_diag[getIdx(i, j)]) {
//                    e = press_diag[getIdx(i, j)];
//                }
//                preconditioner[getIdx(i, j)] = 1 / std::sqrt(e);
//            }
//        }
//    }
//}
//
//
//void Solver::applyPreconditioner() {
//    int overlap = 2;
//    int sub_domains = 5;
//    int subgrid_size = size / sub_domains;
//    //10
////#pragma omp parallel for num_threads(sub_domains) schedule(dynamic) default(shared)
////    for (int k = 0; k < sub_domains * sub_domains; k++) {
////        int k_x = k % sub_domains;
////        int k_y = k / sub_domains;
////        for (int j = k_y * subgrid_size - overlap; j < (k_y + 1) * subgrid_size + overlap; ++j) {
////            for (int i = k_x * subgrid_size - overlap; i < (k_x + 1) * subgrid_size + overlap; ++i) {
//    for (int j = 0; j < size; ++j) {
//        for (int i = 0; i < size; ++i) {
//            if (i >= 0 && j >= 0 && i < size && j < size) {
//                if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
//                    double t = pressureResidual[getIdx(i, j)]
//                               - pressX[getIdx(i - 1, j)]
//                                 * preconditioner[getIdx(i - 1, j)] * q[getIdx(i - 1, j)]
//                               - pressY[getIdx(i, j - 1)]
//                                 * preconditioner[getIdx(i, j - 1)] * q[getIdx(i, j - 1)];
//                    q[getIdx(i, j)] = t * preconditioner[getIdx(i, j)];
//                }
//            }
////            }
//        }
//    }
////#pragma omp parallel for num_threads(sub_domains) schedule(dynamic) default(shared)
////    for (int k = 0; k < sub_domains * sub_domains; k++) {
////        int k_x = k % sub_domains;
////        int k_y = k / sub_domains;
////        for (int j = (k_y + 1) * subgrid_size + overlap - 1; j >= k_y * subgrid_size - overlap; --j) {
////            for (int i = (k_x + 1) * subgrid_size + overlap - 1; i >= (k_x) * subgrid_size - overlap; --i) {
//    for (int j = size - 1; j >= 0; --j) {
//        for (int i = size - 1; i >= 0; --i) {
//            if (i >= 0 && j >= 0 && i < size && j < size) {
//                if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
//                    double t = q[getIdx(i, j)]
//                               - pressX[getIdx(i, j)]
//                                 * preconditioner[getIdx(i, j)] * z[getIdx(i + 1, j)]
//                               - pressY[getIdx(i, j)]
//                                 * preconditioner[getIdx(i, j)] * z[getIdx(i, j + 1)];
//                    z[getIdx(i, j)] = t * preconditioner[getIdx(i, j)];
//                }
//            }
////            }
//        }
//    }
//}
//
//
//void Solver::applyPressureMatrix() {
//    for (int j = 1; j < size - 1; ++j) {
//        for (int i = 1; i < size - 1; ++i) {
//            if (spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
//                double value = 0.0;
//                value += s[getIdx(i - 1, j)] * pressX[getIdx(i - 1, j)];
//                value += s[getIdx(i + 1, j)] * pressX[getIdx(i, j)];
//                value += s[getIdx(i, j - 1)] * pressY[getIdx(i, j - 1)];
//                value += s[getIdx(i, j + 1)] * pressY[getIdx(i, j)];
//                value += s[getIdx(i, j)] * press_diag[getIdx(i, j)];
//
//                z[getIdx(i, j)] = value;
//            }
//        }
//    }
//}
//
//double Solver::dotProduct(double *first, double *second) {
//    double sum = 0.0;
//    for (int i = 0; i < size * size; ++i) {
//        sum += first[i] * second[i];
//    }
//    return sum;
//}
//
////Обновляем скорости с помощью вычисленного вектора давлений
//void
//Solver::kernel2D_updateVelocities(int h, int w, SpaceType *_spaceTypes, double *_pressure, double *_vx, double *_vy) {
//    double scale = dt / (density * dx);
//    for (int j = 1; j < size - 1; ++j) {
//        for (int i = 1; i < size - 1; ++i) {
//            if (_spaceTypes[getIdx(i - 1, j)] == SpaceType::Fluid ||
//                _spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
//                if (_spaceTypes[getIdx(i - 1, j)] == SpaceType::Solid ||
//                    _spaceTypes[getIdx(i, j)] == SpaceType::Solid) {
//                    _vx[getIdxX(i, j)] = 0;
//                } else {
//                    _vx[getIdxX(i, j)] -= scale * (_pressure[getIdx(i, j)] - _pressure[getIdx(i - 1, j)]);
//                }
//            }
//            if (_spaceTypes[getIdx(i, j - 1)] == SpaceType::Fluid ||
//                _spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
//                if (_spaceTypes[getIdx(i, j - 1)] == SpaceType::Solid ||
//                    _spaceTypes[getIdx(i, j)] == SpaceType::Solid) {
//                    _vy[getIdxY(i, j)] = 0;
//
//                } else {
//                    _vy[getIdxY(i, j)] -= scale * (_pressure[getIdx(i, j)] - _pressure[getIdx(i, j - 1)]);
//                }
//            }
//
//        }
//    }
//}
//
//void Solver::fillWithValue(double *v, int size, double value) {
//    for (int i = 0; i < size; ++i) {
//        v[i] = value;
//    }
//}
//
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
//
//void Solver::resetParams() {
//    int s = size * size;
//    fillWithValue(rhs.data(), s);
//    fillWithValue(pressureResidual.data(), s);
//    fillWithValue(press_diag.data(), s);
//    fillWithValue(pressX.data(), s);
//    fillWithValue(pressY.data(), s);
//    fillWithValue(z.data(), s);
//    fillWithValue(this->s.data(), s);
//    fillWithValue(diff_vy.data(), s);
//    fillWithValue(diff_vx.data(), s);
//    fillWithValue(q.data(), s);
//}
//
////void Solver::checkDivergence() {
////    double max_div = 0;
////    int max_i = -1;
////    int max_j = -1;
////    for (int j = 0; j < size; ++j) {
////        for (int i = 0; i < size; ++i) {
////            double div =
////                    (vx[getIdxX(i + 1, j)] - vx[getIdxX(i, j)]) / dx +
////                    (vy[getIdxY(i, j + 1)] - vy[getIdxY(i, j)]) / dx;
////            if (std::abs(div) > std::abs(max_div) && spaceTypes[getIdx(i, j)] == SpaceType::Fluid) {
////                max_div = div;
////                max_i = i;
////                max_j = j;
////            }
////        }
////    }
////    std::cout << "max_div: " << max_div << ", max_i: " << max_i << ", max_j: " << max_j << std::endl;
////}
//
//void
//Solver::kernel2D_dirichleCondition(int h, int w, SpaceType *spaceTypes, double *_pressure, double *_vx, double *_vy) {
//
//    for (int j = 0; j < h; ++j) {
//        for (int i = 0; i < w; ++i) {
//            if (j < size && i < size && spaceTypes[getIdx(i, j)] != SpaceType::Fluid) {
//                _pressure[getIdx(i, j)] = 0;
//            }
//            if (j < size) {
//                if (i > 0) {
//                    if (spaceTypes[getIdx(i - 1, j)] == SpaceType::Solid && _vx[getIdxX(i, j)] < 0) {
//                        _vx[getIdxX(i, j)] = 0;
//                    }
//                }
//                if (i < size) {
//                    if (spaceTypes[getIdx(i, j)] == SpaceType::Solid && _vx[getIdxX(i, j)] > 0) {
//                        _vx[getIdxX(i, j)] = 0;
//                    }
//                }
//            }
//            if (i < size) {
//                if (j > 0) {
//                    if (spaceTypes[getIdx(i, j - 1)] == SpaceType::Solid && _vy[getIdxY(i, j)] < 0) {
//                        _vy[getIdxY(i, j)] = 0;
//                    }
//                }
//                if (j < size) {
//                    if (spaceTypes[getIdx(i, j)] == SpaceType::Solid && _vy[getIdxY(i, j)] > 0) {
//                        _vy[getIdxY(i, j)] = 0;
//                    }
//                }
//            }
//
//        }
//    }
//}
//
//void Solver::kernel1D_clearSpaceTypes(int s, SpaceType *spaceTypes) {
//    for (int i = 0; i < s; ++i) {
//        spaceTypes[i] = SpaceType::Empty;
//    }
//}
//
//void Solver::kernel2D_createSolid(int h, int w, SpaceType *_spaceTypes) {
//    for (int j = 0; j < h; ++j) {
//        for (int i = 0; i < w; ++i) {
//            if (i == 0 || i == size - 1 || j == 0 || j == size - 1) {
//                _spaceTypes[getIdx(i, j)] = SpaceType::Solid;
//            }
//        }
//    }
//}
//
//void Solver::kernel1D_createFluidFromParticles(int s, Particle *_particles, SpaceType *_spaceTypes) {
//    for (int i = 0; i < s; ++i) {
//        int x = roundValue(0, size - 1, _particles[i].pos_x / dx);
//        int y = roundValue(0, size - 1, _particles[i].pos_y / dx);
//        _spaceTypes[getIdx(x, y)] = SpaceType::Fluid;
//    }
//}
//
//double Solver::h2(double x) {
//
//    if (x >= -1.5 && x < -0.5) {
//        return 0.5 * pow(x + 1.5, 2);
//    }
//    if (x >= -0.5 && x < 0.5) {
//        return 0.75 - x * x;
//    }
//    if (x >= 0.5 && x < 1.5) {
//        return 0.5 * pow(1.5 - x, 2);
//    }
//    return 0;
//};
//
////kernel function for interpolating particle values
//double Solver::kFunc(double x, double y) {
//    return h2(x / dx) * h2(y / dx);
//}
//
//void Solver::kernel2D_meanVelocities(int h, int w, GridPICInfo *_gridInfo, double *_vx, double *_vy) {
//    for (int j = 0; j < h; ++j) {
//        for (int i = 0; i < w; ++i) {
//            GridPICInfo &info = _gridInfo[getIdx(i, j)];
//            if (info.sum_vx > 0.0001) {
//                _vx[getIdxX(i, j)] = info.sum_vx / info.weight_vx;
//            }
//            if (info.sum_vy > 0.0001) {
//                _vy[getIdxY(i, j)] = info.sum_vy / info.weight_vy;
//            }
//
//        }
//    }
//}
//
//void Solver::kernel1D_particlesToGridVelocity(int s, Particle *_particles, GridPICInfo *_gridInfo) {
//    for (int i = 0; i < s; ++i) {
//        Particle &particle = _particles[i];
//
//        int x = roundValue(1, size - 2, particle.pos_x / dx);
//        int y = roundValue(1, size - 2, particle.pos_y / dx);
//        GridPICInfo &info = _gridInfo[getIdx(x, y)];
//        double vx_k = kFunc(particle.pos_x - (x - 0.5) * dx, particle.pos_y - y * dx);
//        double vy_k = kFunc(particle.pos_x - x * dx, particle.pos_y - (y - 0.5) * dx);
//        info.sum_vx += particle.vx * vx_k;
//        info.sum_vy += particle.vy * vy_k;
//        info.weight_vx += vx_k;
//        info.weight_vy += vy_k;
//    }
//}
//
//void
//Solver::kernel2D_countDiffXY(int h, int w, double *_vx, double *_vy, double *_prev_vx, double *_prev_vy,
//                             double *_diff_vx,
//                             double *_diff_vy) {
//    for (int j = 0; j < size; ++j) {
//        for (int i = 0; i < size; ++i) {
//            _diff_vx[getIdx(i, j)] = getVelocityX(_vx, i, j) - getVelocityX(_prev_vx, i, j);
//            _diff_vy[getIdx(i, j)] = getVelocityY(_vy, i, j) - getVelocityY(_prev_vy, i, j);
//        }
//    }
//}
//
//
//void
//Solver::kernel1D_advectParticles(int s, Particle *_particles, double *_diff_vx, double *_diff_vy,
//                                 SpaceType *_spaceTypes) {
//    for (int i = 0; i < s; ++i) {
//        Particle &particle = _particles[i];
//        int x = roundValue(1, size - 2, particle.pos_x / dx);
//        int y = roundValue(1, size - 2, particle.pos_y / dx);
//
//        double vx_interpolated = interpolate(_diff_vx[getIdx(x, y)], _diff_vx, particle.pos_x / dx,
//                                             particle.pos_y / dx, x, y);
//        double vy_interpolated = interpolate(_diff_vy[getIdx(x, y)], _diff_vy, particle.pos_x / dx,
//                                             particle.pos_y / dx, x,
//                                             y);
//        particle.vx += vx_interpolated;
//        particle.vy += vy_interpolated;
//        particle.pos_x += particle.vx * dt;
//        particle.pos_y += particle.vy * dt;
//
//
//        double velocityCoef = 0.8;
//        double dxCoef = 1;
//
//        if (particle.vx > 0) {
//            if (particle.pos_x > dx * size) {
//                particle.pos_x = 2 * dx * size - particle.pos_x - 2 * dxCoef * dx;
//                particle.vx = -particle.vx * velocityCoef;
//            } else if (particle.pos_x > dx * size - dx) {
//                particle.pos_x = particle.pos_x - dx;
//                particle.vx = -particle.vx * velocityCoef;
//            } else if (particle.pos_x < dxCoef * dx) {
//                particle.pos_x = dxCoef * dx + particle.pos_x;
//                particle.vx = -particle.vx * velocityCoef;
//            }
//        } else {
//            if (particle.pos_x < 0) {
//                particle.pos_x = -particle.pos_x + dx;
//                particle.vx = -particle.vx * velocityCoef;
//            }
//        }
//        if (particle.vy > 0) {
//            if (particle.pos_y > dx * size) {
//                particle.pos_y = 2 * dx * size - particle.pos_y - 2 * dxCoef * dx;
//                particle.vy = -particle.vy * velocityCoef;
//            } else if (particle.pos_y > dx * size - dx) {
//                particle.pos_y = particle.pos_y - dx;
//                particle.vy = -particle.vy * velocityCoef;
//            } else if (particle.pos_y < dxCoef * dx) {
//                particle.pos_y = dxCoef * dx + particle.pos_y;
//                particle.vy = -particle.vy * velocityCoef;
//            }
//        } else {
//            if (particle.pos_y < 0) {
//                particle.pos_y = dxCoef * dx - particle.pos_y;
//                particle.vy = -particle.vy * velocityCoef;
//            }
//        }
//        x = roundValue(1, size - 2, particle.pos_x / dx);
//        y = roundValue(1, size - 2, particle.pos_y / dx);
//        if (y <= 1 && std::abs(particle.vy) <= 1 && _spaceTypes[getIdx(x, 2)] == SpaceType::Empty) {
//            particle.pos_y = 5 * dx;
//            particle.vy = g * dt * 5;
//        }
//    }
//}
//
////Удаляем, если в ячейке > 8 частиц. Добавляем, если в ячейке <4 частиц
////void Solver::deleteUnnecessaryParticles() {
////    std::map<int, std::vector<int>> counts = std::map<int, std::vector<int>>();
////    for (int i = 0; i < particles_size; ++i) {
////        int x = roundValue(1, size - 2, particles[i].pos_x / dx);
////        int y = roundValue(1, size - 2, particles[i].pos_y / dx);
////        int ind = getIdx(x, y);
////        if (counts.count(ind)) {
////            counts[ind].push_back(i);
////        } else {
////            counts[ind] = {i};
////        }
////    }
////    for (auto &count: counts) {
////        int c = count.second.size();
////        if (c > 40) {
////            std::sort(count.second.begin(), count.second.end());
////            for (int i = c - 1; i >= 40; --i) {
////                particles.erase(particles.begin() + count.second[i]);
////                particles_size--;
////            }
////        }
////    }
////}
////
////void Solver::changeParticlesNum() {
////    std::map<int, std::vector<int>> counts = std::map<int, std::vector<int>>();
////    for (int i = 0; i < particles_size; ++i) {
////        int x = roundValue(1, size - 2, particles[i].pos_x / dx);
////        int y = roundValue(1, size - 2, particles[i].pos_y / dx);
////        int ind = getIdx(x, y);
////        if (counts.count(ind)) {
////            counts[ind].push_back(i);
////        } else {
////            counts[ind] = {i};
////        }
////    }
////    for (auto &count: counts) {
////        int c = count.second.size();
////        if (c <= 2) {
////            for (int i = c; i < 5; ++i) {
////                particles.push_back(getMeanParticle(count.second));
////                particles_size++;
////            }
////        }
////    }
////}
////
////Particle Solver::getMeanParticle(vector<int> &particlesIndices) {
////    double pos_x = 0;
////    double pos_y = 0;
////    double vx = 0;
////    double vy = 0;
////    double s = particlesIndices.size();
////    for (auto i: particlesIndices) {
////        Particle &p = particles[i];
////        pos_x += p.pos_x;
////        pos_y += p.pos_y;
////        vx += p.vx;
////        vy += p.vy;
////    }
////    pos_x /= s;
////    pos_y /= s;
////    vx /= s;
////    vy /= s;
////    return Particle(vx, vy, pos_x, pos_y);
////}
