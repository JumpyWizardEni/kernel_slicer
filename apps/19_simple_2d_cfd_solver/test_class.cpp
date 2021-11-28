#include "test_class.h"
#include <vector>
#include <cstring>

using std::vector;

void Solver::set_bounds(int N, int b, float *x) {
    int size_ = N + 2;
    for (int i = 1; i < N; i++) {
        x[i * size_] = b == 1 ? -x[1 + i * size_] : x[1 + i * size_];
        x[N - 1 + i * size_] = b == 1 ? -x[N - 2 + i * size_] : x[N - 2 + i * size_];
        x[i] = b == 2 ? -x[i + size_] : x[i + size_];
        x[i + (N - 1) * size_] = b == 2 ? -x[i + (N - 2) * size_] : x[i + (N - 2) * size_];
    }
    x[0] = 0.5 * (x[1] + x[size_]);
    x[(N - 1) * size_] = 0.5 * (x[1 + (N - 1) * size_] + x[(N - 2) * size_]);
    x[N - 1] = 0.5 * (x[N - 2] + x[N - 1 + size_]);
    x[N - 1 + (N - 1) * size_] = 0.5 * (x[N - 2 + (N - 1) * size_] + x[N - 1 + (N - 2) * size_]);
}

template<class T>
void swap(T &a, T &b) {
    T c;
    move(a, c);
    move(b, a);
    move(c, b);
}

void Solver::perform(float *out_density) {
    //velocity_step
    kernel1D_AddSources(size * size, dt, vx.data(), vx0.data());
    kernel1D_AddSources(size * size, dt, vy.data(), vy0.data());

    swap(vx0, vx);
    kernel1D_Diffuse(1, vx.data(), vx0.data(), visc);
    swap(vy0, vy);
    kernel1D_Diffuse(2, vy.data(), vy0.data(), visc);

    kernel1D_Project_1(size - 2, vx.data(), vy.data(), vx0.data(), vy0.data());
    kernel1D_Project_2(vx.data(), vy.data(), vx0.data(), vy0.data());
    kernel1D_Project_3(size-2, vx.data(), vy.data(), vx0.data(), vy0.data());


    swap(vx0, vx);
    swap(vy0, vy);

    kernel1D_Advect(size - 2, dt, 1, vx.data(), vx0.data(), vx0.data(), vy0.data());
    kernel1D_Advect(size - 2, dt, 2, vy.data(), vy0.data(), vx0.data(), vy0.data());

    kernel1D_Project_1(size - 2, vx.data(), vy.data(), vx0.data(), vy0.data());
    kernel1D_Project_2(vx.data(), vy.data(), vx0.data(), vy0.data());
    kernel1D_Project_3(size - 2, vx.data(), vy.data(), vx0.data(), vy0.data());

    //density step
    kernel1D_AddSources(size * size, dt, density.data(), density_prev.data());
    swap(density_prev, density);
    kernel1D_Diffuse(0, density.data(), density_prev.data(), diff);
    swap(density_prev, density);
    kernel1D_Advect(size - 2, dt, 0, density.data(), density_prev.data(), vx.data(), vy.data());

    memcpy(out_density, density.data(), density.size() * sizeof(float));
}

void Solver::kernel1D_AddSources(int N, float dt_, float *v, const float *v0) {
    for (int i = 0; i < N; i++) {
        v[i] += dt_ * v0[i];
    }
}

void Solver::kernel1D_Diffuse(int b, float *x, const float *x0, float diffuse) {

    for (int k = 0; k < STEPS_NUM; k++) {
        float a = dt * diffuse * (float) (size - 2) * (float) (size - 2);
        for (int i = 1; i <= size - 2; i++) {
            for (int j = 1; j <= size - 2; j++) {

                x[i + j * size] = (x0[i + j * size] + a * (x[i - 1 + j * size] + x[i + 1 + j * size] +
                                                             x[i + (j - 1) * size] + x[i + (j + 1) * size])) /
                                   (1 + 4 * a);
            }
        }
        set_bounds((int)size - 2, b, x);
    }
}

void Solver::kernel1D_Advect(int N, float dt_, int b, float *d, const float *d0, const float *u, const float *v) {

    for (int i = 1; i <= N; i++) {
        for (int j = 1; j <= N; j++) {
            int i0, j0, i1, j1;
            float x, y, s0, t0, s1, t1, dt0;
            dt0 = dt_ * (float) (N);
            x = i - dt0 * u[i + j * size];
            y = j - dt0 * v[i + j * size];
            if (x < 0.5) x = 0.5;
            if (x > N + 0.5) x = (float)(N + 0.5);
            i0 = (int) x;
            i1 = i0 + 1;
            if (y < 0.5) y = 0.5;
            if (y > N + 0.5) y = (float)(N + 0.5);
            j0 = (int) y;
            j1 = j0 + 1;
            s1 = x - i0;
            s0 = 1 - s1;
            t1 = y - j0;
            t0 = 1 - t1;
            d[i + j * size] = s0 * (t0 * d0[i0 + j0 * size] + t1 * d0[i0 + j1 * size]) +
                               s1 * (t0 * d0[i1 + j0 * size] + t1 * d0[i1 + j1 * size]);
        }
    }
    set_bounds(size - 2, b, d);
}

void Solver::kernel1D_Project_1(int N, float *u, float *v, float *p, float *div) {
    for (int i = 1; i <= N; i++) {
        float h = 1.0f / (float) (size - 2);

        for (int j = 1; j <= size - 2; j++) {
            div[i + j * size] = -0.5 * h * (u[i + 1 + j * size] - u[i - 1 + j * size] +
                                             v[i + (j + 1) * size] - v[i + (j - 1) * size]);
            p[i + j * size] = 0;
        }
    }
    set_bounds(N, 0, div);
    set_bounds(N, 0, p);
}

void Solver::kernel1D_Project_2(float *u, float *v, float *p, float *div) {

    for (int k = 0; k < STEPS_NUM; k++) {
        for (int i = 1; i <= size - 2; i++) {
            for (int j = 1; j <= size - 2; j++) {
                p[i + j * size] = (div[i + j * size] + p[i - 1 + j * size] + p[i + 1 + j * size] +
                                    p[i + (j - 1) * size] + p[i + (j + 1) * size]) / 4;
            }
        }
        set_bounds(size - 2, 0, p);
    }
}

void Solver::kernel1D_Project_3(int N, float *u, float *v, float *p, float *div) {

    for (int i = 1; i <= N; i++) {
        float h = 1.0f / (float) N;
        for (int j = 1; j <= size - 2; j++) {
            u[i + j * size] -= 0.5 * (p[i + 1 + j * size] - p[i - 1 + j * size]) / h;
            v[i + j * size] -= 0.5 * (p[i + (j + 1) * size] - p[i + (j - 1) * size]) / h;
        }
    }
    set_bounds(N, 1, u);
    set_bounds(N, 2, v);
}

void Solver::setParameters(int size, vector<float> &density, vector<float> &vx, vector<float> &vy, float dt, float visc,
                           float diff) {
    this->size = size;
    this->density = density;
    this->density_prev = this->density;
    this->vx = vx;
    this->vx0 = this->vx;
    this->vy = vy;
    this->vy0 = this->vy;
    this->dt = dt;
    this->visc = visc;
    this->diff = diff;
}

Solver::Solver() = default;

