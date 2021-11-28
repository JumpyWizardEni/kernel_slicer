#include "stb_image_write.h"
#include <vector>
#include <iostream>
#include <memory>
#include <chrono>



#include "vk_utils.h"
#include "vk_pipeline.h"
#include "vk_copy.h"
#include "vk_buffers.h"
#include "test_class_gpu.h"
#include "test_class.h"
#include "test_class_generated.h"
const int N = 50;

std::vector<float> solve_cpu() {
    std::vector<float> out(N * N);
    std::vector<float> density;
    density.resize(N * N);
    for (int i = 0; i < N * N; ++i) {
        density[i] = randfrom(0, 1);
    }
    std::vector<float> vx;
    vx.resize(N * N);
    for (int i = 0; i < N * N; ++i) {
        vx[i] = randfrom(-5, 5);
    }
    std::vector<float> vy;
    vy.resize(N * N);
    for (int i = 0; i < N * N; ++i) {
        vy[i] = randfrom(-5, 5);
    }
    Solver s = Solver();
    s.setParameters(N, density, vx, vy, 0.033, 0.001, 0.00001);
    for (int i = 0; i < 50; ++ i) {
        s.perform(out.data());
        save_image("images/" + std::to_string(i) + ".jpeg", out);
    }
    return out;
}


int main() {
    auto x = solve_gpu();
//    solve_cpu();
    return 0;
}