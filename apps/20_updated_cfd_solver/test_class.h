#ifndef TEST_TEST_CLASS_H
#define TEST_TEST_CLASS_H

#include <vector>

using std::vector;

enum class SpaceType {
    Fluid, Solid, Empty
};

class Solver {

public:
    int size = 0; // Количество ячеек по одному направлению
    float dt = 0.1; // Меняется каждый шаг
    float dx = 6; // Размер сетки в условных единицах
    const float g = 9.8f; // Ускорение свободного падения
    const int PCG_MAX_ITERS = 10; // Максимальное число итераций для PCG алгоритма
    const float TOL = 1e-5; //
//    float visc = 1;
//    float diff = 0.5;

    static constexpr int STEPS_NUM = 20;

    float density;
    vector<float> pressure;
    vector<float> pressure_result;
    vector<float> density_prev;
    vector<float> press_diag;
    vector<float> press_x;
    vector<float> press_y;
    vector<float> rhs;

    vector<SpaceType> spaceTypes;
    vector<SpaceType> spaceTypesOld;

    vector<float> vx; // size + 1, size
    vector<float> extra_vec;
    vector<float> transfer_vec;

    vector<float> vy; // size, size + 1

    vector<float> vx0; // содержит горизонтальную компоненту скорости на предыдущем шаге
    vector<float> vy0; // содержит вертикальную компоненту скорости на предыдущем шаге

    Solver();

    void
    setParameters(int size, const vector<float> &density, const vector<float> &vx, const vector<float> &vy, float dt,
                  float visc, float diff);


    void performStep();

    //dt <= 5 * dx / max(скорость) на каждом шаге
    void countTimeDelta(const float *vx, const float *vy);

    void add_forces(float *v); // добавляются внешние силы (в нашем случае - сила притяжения)

    void advect(float *vx, float *vy, float *q); // перенос некоторой величины q через поле u. Решение уравнения Dq/Dt = 0

//    void project();

    void calcNegativeDivergence();
//
//    void fillPressureMatrix();
//
//    void PCG();
//
//    void applyPreconditioner();
//
//    void calcPreconditioner();

      float getVelocityX(float i, float j);

      float getVelocityY(float i, float j);

      void moveCells(SpaceType *old_s, SpaceType *new_s);

      float cutValue(float from, float to, float value);
};


#endif //TEST_TEST_CLASS_H
