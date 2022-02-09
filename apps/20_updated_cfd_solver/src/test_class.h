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
    float dx = 0.015; // Размер сетки в условных единицах
    float density = 1000;
    const float g = 9.82f; // Ускорение свободного падения
    const int PCG_MAX_ITERS = 10; // Максимальное число итераций для PCG алгоритма
    const float TOL = 1e-5; // epsilon для давления

    //Давление. Решаем уравнение PRESS * pressure = rhs. PRESS - симметричная матрица.
    vector<float> pressure;
    vector<float> pressureResult;
    vector<float> rhs;
    vector<float> press_diag;
    vector<float> pressX;
    vector<float> pressY;
    vector<float> preconditioner; // (size + 1) X (size + 1)
    vector<float> q;
    vector<float> z;
    vector<float> s;
    vector<float> sygma;

    vector<float> velocityExtra;




    vector<SpaceType> spaceTypes;
    vector<SpaceType> spaceTypesOld;

    vector<float> vx; // size + 1, size
    vector<float> vy; // size, size + 1

    vector<float> extraVec;
    vector<float> transferVec;



    Solver();

    void
    setParameters(int size, const vector<float> &density, const vector<float> &vx, const vector<float> &vy, float dt,
                  float visc, float diff);


    void performStep();

    //dt <= 5 * dx / max(скорость) на каждом шаге
    void countTimeDelta(const float *vx, const float *vy);

    void addForces(float *v); // добавляются внешние силы (в нашем случае - сила притяжения)

    void
    advect(float *vx, float *vy, float *q); // перенос некоторой величины q через поле u. Решение уравнения Dq/Dt = 0

    void project();

    void calcNegativeDivergence();


    void fillPressureMatrix();

    void PCG();

    void applyPreconditioner();

    void calcPreconditioner();

    float getVelocityX(float i, float j);

    float getVelocityY(float i, float j);

    void moveCells(SpaceType *old_s, SpaceType *new_s);

    int cutValue(float from, float to, float value);

    void dotProduct(float *first, float *second, float *result);

    void multPressMatrix();

    void dotDiv(float *first, float *second, float *result);

    void updateVelocities();
};


#endif //TEST_TEST_CLASS_H
