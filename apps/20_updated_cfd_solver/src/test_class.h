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
    float dx = 0.125; // Размер сетки в условных единицах
    float density = 1000;
    const float g = -9.82f; // Ускорение свободного падения
    const int PCG_MAX_ITERS = 1000; // Максимальное число итераций для PCG алгоритма
    const float TOL = 1e-5; // epsilon для давления

    //Давление. Решаем уравнение PRESS * pressure = rhs. PRESS - симметричная матрица.
    vector<float> pressure;
    vector<float> pressureResidual;
    vector<float> rhs;
    vector<float> press_diag;
    vector<float> pressX;
    vector<float> pressY;
    vector<float> preconditioner; // (size + 1) X (size + 1)
    vector<float> q;
    vector<float> z;
    vector<float> s;

    vector<float> velocityExtra;
    vector<float> velocityExtra2;

    vector<SpaceType> spaceTypes;
    vector<SpaceType> spaceTypesOld;

    vector<float> vx; // (size + 1), size
    vector<float> vy; // size, (size + 1)

    vector<float> extraVec;
    vector<float> transferVec;



    Solver();

    void setParameters();

    void performStep();

    //dt <= 5 * dx / max(скорость) на каждом шаге
    void countTimeDelta(const float *vx, const float *vy);

    void addForces(float *v); // добавляются внешние силы (в нашем случае - сила притяжения)

    //TODO test
    void
    advect(float *vx, float *vy, float *q_copy, float *q); // перенос некоторой величины q_copy через поле u. Решение уравнения Dq/Dt = 0

    void project();

    void calcNegativeDivergence();


    void fillPressureMatrix();

    //TODO test
    void PCG();

    //TODO test
    void applyPreconditioner();

    //TODO test
    void calcPreconditioner();

    //TODO test
    float getVelocityX(int i, int j);

    //TODO test
    float getVelocityY(int i, int j);

    //TODO test
    void moveCells(SpaceType *old_s, SpaceType *new_s);

    //TODO test
    int cutValue(float from, float to, float value);

    float dotProduct(float *first, float *second);

    //TODO test
    void updateVelocities();

    //TODO test
    void applyPressureMatrix();

    // Все формулы записаны в стандартных кооридинатах, однако при индексации ось Y направлена вниз
    int getIdx(int i, int j);

    int getIdxX(int i, int j);

    int getIdxY(int i, int j);
};


#endif //TEST_TEST_CLASS_H
