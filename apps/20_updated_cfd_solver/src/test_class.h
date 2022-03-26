#ifndef TEST_TEST_CLASS_H
#define TEST_TEST_CLASS_H

#include <vector>

using std::vector;

enum class SpaceType {
    Fluid, Solid, Empty
};

class Velocity {

};

struct Particle {
    float vx = 0.0;
    float vy = 0.0;
    float pos_x = 0.0; // from 0.0 to 1.0
    float pos_y = 0.0; // from 0.0 to 1.0

    Particle() = default;
    Particle(float vx, float vy, float pos_x, float pos_y): vx(vx), vy(vy), pos_x(pos_x), pos_y(pos_y) {};
};

struct GridPICInfo {
    float sum_vx;
    float sum_vy;
    float weight_vx;
    float weight_vy;
};

class Solver {

public:
    int size = 0; // Количество ячеек по одному направлению
    int particles_size = 0;
    vector<Particle> particles;
    vector<GridPICInfo> gridInfo;
    float dt = 0.01; // Меняется каждый шаг
    float dx = 0.125; // Размер сетки в условных единицах
    float density = 1000;
    const float g = 9.82f; // Ускорение свободного падения
    const int PCG_MAX_ITERS = 100000; // Максимальное число итераций для PCG алгоритма
    const float TOL = 1e-9; // epsilon для давления

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

    vector<float> prev_vx;
    vector<float> prev_vy;

    vector<SpaceType> spaceTypes;

    vector<float> vx; // (size + 1), size
    vector<float> vy; // size, (size + 1)

    vector<float> diff_vx;
    vector<float> diff_vy;

    vector<int> mask;
    vector<int> wavefront;

    Solver();

    void setParameters();

    void performStep();

    //dt <= 5 * dx / max(скорость) на каждом шаге
    void countTimeDelta(const float *vx, const float *vy);

    void addForces(float *v, float a); // добавляются внешние силы (в нашем случае - сила притяжения)

    //TODO test
    float
    interpolate(float q, float *q_copy, float x, float y, int i,
                int j); // перенос некоторой величины q_copy через поле u. Решение уравнения Dq/Dt = 0

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
    float getVelocityX(float *vx, int i, int j);

    //TODO test
    float getVelocityY(float *vy, int i, int j);

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

    bool isFluidVelocityX(int i, int j);

    void resetParams();

    void checkDivergence();

    void dirichleCondition();

    bool isFluidVelocityY(int i, int j);

    void checkNonFluidVelocities();

    void visualise();

    void visualiseVx();

    void visualiseSpaceTypes();

    void visualiseVy();

    void visualisePressure();

    void fillWithZeros(float *v, int size);

    void checkAdvected(float *q, float *prev_q, float *vx, float *vy);

    void assembleGridFromParticles();

    void clearGrid();

    void createSpaceTypes();

    void getVelocitiesFromParticles();

    float kernelFunc(float x, float y);

    float h2(float x);

    void advectParticles();

    void countDiffXY();

    double randfrom1(double min, double max);

    void changeParticlesNum();

    Particle getMeanParticle(vector<int> &particlesIndices);

    void extrapolateVelocities();
};


#endif //TEST_TEST_CLASS_H
