#ifndef TEST_TEST_CLASS_H
#define TEST_TEST_CLASS_H

#include <vector>

using std::vector;

class Solver {

public:
    struct Particle {
        float vx;
        float vy;
        float pos_x; // from 0.0 to 1.0
        float pos_y; // from 0.0 to 1.0
    };

    struct GridPICInfo {
        float sum_vx;
        float sum_vy;
        float weight_vx;
        float weight_vy;
    };

    const int Solid = 0;
    const int Empty = 1;
    const int Fluid = 2;

    int size = 0; // Количество ячеек по одному направлению
    int particlesSize = 0;
    float dotResult = 0;
    vector<Particle> particles;
    vector<GridPICInfo> gridInfo;
    float dt = 0.015; // Меняется каждый шаг
    float dx = 0.125; // Размер сетки в условных единицах
    float density = 1000;
    const float g = 9.82; // Ускорение свободного падения
    const int PCG_MAX_ITERS = 1000; // Максимальное число итераций для PCG алгоритма
    const float TOL = 1e-9; // epsilon для давления
    float viscosity = 0.0001;
    int particles_pressure_coef = 10;//10
    float maximum_vel = 0.0;

    int overlap = 2;
    int sub_domains = 2;
    int subgrid_size = 0;

    vector<int> isEnd;

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

    vector<int> spaceTypes;
    vector<int> counts;

    vector<float> vx; // (size + 1), size
    vector<float> vy; // size, (size + 1)

    vector<float> diff_vx;
    vector<float> diff_vy;

    vector<int> solid_indices;

    Solver();

    void setParameters(int grid_num, float _dx, vector<int> &_solid_indices, int pSize);

    virtual void CommitDeviceData() {}                                       // will be overriden in generated class
    virtual void GetExecutionTime(const char* a_funcName, float a_out[4]) {} // will be overriden in generated class

    virtual void performStep(int pSize, const Particle *input __attribute__((size("pSize"))), Particle *output __attribute__((size("pSize"))));

    //dt <= 5 * dx / max(скорость) на каждом шаге
    void kernel1D_countTimeDelta(int _size, float *p_vx, float *p_vy);


    float
    interpolate(float _q, float *q_copy, float x, float y, int i, int j, float _dx, int _size); // перенос некоторой величины q_copy через поле u. Решение уравнения Dq/Dt = 0

    void kernel2D_addForces(int h, int w, float *v, float a, int *_spaceTypes); // добавляются внешние силы (в нашем случае - сила притяжения)

    void kernel1D_applyPreconditionerBackward(int _size);

    void kernel1D_calcPreconditioner(int _size);

    float getVelocityX(float *vx, int i, int j);

    float getVelocityY(float *vy, int i, int j);

    void kernel1D_dotProduct(int size, float *first, float *second);

    float pow(float value);

    void kernel2D_applyPressureMatrix(int h, int w);

    // Все формулы записаны в стандартных кооридинатах, однако при индексации ось Y направлена вниз
    int getIdx(int i, int j);

    int getIdxX(int i, int j);

    int getIdxY(int i, int j);

    void checkDivergence();

    void kernel1D_fillWithZeros_float(int _size, float *v);

    float kFunc(float x, float y);

    float h2(float x);



    int roundValue(int i, int i1, float d);

    float getAlpha();

    void
    kernel1D_advectParticles(int _size, float *_diff_vx, float *_diff_vy, float *_vx, float *_vy,
                             int *_spaceTypes);

    void kernel2D_calcNegativeDivergence(int h, int w, int *_spaceTypes, float *_rhs, float *_vx, float *_vy);

    void kernel2D_fillPressureMatrix(int h, int w, int *_spaceTypes, float *_press_diag, float *_pressX,
                                     float *_pressY);

    void kernel2D_updateVelocities(int h, int w, int *_spaceTypes, float *_pressure, float *_vx, float *_vy);

    void kernel2D_dirichleCondition(int h, int w, int *_spaceTypes, float *_pressure, float *_vx, float *_vy);

    void kernel1D_clearSpaceTypes(int _size, int *_spaceTypes);

    void kernel2D_createSolid(int h, int w, int *_spaceTypes);

    void kernel2D_meanVelocities(int h, int w, float *_vx, float *_vy);

    void kernel1D_createFluidFromParticles(int _size);

    void kernel1D_particlesToGridVelocity(int _size);

    void
    kernel2D_countDiffXY(int h, int w, float *_vx, float *_vy, float *_prev_vx, float *_prev_vy, float *_diff_vx,
                         float *_diff_vy);

    void kernel1D_createAdditionalSolid(int _size, int *indices, int *_spaceTypes);

    void kernel1D_fillWithZeros(int _size, int *v);

    void kernel1D_countParticlesNum(int _size);

    void kernel2D_changePressureWithParticles(int h, int w, int *spaceTypes, int *counts, float *pressure);

    void kernel1D_applyPreconditionerForward(int _size);

    void kernel1D_changeParticlesSize(int _size);

    void kernel1D_changeSearchVector(int _size, float beta);

    void kernel1D_checkZeroRhs(int _size);

    void kernel1D_changePressure(int _size, float alpha);
};


#endif //TEST_TEST_CLASS_H
