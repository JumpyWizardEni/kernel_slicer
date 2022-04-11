#ifndef TEST_TEST_CLASS_H
#define TEST_TEST_CLASS_H

#include <vector>

using std::vector;

class Solver {

public:
    struct Particle {
        double vx;
        double vy;
        double pos_x; // from 0.0 to 1.0
        double pos_y; // from 0.0 to 1.0
    };

    struct GridPICInfo {
        double sum_vx;
        double sum_vy;
        double weight_vx;
        double weight_vy;
    };

    const int Solid = 0;
    const int Empty = 1;
    const int Fluid = 2;

    int size = 0; // Количество ячеек по одному направлению
    int particlesSize = 0;
    double dotResult = 0;
    vector<Particle> particles;
    vector<GridPICInfo> gridInfo;
    double dt = 0.033; // Меняется каждый шаг
    double dx = 0.125; // Размер сетки в условных единицах
    double density = 1000;
    const double g = 9.82; // Ускорение свободного падения
    const int PCG_MAX_ITERS = 1000; // Максимальное число итераций для PCG алгоритма
    const double TOL = 1e-9; // epsilon для давления
    double viscosity = 0.0001;
    int particles_pressure_coef = 0;//10
    double maximum_vel = 0.0;

    int overlap = 2;
    int sub_domains = 5;
    int subgrid_size = size / sub_domains;

    //Давление. Решаем уравнение PRESS * pressure = rhs. PRESS - симметричная матрица.
    vector<double> pressure;
    vector<double> pressureResidual;
    vector<double> rhs;
    vector<double> press_diag;
    vector<double> pressX;
    vector<double> pressY;
    vector<double> preconditioner; // (size + 1) X (size + 1)
    vector<double> q;
    vector<double> z;
    vector<double> s;

    vector<double> prev_vx;
    vector<double> prev_vy;

    vector<int> spaceTypes;
    vector<int> counts;

    vector<double> vx; // (size + 1), size
    vector<double> vy; // size, (size + 1)

    vector<double> diff_vx;
    vector<double> diff_vy;

    vector<int> solid_indices;

    Solver();

    void setParameters();

    virtual void CommitDeviceData() {}                                       // will be overriden in generated class
    virtual void GetExecutionTime(const char* a_funcName, float a_out[4]) {} // will be overriden in generated class

    virtual void performStep(int w, int h, double *input __attribute__((size("w", "h"))), double *output __attribute__((size("w", "h"))));

    //dt <= 5 * dx / max(скорость) на каждом шаге
    void kernel1D_countTimeDelta(int size, const double *p_vx, const double *p_vy);


    double
    interpolate(double q, double *q_copy, double x, double y, int i, int j, double dx, int size); // перенос некоторой величины q_copy через поле u. Решение уравнения Dq/Dt = 0

    void kernel2D_addForces(int h, int w, double *v, double a, int *_spaceTypes); // добавляются внешние силы (в нашем случае - сила притяжения)

    void applyPreconditioner();

    void calcPreconditioner();

    double getVelocityX(double *vx, int i, int j);

    double getVelocityY(double *vy, int i, int j);

    void kernel1D_dotProduct(int size, double *first, double *second, double *result);

    double pow(double value);

    void kernel2D_applyPressureMatrix(int h, int w, int *spaceTypes, double *s, double *pressX, double *pressY, double *z,
                                      double *press_diag);

    // Все формулы записаны в стандартных кооридинатах, однако при индексации ось Y направлена вниз
    int getIdx(int i, int j);

    int getIdxX(int i, int j);

    int getIdxY(int i, int j);

    void resetParams();

    void checkDivergence();

    void fillWithZeros(double *v, int size);

    double kFunc(double x, double y);

    double h2(double x);



    int roundValue(int i, int i1, double d);

    double getAlpha();

    void
    kernel1D_advectParticles(int particles_size, Particle *_particles, double *_diff_vx, double *_diff_vy,
                             double *_vx, double *_vy, int *_spaceTypes);

    void kernel2D_calcNegativeDivergence(int h, int w, int *_spaceTypes, double *_rhs, double *_vx, double *_vy);

    void kernel2D_fillPressureMatrix(int h, int w, int *_spaceTypes, double *_press_diag, double *_pressX,
                                     double *_pressY);

    void kernel2D_updateVelocities(int h, int w, int *_spaceTypes, double *_pressure, double *_vx, double *_vy);

    void kernel2D_dirichleCondition(int h, int w, int *spaceTypes, double *_pressure, double *_vx, double *_vy);

    void kernel1D_clearSpaceTypes(int s, int *spaceTypes);

    void kernel2D_createSolid(int h, int w, int *_spaceTypes);

    void kernel2D_meanVelocities(int h, int w, double *_vx, double *_vy);

    void kernel1D_createFluidFromParticles(int _size);

    void kernel1D_particlesToGridVelocity(int _size);

    void
    kernel2D_countDiffXY(int h, int w, double *_vx, double *_vy, double *_prev_vx, double *_prev_vy, double *_diff_vx,
                         double *_diff_vy);

    void kernel1D_createAdditionalSolid(int size, int *indices, int *_spaceTypes);

    void fillWithZeros(int *v, int size);

    void kernel1D_countParticlesNum(int size);

    void kernel2D_changePressureWithParticles(int h, int w, int *spaceTypes, int *counts, double *pressure);

    void kernel1D_applyPreconditionerForward(int _size);
};


#endif //TEST_TEST_CLASS_H
