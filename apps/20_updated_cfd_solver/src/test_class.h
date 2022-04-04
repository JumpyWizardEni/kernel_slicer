#ifndef TEST_TEST_CLASS_H
#define TEST_TEST_CLASS_H

#include <../../TINYSTL/vector>

using std::vector;

enum class SpaceType {
    Fluid, Solid, Empty
};

struct Particle {
    double vx = 0.0;
    double vy = 0.0;
    double pos_x = 0.0; // from 0.0 to 1.0
    double pos_y = 0.0; // from 0.0 to 1.0

    Particle() = default;
    Particle(double vx, double vy, double pos_x, double pos_y): vx(vx), vy(vy), pos_x(pos_x), pos_y(pos_y) {};
};

struct GridPICInfo {
    double sum_vx = 0;
    double sum_vy = 0;
    double weight_vx = 0;
    double weight_vy = 0;
};

class Solver {

public:
    int size = 0; // Количество ячеек по одному направлению
    int particles_size = 0;
    vector<Particle> particles;
    vector<GridPICInfo> gridInfo;
    double dt = 0.033; // Меняется каждый шаг
    double dx = 0.125; // Размер сетки в условных единицах
    double density = 1000;
    const double g = 9.82; // Ускорение свободного падения
    const int PCG_MAX_ITERS = 1000; // Максимальное число итераций для PCG алгоритма
    const double TOL = 1e-9; // epsilon для давления
    double viscosity = 0.0001;
    int particles_pressure_coef = 30;

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

    vector<SpaceType> spaceTypes;

    vector<double> vx; // (size + 1), size
    vector<double> vy; // size, (size + 1)

    vector<double> diff_vx;
    vector<double> diff_vy;

    vector<float> vx2;
    vector<float> vy2;

    vector<int> mask;

    Solver();

    void setParameters();

    virtual void CommitDeviceData() {}                                       // will be overriden in generated class
    virtual void GetExecutionTime(const char* a_funcName, float a_out[4]) {} // will be overriden in generated class

    virtual void performStep(int w, int h, double *input __attribute__((size("w", "h"))), double *output __attribute__((size("w", "h"))));

    //dt <= 5 * dx / max(скорость) на каждом шаге
    void countTimeDelta(const double *vx, const double *vy);


    //TODO test
    double
    interpolate(double q, double *q_copy, double x, double y, int i,
                int j); // перенос некоторой величины q_copy через поле u. Решение уравнения Dq/Dt = 0

    void project();

    void calcNegativeDivergence();


    void fillPressureMatrix();

    void kernel2D_addForces(int h, int w, double *v, double a, SpaceType *_spaceTypes); // добавляются внешние силы (в нашем случае - сила притяжения)

    //TODO test
    void applyPreconditioner();

    //TODO test
    void calcPreconditioner();

    //TODO test
    double getVelocityX(double *vx, int i, int j);

    //TODO test
    double getVelocityY(double *vy, int i, int j);

    //TODO test
    int cutValue(double from, double to, double value);

    double dotProduct(double *first, double *second);

    //TODO test
    void updateVelocities();

    double pow(double value);

    //TODO test
    void applyPressureMatrix();

    // Все формулы записаны в стандартных кооридинатах, однако при индексации ось Y направлена вниз
    int getIdx(int i, int j);

    int getIdxX(int i, int j);

    int getIdxY(int i, int j);

    void resetParams();

    void checkDivergence();

    void dirichleCondition();

    void fillWithZeros(double *v, int size);

    void checkAdvected(double *q, double *prev_q, double *vx, double *vy);

    void assembleGridFromParticles();

    void clearGrid();

    void createSpaceTypes();

    void getVelocitiesFromParticles();

    double kFunc(double x, double y);

    double h2(double x);

    void advectParticles();

    void countDiffXY();

    int roundValue(int i, int i1, double d);

    double getAlpha();

    void kernel2D_calcNegativeDivergence(int h, int w, SpaceType *_spaceTypes, double *_rhs, double *_vx, double *_vy);

    void kernel2D_fillPressureMatrix(int h, int w, SpaceType *_spaceTypes, double *_press_diag, double *_pressX,
                                     double *_pressY);

    void kernel2D_updateVelocities(int h, int w, SpaceType *_spaceTypes, double *_pressure, double *_vx, double *_vy);

    void kernel2D_dirichleCondition(int h, int w, SpaceType *spaceTypes, double *_pressure, double *_vx, double *_vy);

    void kernel1D_clearSpaceTypes(int s, SpaceType *spaceTypes);

    void kernel2D_createSolid(int h, int w, SpaceType *_spaceTypes);

    void kernel2D_meanVelocities(int h, int w, GridPICInfo *_gridInfo, double *_vx, double *_vy);

    void kernel1D_createFluidFromParticles(int s, Particle *_particles, SpaceType *_spaceTypes);

    void kernel1D_particlesToGridVelocity(int s, Particle *_particles, GridPICInfo *_gridInfo);

    void
    kernel2D_countDiffXY(int h, int w, double *_vx, double *_vy, double *_prev_vx, double *_prev_vy, double *_diff_vx,
                         double *_diff_vy);

    int round(double d);
};


#endif //TEST_TEST_CLASS_H
