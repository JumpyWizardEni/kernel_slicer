#ifndef TEST_SOLVER_TEST_H
#define TEST_SOLVER_TEST_H

#include <cxxtest/TestSuite.h>
#include "../src/test_class.h"
#include <vector>

using std::vector;

float EPS = 10e-5;

#define TS_ASSERT_VECTOR_EQUALS(vec, answer) \
    for (int i = 0; i < vec.size(); i++) {   \
        TS_ASSERT_DELTA(vec[i], answer[i], EPS)\
    }

//По краям стены, есть пустые клетки, есть клетки воды, окруженные только водой
// S S S S S S
// S F F F F S
// S F F F F S
// S E F E F S
// S F E F F S
// S S S S S S
void basicSolidSetup(Solver &solver) {
    int size = 6;
    int dx = 1;
    int dt = 1;
    vector<SpaceType> spaceTypes(size * size);
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (i == 0 || i == size - 1 || j == 0 || j == size - 1) {
                spaceTypes[i + size * j] = SpaceType::Solid;
            } else if ((j == 3 && (i == 1 || i == 3)) || (j == 4 && i == 2)) {
                spaceTypes[i + size * j] = SpaceType::Empty;
            } else {
                spaceTypes[i + size * j] = SpaceType::Fluid;
            }
        }
    }

    solver.vx = {
            1, 2, 3, 4, 5, 6, 7,
            1, 2, 3, 4, 5, 6, 7,
            1, 2, 3, 4, 5, 6, 7,
            1, 2, 3, 4, 5, 6, 7,
            1, 2, 3, 4, 5, 6, 7,
            1, 2, 3, 4, 5, 6, 7
    };

    solver.vy = {
            1, 1, 1, 1, 1, 1,
            2, 2, 2, 2, 2, 2,
            3, 3, 3, 3, 3, 3,
            4, 4, 4, 4, 4, 4,
            5, 5, 5, 5, 5, 5,
            6, 6, 6, 6, 6, 6,
            7, 7, 7, 7, 7, 7
    };

    solver.spaceTypes = spaceTypes;
    solver.size = size;
    solver.dx = dx;
    solver.dt = dt;
    solver.setParameters();
}


class SolverTest : public CxxTest::TestSuite {
public:
    void testCountDelta() {
        Solver solver = Solver();
        solver.vx = {1, 2, 3,
                     4, 5, 6};
        solver.vy = {9, 8,
                     6, 5,
                     3, 2};
        solver.dx = 1;
        solver.size = 2;
        float gt_dt = 0.311283475;

        solver.countTimeDelta(solver.vx.data(), solver.vy.data());

        TS_ASSERT_DELTA(solver.dt, gt_dt, EPS);
    }

    void testAddForces() {
        Solver solver = Solver();
        solver.vy = {9, 8,
                     6, 5,
                     3, 2};
        solver.dt = 0.5;
        solver.size = 2;
        vector<float> gt = {
                4.09, 3.09,
                1.09, 0.09,
                -1.91, -2.91};

        solver.addForces(solver.vy.data());

        TS_ASSERT_VECTOR_EQUALS(solver.vy, gt)
    }

    //TODO advect test

    void testCalcNegativeDivergence() {
        Solver solver = Solver();
        basicSolidSetup(solver);
        vector<float> gt = {
                0, 0, 0, 0, 0, 0,
                0, 0, 2, 2, 8, 0,
                0, -2, 0, 0, 6, 0,
                0, 0, 0, 0, 6, 0,
                0, -8, 0, -6, 0, 0,
                0, 0, 0, 0, 0, 0,

        };

        solver.calcNegativeDivergence();

        TS_ASSERT_VECTOR_EQUALS(solver.rhs, gt)
    }

    void testPressureMatrix() {
        Solver solver = Solver();
        basicSolidSetup(solver);
        solver.density = 0.5;
        vector<float> gt_press_diag = {
                0, 0, 0, 0, 0, 0,
                0, 4, 6, 6, 4, 0,
                0, 6, 8, 8, 6, 0,
                0, 0, 8, 0, 6, 0,
                0, 4, 0, 6, 4, 0,
                0, 0, 0, 0, 0, 0,
        };
        vector<float> gt_press_x = {
                0,  0,  0,  0, 0, 0,
                0, -2, -2, -2, 0, 0,
                0, -2, -2, -2, 0, 0,
                0,  0,  0,  0, 0, 0,
                0,  0,  0, -2, 0, 0,
                0,  0,  0,  0, 0, 0,
        };
        vector<float> gt_press_y = {
                0,  0,  0,  0,  0, 0,
                0,  0,  0,  0,  0, 0,
                0, -2, -2, -2, -2, 0,
                0,  0, -2,  0, -2, 0,
                0,  0,  0,  0, -2, 0,
                0,  0,  0,  0,  0, 0,
        };

        solver.fillPressureMatrix();

        TS_ASSERT_VECTOR_EQUALS(solver.press_diag, gt_press_diag)
        TS_ASSERT_VECTOR_EQUALS(solver.pressX, gt_press_x)
        TS_ASSERT_VECTOR_EQUALS(solver.pressY, gt_press_y)
    }

    void testDotProduct() {
        Solver solver = Solver();
        solver.size = 3;
        vector<float> first = {
                1, 1, 1,
                2, 2, 2,
                1, 1, 1
        };
        vector<float> second = {
                0, 0, 0,
                3.14, -2.3, 1,
                1, 5, 1
        };
        float gt = 10.68;

        float res = solver.dotProduct(first.data(), second.data());

        TS_ASSERT_DELTA(res, gt, EPS)

    }

    void testGetIndex() {
        Solver solver = Solver();
        solver.size = 5;

        TS_ASSERT_EQUALS(solver.getIdx(1, 1), 16)
        TS_ASSERT_EQUALS(solver.getIdxX(1, 2), 13)
        TS_ASSERT_EQUALS(solver.getIdxY(1, 3), 11)
    }

    void testCalcPreconditioner() {

    }

    void project() {
        Solver solver = Solver();
        int size = 4;
        vector<SpaceType> spaceTypes(size * size);
        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                if (i == 0 || i == size - 1 || j == 0 || j == size - 1) {
                    spaceTypes[i + size * j] = SpaceType::Solid;
                } else {
                    spaceTypes[i + size * j] = SpaceType::Fluid;
                }
            }
        }
//
//        solver.vx = {
//                1, 2, 3, 4, 5, 6, 7,
//                1, 2, 3, 4, 5, 6, 7,
//                1, 2, 3, 4, 5, 6, 7,
//                1, 2, 3, 4, 5, 6, 7,
//                1, 2, 3, 4, 5, 6, 7,
//                1, 2, 3, 4, 5, 6, 7
//        };
//
//        solver.vy = {
//                1, 1, 1, 1, 1, 1,
//                2, 2, 2, 2, 2, 2,
//                3, 3, 3, 3, 3, 3,
//                4, 4, 4, 4, 4, 4,
//                5, 5, 5, 5, 5, 5,
//                6, 6, 6, 6, 6, 6,
//                7, 7, 7, 7, 7, 7
//        };

        solver.spaceTypes = spaceTypes;
        solver.size = size;
        solver.dx = 1;
        solver.dt = 1;
        solver.density = 1;
        solver.setParameters();
        solver.project();
    }

};

#endif //TEST_SOLVER_TEST_H
