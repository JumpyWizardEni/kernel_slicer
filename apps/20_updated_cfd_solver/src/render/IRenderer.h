#ifndef TEST_IRENDERER_H
#define TEST_IRENDERER_H

#include <vector>
#include "../test_class.h"


enum class RenderMode {
    Square, Blobbies
};

class IRenderer {
protected:
    const int Solid = 0;
    const int Empty = 1;
    const int Fluid = 2;
public:
    virtual void
    saveImage(const std::string &image_name, std::vector<int> &spaceTypes, std::vector<Solver::Particle> &particles,
              RenderMode mode) = 0;

    virtual ~IRenderer() = default;
};


#endif //TEST_IRENDERER_H
