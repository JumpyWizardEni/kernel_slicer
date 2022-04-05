#ifndef TEST_IRENDERER_H
#define TEST_IRENDERER_H

#include <vector>
#include "../test_class.h"


enum class RenderMode {
    Square, Blobbies
};

class IRenderer {
public:
    virtual void saveImage(const std::string &image_name, std::vector<int> &spaceTypes,
                           std::vector<Particle>, RenderMode mode) = 0;

    virtual ~IRenderer() = default;
};


#endif //TEST_IRENDERER_H
