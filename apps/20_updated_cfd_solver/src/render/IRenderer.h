#ifndef TEST_IRENDERER_H
#define TEST_IRENDERER_H

enum class RenderMode {
    Square, Blobbies
};

class IRenderer {
public:
    virtual void saveImage(const std::string &image_name, std::vector<SpaceType> &spaceTypes,
                          std::vector<Particle>, RenderMode mode) = 0;

    virtual ~IRenderer() = default;
};


#endif //TEST_IRENDERER_H
