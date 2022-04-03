#ifndef TEST_VULKANRENDER_H
#define TEST_VULKANRENDER_H


#include <string>
#include <vector>
#include "IRenderer.h"
#include "SimpleRenderer.h"

class VulkanRender: IRenderer {
    void saveImage(const std::string &image_name, std::vector<SpaceType> &spaceTypes,
                   std::vector<Particle>, RenderMode mode) override;

    
};


#endif //TEST_VULKANRENDER_H
