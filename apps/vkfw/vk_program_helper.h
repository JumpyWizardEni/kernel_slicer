#ifndef VULKAN_PROGRAM_HELPER_H
#define VULKAN_PROGRAM_HELPER_H

#include "volk.h"
#include "vk_utils.h"

#include <vector>
#include <map>
#include <unordered_map>

namespace vkfw
{
  struct PBinding
  {
    bool operator==(const PBinding& rhs) const
    {
      bool buffViewMatch = ((buffView != nullptr) && (rhs.buffView != nullptr)) ||
                           ((buffView == nullptr) && (rhs.buffView == nullptr));

      bool buffsMatch = ((buffer != nullptr) && (rhs.buffer != nullptr)) ||
                        ((buffer == nullptr) && (rhs.buffer == nullptr));

      bool buffsMatch2 = (bufferOffset == rhs.bufferOffset);

      bool imgMatch = ((imageView != nullptr) && (rhs.imageView != nullptr)) ||
                      ((imageView == nullptr) && (rhs.imageView == nullptr));

      bool samMatch = ((imageSampler != nullptr) && (rhs.imageSampler != nullptr)) ||
                      ((imageSampler == nullptr) && (rhs.imageSampler == nullptr));

      bool asMatch = ((accelStruct != nullptr) && (rhs.accelStruct != nullptr)) ||
                      ((accelStruct == nullptr) && (rhs.accelStruct == nullptr));

      return buffViewMatch && buffsMatch && buffsMatch2 && imgMatch && samMatch && asMatch && (type == rhs.type);
    }

    VkBufferView buffView     = nullptr;
    VkBuffer     buffer       = nullptr;
    size_t       bufferOffset = 0;
    VkImageView  imageView    = nullptr;
    VkSampler    imageSampler = nullptr;
    VkImageLayout imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    VkAccelerationStructureKHR accelStruct = nullptr;
    VkDescriptorType type;
  };

  struct PBKey
  {
    PBKey() {}
    PBKey(const std::map<int, PBinding>& a_currBindings, VkShaderStageFlags a_shaderStage) : m_currBindings(a_currBindings), m_stage(a_shaderStage) {}

    bool operator==(const PBKey& rhs) const
    {
      if (rhs.m_currBindings.size() != m_currBindings.size())
        return false;

      if (rhs.m_stage != m_stage)
        return false;

      bool equals = true;

      for (auto p : m_currBindings)
      {
        auto p2 = rhs.m_currBindings.find(p.first);
        if (p2 != rhs.m_currBindings.end() && !(p2->second == p.second))
        {
          equals = false;
          break;
        }
      }

      return equals;
    }

    std::map<int, PBinding> m_currBindings;
    VkShaderStageFlags      m_stage;
  };
};

namespace std 
{

  template <>
  struct hash<vkfw::PBKey>
  {
    std::size_t operator()(const vkfw::PBKey& k) const
    {
      using std::size_t;
      using std::hash;
      using std::string;

      // Compute individual hash values and combine them using XOR and bit shifting:
      //
      size_t currHash = k.m_currBindings.size();

      for (const auto& b : k.m_currBindings)
      {
        currHash ^= ( (hash<int>()(b.first)));
        currHash ^= ( (hash<bool>()(b.second.buffView     == nullptr)  << 1 ));
        currHash ^= ( (hash<bool>()(b.second.imageView    == nullptr)  << 2 ));
        currHash ^= ( (hash<bool>()(b.second.imageSampler == nullptr)  << 3 ));
        currHash ^= ( (hash<bool>()(b.second.buffer == nullptr)        << 5 ));
        currHash ^= ( (hash<bool>()(b.second.accelStruct == nullptr)   << 7 ));
        currHash ^= ( (hash<int> ()(b.second.type))                        << 9);
      }

      return currHash;
    }
  };
}



#endif