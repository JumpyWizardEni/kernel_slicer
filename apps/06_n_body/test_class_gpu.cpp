#include <vector>
#include <iostream>
#include <memory>
#include <chrono>

#include "vk_utils.h"
#include "vk_program.h"
#include "vk_copy.h"
#include "vk_buffer.h"

#include "vulkan_basics.h"
#include "test_class_generated.h"

std::vector<nBody::BodyState> n_body_gpu(uint32_t seed, uint32_t iterations)
{
  std::vector<nBody::BodyState> outBodies(nBody::BODIES_COUNT);

  // (1) init vulkan
  //
  VkInstance       instance       = VK_NULL_HANDLE;
  VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
  VkDevice         device         = VK_NULL_HANDLE;
  VkCommandPool    commandPool    = VK_NULL_HANDLE;

  #ifndef NDEBUG
  bool enableValidationLayers = true;
  #else
  bool enableValidationLayers = false;
  #endif

  std::vector<const char*> enabledLayers;
  std::vector<const char*> extensions;
  enabledLayers.push_back("VK_LAYER_KHRONOS_validation");
  enabledLayers.push_back("VK_LAYER_LUNARG_standard_validation");
  instance = vk_utils::CreateInstance(enableValidationLayers, enabledLayers, extensions);

  physicalDevice       = vk_utils::FindPhysicalDevice(instance, true, 0);
  auto queueComputeFID = vk_utils::GetQueueFamilyIndex(physicalDevice, VK_QUEUE_TRANSFER_BIT | VK_QUEUE_COMPUTE_BIT);

  // query for shaderInt8
  //
  VkPhysicalDeviceShaderFloat16Int8Features features = {};
  features.sType      = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_FLOAT16_INT8_FEATURES;
  features.shaderInt8 = VK_TRUE;

  VkPhysicalDeviceFeatures2 physDevFeatures2 = {};
  physDevFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  physDevFeatures2.pNext = &features;

  std::vector<const char*> validationLayers, deviceExtensions;
  VkPhysicalDeviceFeatures enabledDeviceFeatures = {};
  vk_utils::queueFamilyIndices fIDs = {};

  deviceExtensions.push_back("VK_KHR_shader_non_semantic_info");
  deviceExtensions.push_back("VK_KHR_shader_float16_int8");

  fIDs.compute = queueComputeFID;
  device       = vk_utils::CreateLogicalDevice(physicalDevice, validationLayers, deviceExtensions, enabledDeviceFeatures,
                                               fIDs, VK_QUEUE_TRANSFER_BIT | VK_QUEUE_COMPUTE_BIT, physDevFeatures2);

  commandPool  = vk_utils::CreateCommandPool(device, physicalDevice, VK_QUEUE_COMPUTE_BIT, VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT);

  // (2) initialize vulkan helpers
  //  
  VkQueue computeQueue, transferQueue;
  {
    auto queueComputeFID = vk_utils::GetQueueFamilyIndex(physicalDevice, VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT);
    vkGetDeviceQueue(device, queueComputeFID, 0, &computeQueue);
    vkGetDeviceQueue(device, queueComputeFID, 0, &transferQueue);
  }

  auto pCopyHelper = std::make_shared<vkfw::SimpleCopyHelper>(physicalDevice, device, transferQueue, queueComputeFID, 8*1024*1024);

  auto pGPUImpl = std::make_shared<nBody_Generated>();          // !!! USING GENERATED CODE !!!
  pGPUImpl->setParameters(seed, iterations);
  pGPUImpl->InitVulkanObjects(device, physicalDevice, nBody::BODIES_COUNT, 32, 1, 1); // !!! USING GENERATED CODE !!!

  pGPUImpl->InitMemberBuffers();                                      // !!! USING GENERATED CODE !!!
  pGPUImpl->UpdateAll(pCopyHelper);                                   // !!! USING GENERATED CODE !!!

  // (3) Create buffer
  //
  VkBuffer outBuffer = vkfw::CreateBuffer(device, nBody::BODIES_COUNT * sizeof(nBody::BodyState),  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
  VkDeviceMemory bufferMem    = vkfw::AllocateAndBindWithPadding(device, physicalDevice, {outBuffer});
  pGPUImpl->SetVulkanInOutFor_perform(outBuffer, 0);

  // now compute some thing useful
  //
  {
    VkCommandBuffer commandBuffer = vk_utils::CreateCommandBuffers(device, commandPool, 1)[0];

    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    // vkCmdFillBuffer(commandBuffer, outBuffer, 0, VK_WHOLE_SIZE, 0x0000FFFF); // fill with yellow color
    pGPUImpl->performCmd(commandBuffer, outBodies.data());         // !!! USING GENERATED CODE !!!
    vkEndCommandBuffer(commandBuffer);

    auto start = std::chrono::high_resolution_clock::now();
    vk_utils::ExecuteCommandBufferNow(commandBuffer, computeQueue, device);
    auto stop = std::chrono::high_resolution_clock::now();
    auto ms   = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
    std::cout << ms << " ms for command buffer execution " << std::endl;

    pCopyHelper->ReadBuffer(outBuffer, 0, outBodies.data(), outBodies.size() * sizeof(outBodies[0]));

    std::cout << std::endl;
  }

  // (6) destroy and free resources before exit
  //
  pCopyHelper = nullptr;
  pGPUImpl = nullptr;                                                       // !!! USING GENERATED CODE !!!

  vkDestroyBuffer(device, outBuffer, nullptr);
  vkFreeMemory(device, bufferMem, nullptr);

  vkDestroyCommandPool(device, commandPool, nullptr);

  vkDestroyDevice(device, nullptr);
  vkDestroyInstance(instance, nullptr);
  return outBodies;
}
