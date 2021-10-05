#include "include/BasicLogic.h" 
#include "Bitmap.h"

#include <vector>
#include <iostream>
#include <memory>
#include <chrono>

#include "vk_utils.h"
#include "vk_descriptor_sets.h"
#include "vk_copy.h"
#include "vk_buffers.h"

#include "ray_tracing/vk_rt_utils.h"
#include "ray_tracing/vk_rt_funcs.h"

#include "scene_mgr.h"

#include "vulkan_basics.h"
#include "test_class_generated.h"

using LiteMath::uint4;

class TestClass_GPU : public TestClass_Generated
{
public:
  TestClass_GPU() 
  {
  
  }
  
  ~TestClass_GPU()
  {
    
  }
  
};

struct RTXDeviceFeatures
{
  VkPhysicalDeviceAccelerationStructureFeaturesKHR m_enabledAccelStructFeatures{};
  VkPhysicalDeviceBufferDeviceAddressFeatures      m_enabledDeviceAddressFeatures{};
  VkPhysicalDeviceRayQueryFeaturesKHR              m_enabledRayQueryFeatures;
};

static RTXDeviceFeatures SetupRTXFeatures(VkPhysicalDevice a_physDev)
{
  static RTXDeviceFeatures g_rtFeatures;

  g_rtFeatures.m_enabledRayQueryFeatures.sType    = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR;
  g_rtFeatures.m_enabledRayQueryFeatures.rayQuery = VK_TRUE;

  g_rtFeatures.m_enabledDeviceAddressFeatures.sType               = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES;
  g_rtFeatures.m_enabledDeviceAddressFeatures.bufferDeviceAddress = VK_TRUE;
  g_rtFeatures.m_enabledDeviceAddressFeatures.pNext               = &g_rtFeatures.m_enabledRayQueryFeatures;

  g_rtFeatures.m_enabledAccelStructFeatures.sType                 = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
  g_rtFeatures.m_enabledAccelStructFeatures.accelerationStructure = VK_TRUE;
  g_rtFeatures.m_enabledAccelStructFeatures.pNext                 = &g_rtFeatures.m_enabledDeviceAddressFeatures;

  return g_rtFeatures;
}

void test_class_gpu()
{
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
  enabledLayers.push_back("VK_LAYER_LUNARG_monitor");
  
  VK_CHECK_RESULT(volkInitialize());
  instance = vk_utils::createInstance(enableValidationLayers, enabledLayers, extensions);
  volkLoadInstance(instance);

  physicalDevice       = vk_utils::findPhysicalDevice(instance, true, 0);
  auto queueComputeFID = vk_utils::getQueueFamilyIndex(physicalDevice, VK_QUEUE_TRANSFER_BIT | VK_QUEUE_COMPUTE_BIT);
  
  // query features for RTX
  //
  RTXDeviceFeatures rtxFeatures = SetupRTXFeatures(physicalDevice);

  // query features for shaderInt8
  //
  VkPhysicalDeviceShaderFloat16Int8Features features = {};
  features.sType      = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_FLOAT16_INT8_FEATURES;
  features.shaderInt8 = VK_TRUE;
  features.pNext      = &rtxFeatures.m_enabledAccelStructFeatures;


  std::vector<const char*> validationLayers, deviceExtensions;
  VkPhysicalDeviceFeatures enabledDeviceFeatures = {};
  vk_utils::QueueFID_T fIDs = {};
  enabledDeviceFeatures.shaderInt64 = VK_TRUE;
  
  // Required by clspv for some reason
  deviceExtensions.push_back("VK_KHR_shader_non_semantic_info");
  deviceExtensions.push_back("VK_KHR_shader_float16_int8"); 
  
  // Required by VK_KHR_RAY_QUERY
  deviceExtensions.push_back(VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME);
  deviceExtensions.push_back(VK_KHR_RAY_QUERY_EXTENSION_NAME);
  deviceExtensions.push_back("VK_KHR_spirv_1_4");
  deviceExtensions.push_back("VK_KHR_shader_float_controls");  

  // Required by VK_KHR_acceleration_structure
  deviceExtensions.push_back(VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);
  deviceExtensions.push_back(VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME);
  deviceExtensions.push_back(VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME);

  // // Required by VK_KHR_ray_tracing_pipeline
  // m_deviceExtensions.push_back(VK_KHR_SPIRV_1_4_EXTENSION_NAME);
  // // Required by VK_KHR_spirv_1_4
  // m_deviceExtensions.push_back(VK_KHR_SHADER_FLOAT_CONTROLS_EXTENSION_NAME);

  fIDs.compute = queueComputeFID;
  device       = vk_utils::createLogicalDevice(physicalDevice, validationLayers, deviceExtensions, enabledDeviceFeatures,
                                               fIDs, VK_QUEUE_TRANSFER_BIT | VK_QUEUE_COMPUTE_BIT, &features);
  volkLoadDevice(device);
  vk_rt_utils::LoadRayTracingFunctions(device);

  commandPool  = vk_utils::createCommandPool(device, fIDs.compute, VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT);

  // (2) initialize vulkan helpers
  //  
  VkQueue computeQueue, transferQueue;
  {
    auto queueComputeFID = vk_utils::getQueueFamilyIndex(physicalDevice, VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT);
    vkGetDeviceQueue(device, queueComputeFID, 0, &computeQueue);
    vkGetDeviceQueue(device, queueComputeFID, 0, &transferQueue);
  }

  auto pCopyHelper = std::make_shared<vk_utils::SimpleCopyHelper>(physicalDevice, device, transferQueue, queueComputeFID, 8*1024*1024);\
  //auto pScnMgr     = std::make_shared<SceneManager>(device, physicalDevice, queueComputeFID, queueComputeFID, true);
  auto pGPUImpl    = std::make_shared<TestClass_GPU>();               // !!! USING GENERATED CODE !!! 
  
  pGPUImpl->InitVulkanObjects(device, physicalDevice, WIN_WIDTH*WIN_HEIGHT); // !!! USING GENERATED CODE !!!                        
  pGPUImpl->LoadScene("../10_virtual_func_rt_test1/cornell_collapsed.vsgf");

  // must initialize all vector members with correct capacity before call 'InitMemberBuffers()'
  //
  pGPUImpl->InitRandomGens(WIN_WIDTH*WIN_HEIGHT);                            // !!! USING GENERATED CODE !!!
  pGPUImpl->InitMemberBuffers();                                             // !!! USING GENERATED CODE !!!

  // (3) Create buffer
  //
  const size_t bufferSize1 = WIN_WIDTH*WIN_HEIGHT*sizeof(uint32_t);
  const size_t bufferSize2 = WIN_WIDTH*WIN_HEIGHT*sizeof(float)*4;
  VkBuffer xyBuffer        = vk_utils::createBuffer(device, bufferSize1,  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
  VkBuffer colorBuffer1    = vk_utils::createBuffer(device, bufferSize1,  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
  
  VkDeviceMemory colorMem  = vk_utils::allocateAndBindWithPadding(device, physicalDevice, {xyBuffer, colorBuffer1});
  
  pGPUImpl->SetVulkanInOutFor_PackXY(xyBuffer, 0);            // !!! USING GENERATED CODE !!! 

  pGPUImpl->SetVulkanInOutFor_CastSingleRay(xyBuffer,     0,  // !!! USING GENERATED CODE !!!
                                            colorBuffer1, 0); // !!! USING GENERATED CODE !!!

  pGPUImpl->UpdateAll(pCopyHelper);                           // !!! USING GENERATED CODE !!!
  
  // now compute some thing useful
  //
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    //vkCmdFillBuffer(commandBuffer, xyBuffer, 0, VK_WHOLE_SIZE, 0x0000FFFF); // fill with yellow color
    pGPUImpl->PackXYCmd(commandBuffer, WIN_WIDTH, WIN_HEIGHT, nullptr);       // !!! USING GENERATED CODE !!! 
    vkEndCommandBuffer(commandBuffer);  
    vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);

    vkResetCommandBuffer(commandBuffer, 0);
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    pGPUImpl->CastSingleRayCmd(commandBuffer, WIN_WIDTH*WIN_HEIGHT, nullptr, nullptr);  // !!! USING GENERATED CODE !!! 
    vkEndCommandBuffer(commandBuffer);  
   
    auto start = std::chrono::high_resolution_clock::now();
    vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
    auto stop = std::chrono::high_resolution_clock::now();
    float ms  = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
    std::cout << ms << " ms for full command buffer execution " << std::endl;

    std::vector<uint32_t> pixelData(WIN_WIDTH*WIN_HEIGHT);
    pCopyHelper->ReadBuffer(colorBuffer1, 0, pixelData.data(), pixelData.size()*sizeof(uint32_t));
    SaveBMP("zout_gpu.bmp", pixelData.data(), WIN_WIDTH, WIN_HEIGHT);    
  }
  
  // (6) destroy and free resources before exit
  //
  pGPUImpl    = nullptr;     
  pCopyHelper = nullptr;

  vkDestroyBuffer(device, xyBuffer, nullptr);
  vkDestroyBuffer(device, colorBuffer1, nullptr);
  vkFreeMemory(device, colorMem, nullptr);

  vkDestroyCommandPool(device, commandPool, nullptr);

  vkDestroyDevice(device, nullptr);
  vkDestroyInstance(instance, nullptr);
}