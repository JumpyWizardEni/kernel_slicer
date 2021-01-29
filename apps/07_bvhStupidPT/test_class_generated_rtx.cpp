#include <vector>
#include <memory>
#include <limits>

#include <cassert>

#include "vulkan_basics.h"
#include "test_class_generated_rtx.h"
#include "include/TestClass_ubo.h"

constexpr static uint32_t MEMCPY_BLOCK_SIZE = 256;

uint64_t TestClass_Generated_RTX::getBufferDeviceAddress(VkBuffer buffer)
{
  VkBufferDeviceAddressInfoKHR bufferDeviceAI{};
  bufferDeviceAI.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
  bufferDeviceAI.buffer = buffer;
  return vkGetBufferDeviceAddressKHR(device, &bufferDeviceAI);
}

TestClass_Generated_RTX::~TestClass_Generated_RTX()
{
  m_pMaker = nullptr;

  vkDestroyDescriptorSetLayout(device, ContributeToImageDSLayout, nullptr);
  ContributeToImageDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, NextBounceDSLayout, nullptr);
  NextBounceDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, InitAccumDataDSLayout, nullptr);
  InitAccumDataDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, RayTraceDSLayout, nullptr);
  RayTraceDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, InitEyeRayDSLayout, nullptr);
  InitEyeRayDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, GetMaterialColorDSLayout, nullptr);
  GetMaterialColorDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, PackXYDSLayout, nullptr);
  PackXYDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, copyKernelFloatDSLayout, nullptr);
  vkDestroyDescriptorPool(device, m_dsPool, NULL); m_dsPool = VK_NULL_HANDLE;

  vkDestroyBuffer(device, StupidPathTrace_local.hitBuffer, nullptr);
  vkDestroyBuffer(device, StupidPathTrace_local.threadFlagsBuffer, nullptr);
  vkDestroyBuffer(device, StupidPathTrace_local.rayDirAndFarBuffer, nullptr);
  vkDestroyBuffer(device, StupidPathTrace_local.rayPosAndNearBuffer, nullptr);
  vkDestroyBuffer(device, StupidPathTrace_local.accumThoroughputBuffer, nullptr);
  vkDestroyBuffer(device, StupidPathTrace_local.accumColorBuffer, nullptr);

  vkDestroyBuffer(device, CastSingleRay_local.hitBuffer, nullptr);
  vkDestroyBuffer(device, CastSingleRay_local.threadFlagsBuffer, nullptr);
  vkDestroyBuffer(device, CastSingleRay_local.rayDirAndFarBuffer, nullptr);
  vkDestroyBuffer(device, CastSingleRay_local.rayPosAndNearBuffer, nullptr);


  vkDestroyBuffer(device, m_classDataBuffer, nullptr);

  vkDestroyBuffer(device, m_vdata.m_randomGensBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.spheresMaterialsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_vPos4fBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_intervalsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_vNorm4fBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_nodesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_indicesReorderedBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_indicesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_matrixBuffer, nullptr);

  if(m_allMem != VK_NULL_HANDLE)
    vkFreeMemory(device, m_allMem, nullptr);
  
  if(m_vdata.m_vecMem != VK_NULL_HANDLE)
    vkFreeMemory(device, m_vdata.m_vecMem, nullptr);
}

void TestClass_Generated_RTX::InitHelpers()
{
  vkGetBufferDeviceAddressKHR = reinterpret_cast<PFN_vkGetBufferDeviceAddressKHR>(vkGetDeviceProcAddr(device, "vkGetBufferDeviceAddressKHR"));
  vkCmdBuildAccelerationStructuresKHR = reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(vkGetDeviceProcAddr(device, "vkCmdBuildAccelerationStructuresKHR"));
  vkBuildAccelerationStructuresKHR = reinterpret_cast<PFN_vkBuildAccelerationStructuresKHR>(vkGetDeviceProcAddr(device, "vkBuildAccelerationStructuresKHR"));
  vkCreateAccelerationStructureKHR = reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(vkGetDeviceProcAddr(device, "vkCreateAccelerationStructureKHR"));
  vkDestroyAccelerationStructureKHR = reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(vkGetDeviceProcAddr(device, "vkDestroyAccelerationStructureKHR"));
  vkGetAccelerationStructureBuildSizesKHR = reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(vkGetDeviceProcAddr(device, "vkGetAccelerationStructureBuildSizesKHR"));
  vkGetAccelerationStructureDeviceAddressKHR = reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(vkGetDeviceProcAddr(device, "vkGetAccelerationStructureDeviceAddressKHR"));
  vkCmdTraceRaysKHR = reinterpret_cast<PFN_vkCmdTraceRaysKHR>(vkGetDeviceProcAddr(device, "vkCmdTraceRaysKHR"));
  vkGetRayTracingShaderGroupHandlesKHR = reinterpret_cast<PFN_vkGetRayTracingShaderGroupHandlesKHR>(vkGetDeviceProcAddr(device, "vkGetRayTracingShaderGroupHandlesKHR"));
  vkCreateRayTracingPipelinesKHR = reinterpret_cast<PFN_vkCreateRayTracingPipelinesKHR>(vkGetDeviceProcAddr(device, "vkCreateRayTracingPipelinesKHR"));


  vkGetPhysicalDeviceProperties(physicalDevice, &m_devProps);
  m_pMaker = std::make_unique<vkfw::ComputePipelineMaker>();


  rtxPipelineProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR;
  VkPhysicalDeviceProperties2 deviceProperties2{};
  deviceProperties2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
  deviceProperties2.pNext = &rtxPipelineProperties;
  vkGetPhysicalDeviceProperties2(physicalDevice, &deviceProperties2);

  accStructureFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
  VkPhysicalDeviceFeatures2 deviceFeatures2{};
  deviceFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  deviceFeatures2.pNext = &accStructureFeatures;
  vkGetPhysicalDeviceFeatures2(physicalDevice, &deviceFeatures2);
}

VkDescriptorSetLayout TestClass_Generated_RTX::CreateContributeToImageDSLayout()
{
  VkDescriptorSetLayoutBinding dsBindings[4+1] = {};
  
  // binding for a_accumColor
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for in_pakedXY
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for out_color
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[2].pImmutableSamplers = nullptr;

  // binding for kgen_threadFlags
  dsBindings[3].binding            = 3;
  dsBindings[3].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[3].descriptorCount    = 1;
  dsBindings[3].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[3].pImmutableSamplers = nullptr;

  // binding for POD members stored in m_classDataBuffer
  dsBindings[4].binding            = 4;
  dsBindings[4].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[4].descriptorCount    = 1;
  dsBindings[4].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[4].pImmutableSamplers = nullptr;
  
  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(4+1);
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings;
  
  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}

VkDescriptorSetLayout TestClass_Generated_RTX::CreateNextBounceDSLayout()
{
  VkDescriptorSetLayoutBinding dsBindings[9+1] = {};
  
  // binding for in_hit
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for rayPosAndNear
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for rayDirAndFar
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[2].pImmutableSamplers = nullptr;

  // binding for accumColor
  dsBindings[3].binding            = 3;
  dsBindings[3].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[3].descriptorCount    = 1;
  dsBindings[3].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[3].pImmutableSamplers = nullptr;

  // binding for accumThoroughput
  dsBindings[4].binding            = 4;
  dsBindings[4].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[4].descriptorCount    = 1;
  dsBindings[4].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[4].pImmutableSamplers = nullptr;

  // binding for kgen_threadFlags
  dsBindings[5].binding            = 5;
  dsBindings[5].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[5].descriptorCount    = 1;
  dsBindings[5].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[5].pImmutableSamplers = nullptr;

  // binding for m_randomGens
  dsBindings[6].binding            = 6;
  dsBindings[6].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[6].descriptorCount    = 1;
  dsBindings[6].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[6].pImmutableSamplers = nullptr;

  // binding for m_vNorm4f
  dsBindings[7].binding            = 7;
  dsBindings[7].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[7].descriptorCount    = 1;
  dsBindings[7].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[7].pImmutableSamplers = nullptr;

  // binding for spheresMaterials
  dsBindings[8].binding            = 8;
  dsBindings[8].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[8].descriptorCount    = 1;
  dsBindings[8].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[8].pImmutableSamplers = nullptr;

  // binding for POD members stored in m_classDataBuffer
  dsBindings[9].binding            = 9;
  dsBindings[9].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[9].descriptorCount    = 1;
  dsBindings[9].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[9].pImmutableSamplers = nullptr;
  
  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(9+1);
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings;
  
  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}

VkDescriptorSetLayout TestClass_Generated_RTX::CreateInitAccumDataDSLayout()
{
  VkDescriptorSetLayoutBinding dsBindings[3+1] = {};
  
  // binding for accumColor
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for accumuThoroughput
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for kgen_threadFlags
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[2].pImmutableSamplers = nullptr;

  // binding for POD members stored in m_classDataBuffer
  dsBindings[3].binding            = 3;
  dsBindings[3].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[3].descriptorCount    = 1;
  dsBindings[3].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[3].pImmutableSamplers = nullptr;
  
  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(3+1);
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings;
  
  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}

VkDescriptorSetLayout TestClass_Generated_RTX::CreateRayTraceDSLayout()
{
  VkDescriptorSetLayoutBinding dsBindings[8+1] = {};
  
  // binding for rayPosAndNear
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for rayDirAndFar
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for out_hit
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[2].pImmutableSamplers = nullptr;

  // binding for indicesReordered
  dsBindings[3].binding            = 3;
  dsBindings[3].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[3].descriptorCount    = 1;
  dsBindings[3].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[3].pImmutableSamplers = nullptr;

  // binding for meshVerts
  dsBindings[4].binding            = 4;
  dsBindings[4].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[4].descriptorCount    = 1;
  dsBindings[4].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[4].pImmutableSamplers = nullptr;

  // binding for kgen_threadFlags
  dsBindings[5].binding            = 5;
  dsBindings[5].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[5].descriptorCount    = 1;
  dsBindings[5].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[5].pImmutableSamplers = nullptr;

  // binding for m_intervals
  dsBindings[6].binding            = 6;
  dsBindings[6].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[6].descriptorCount    = 1;
  dsBindings[6].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[6].pImmutableSamplers = nullptr;

  // binding for m_nodes
  dsBindings[7].binding            = 7;
  dsBindings[7].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[7].descriptorCount    = 1;
  dsBindings[7].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[7].pImmutableSamplers = nullptr;

  // binding for POD members stored in m_classDataBuffer
  dsBindings[8].binding            = 8;
  dsBindings[8].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[8].descriptorCount    = 1;
  dsBindings[8].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[8].pImmutableSamplers = nullptr;
  
  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(8+1);
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings;
  
  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}

VkDescriptorSetLayout TestClass_Generated_RTX::CreateInitEyeRayDSLayout()
{
  VkDescriptorSetLayoutBinding dsBindings[4+1] = {};
  
  // binding for packedXY
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for rayPosAndNear
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for rayDirAndFar
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[2].pImmutableSamplers = nullptr;

  // binding for kgen_threadFlags
  dsBindings[3].binding            = 3;
  dsBindings[3].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[3].descriptorCount    = 1;
  dsBindings[3].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[3].pImmutableSamplers = nullptr;

  // binding for POD members stored in m_classDataBuffer
  dsBindings[4].binding            = 4;
  dsBindings[4].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[4].descriptorCount    = 1;
  dsBindings[4].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[4].pImmutableSamplers = nullptr;
  
  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(4+1);
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings;
  
  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}

VkDescriptorSetLayout TestClass_Generated_RTX::CreateGetMaterialColorDSLayout()
{
  VkDescriptorSetLayoutBinding dsBindings[4+1] = {};
  
  // binding for in_hit
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for out_color
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for kgen_threadFlags
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[2].pImmutableSamplers = nullptr;

  // binding for spheresMaterials
  dsBindings[3].binding            = 3;
  dsBindings[3].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[3].descriptorCount    = 1;
  dsBindings[3].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[3].pImmutableSamplers = nullptr;

  // binding for POD members stored in m_classDataBuffer
  dsBindings[4].binding            = 4;
  dsBindings[4].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[4].descriptorCount    = 1;
  dsBindings[4].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[4].pImmutableSamplers = nullptr;
  
  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(4+1);
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings;
  
  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}

VkDescriptorSetLayout TestClass_Generated_RTX::CreatePackXYDSLayout()
{
  VkDescriptorSetLayoutBinding dsBindings[1+1] = {};
  
  // binding for out_pakedXY
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for POD members stored in m_classDataBuffer
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[1].pImmutableSamplers = nullptr;
  
  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(1+1);
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings;
  
  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}


VkDescriptorSetLayout TestClass_Generated_RTX::CreatecopyKernelFloatDSLayout()
{
  VkDescriptorSetLayoutBinding dsBindings[3] = {};

  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[0].pImmutableSamplers = nullptr;

  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for POD members stored in m_classDataBuffer
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[2].pImmutableSamplers = nullptr;

  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = 3;
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings;

  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}

void TestClass_Generated_RTX::InitKernels(const char* a_filePath, uint32_t a_blockSizeX, uint32_t a_blockSizeY, uint32_t a_blockSizeZ,
                                              KernelConfig* a_kernelConfigs, size_t a_configSize)
{
  VkSpecializationMapEntry specializationEntries[3] = {};
  {
    specializationEntries[0].constantID = 0;
    specializationEntries[0].offset     = 0;
    specializationEntries[0].size       = sizeof(uint32_t);
  
    specializationEntries[1].constantID = 1;
    specializationEntries[1].offset     = sizeof(uint32_t);
    specializationEntries[1].size       = sizeof(uint32_t);
  
    specializationEntries[2].constantID = 2;
    specializationEntries[2].offset     = 2 * sizeof(uint32_t);
    specializationEntries[2].size       = sizeof(uint32_t);
  }

  uint32_t specializationData[3] = {a_blockSizeX, a_blockSizeY, a_blockSizeZ};
  VkSpecializationInfo specsForWGSize = {};
  {
    specsForWGSize.mapEntryCount = 3;
    specsForWGSize.pMapEntries   = specializationEntries;
    specsForWGSize.dataSize      = 3 * sizeof(uint32_t);
    specsForWGSize.pData         = specializationData;
  }
  
  VkSpecializationInfo specsForWGSizeExcep = specsForWGSize;
  m_kernelExceptions.clear();
  for(size_t i=0;i<a_configSize;i++)
    m_kernelExceptions[a_kernelConfigs[i].kernelName] = a_kernelConfigs[i];

  {
    auto ex = m_kernelExceptions.find("kernel_ContributeToImage");
    if(ex == m_kernelExceptions.end())
    {
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSize, "kernel_ContributeToImage");
    }
    else
    {
      specsForWGSizeExcep.pData = ex->second.blockSize;   
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSizeExcep, "kernel_ContributeToImage");
    }    
    
    ContributeToImageDSLayout = CreateContributeToImageDSLayout();
    ContributeToImageLayout   = m_pMaker->MakeLayout(device, ContributeToImageDSLayout, 128); // at least 128 bytes for push constants
    ContributeToImagePipeline = m_pMaker->MakePipeline(device);   
  }

  {
    auto ex = m_kernelExceptions.find("kernel_NextBounce");
    if(ex == m_kernelExceptions.end())
    {
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSize, "kernel_NextBounce");
    }
    else
    {
      specsForWGSizeExcep.pData = ex->second.blockSize;   
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSizeExcep, "kernel_NextBounce");
    }    
    
    NextBounceDSLayout = CreateNextBounceDSLayout();
    NextBounceLayout   = m_pMaker->MakeLayout(device, NextBounceDSLayout, 128); // at least 128 bytes for push constants
    NextBouncePipeline = m_pMaker->MakePipeline(device);   
  }

  {
    auto ex = m_kernelExceptions.find("kernel_InitAccumData");
    if(ex == m_kernelExceptions.end())
    {
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSize, "kernel_InitAccumData");
    }
    else
    {
      specsForWGSizeExcep.pData = ex->second.blockSize;   
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSizeExcep, "kernel_InitAccumData");
    }    
    
    InitAccumDataDSLayout = CreateInitAccumDataDSLayout();
    InitAccumDataLayout   = m_pMaker->MakeLayout(device, InitAccumDataDSLayout, 128); // at least 128 bytes for push constants
    InitAccumDataPipeline = m_pMaker->MakePipeline(device);   
  }

  {
    auto ex = m_kernelExceptions.find("kernel_RayTrace");
    if(ex == m_kernelExceptions.end())
    {
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSize, "kernel_RayTrace");
    }
    else
    {
      specsForWGSizeExcep.pData = ex->second.blockSize;   
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSizeExcep, "kernel_RayTrace");
    }    
    
    RayTraceDSLayout = CreateRayTraceDSLayout();
    RayTraceLayout   = m_pMaker->MakeLayout(device, RayTraceDSLayout, 128); // at least 128 bytes for push constants
    RayTracePipeline = m_pMaker->MakePipeline(device);   
  }

  {
    auto ex = m_kernelExceptions.find("kernel_InitEyeRay");
    if(ex == m_kernelExceptions.end())
    {
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSize, "kernel_InitEyeRay");
    }
    else
    {
      specsForWGSizeExcep.pData = ex->second.blockSize;   
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSizeExcep, "kernel_InitEyeRay");
    }    
    
    InitEyeRayDSLayout = CreateInitEyeRayDSLayout();
    InitEyeRayLayout   = m_pMaker->MakeLayout(device, InitEyeRayDSLayout, 128); // at least 128 bytes for push constants
    InitEyeRayPipeline = m_pMaker->MakePipeline(device);   
  }

  {
    auto ex = m_kernelExceptions.find("kernel_GetMaterialColor");
    if(ex == m_kernelExceptions.end())
    {
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSize, "kernel_GetMaterialColor");
    }
    else
    {
      specsForWGSizeExcep.pData = ex->second.blockSize;   
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSizeExcep, "kernel_GetMaterialColor");
    }    
    
    GetMaterialColorDSLayout = CreateGetMaterialColorDSLayout();
    GetMaterialColorLayout   = m_pMaker->MakeLayout(device, GetMaterialColorDSLayout, 128); // at least 128 bytes for push constants
    GetMaterialColorPipeline = m_pMaker->MakePipeline(device);   
  }

  {
    auto ex = m_kernelExceptions.find("kernel_PackXY");
    if(ex == m_kernelExceptions.end())
    {
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSize, "kernel_PackXY");
    }
    else
    {
      specsForWGSizeExcep.pData = ex->second.blockSize;   
      m_pMaker->CreateShader(device, a_filePath, &specsForWGSizeExcep, "kernel_PackXY");
    }    
    
    PackXYDSLayout = CreatePackXYDSLayout();
    PackXYLayout   = m_pMaker->MakeLayout(device, PackXYDSLayout, 128); // at least 128 bytes for push constants
    PackXYPipeline = m_pMaker->MakePipeline(device);   
  }


  uint32_t specializationDataMemcpy[3] = {MEMCPY_BLOCK_SIZE, 1, 1};
  specsForWGSize.pData = specializationDataMemcpy;
  m_pMaker->CreateShader(device, a_filePath, &specsForWGSize, "copyKernelFloat");

  copyKernelFloatDSLayout = CreatecopyKernelFloatDSLayout();
  copyKernelFloatLayout   = m_pMaker->MakeLayout(device, copyKernelFloatDSLayout, sizeof(uint32_t)); // for this kernel we only need 4 bytes 
  copyKernelFloatPipeline = m_pMaker->MakePipeline(device);
}


void TestClass_Generated_RTX::UpdatePlainMembers(std::shared_ptr<vkfw::ICopyEngine> a_pCopyEngine)
{
  const size_t maxAllowedSize = std::numeric_limits<uint32_t>::max();

  m_uboData.m_worldViewProjInv = m_worldViewProjInv;
  m_uboData.camPos = camPos;
  m_uboData.m_randomGens_size     = uint32_t( m_randomGens.size() );    assert( m_randomGens.size() < maxAllowedSize );
  m_uboData.m_randomGens_capacity = uint32_t( m_randomGens.capacity() ); assert( m_randomGens.capacity() < maxAllowedSize );
  m_uboData.spheresMaterials_size     = uint32_t( spheresMaterials.size() );    assert( spheresMaterials.size() < maxAllowedSize );
  m_uboData.spheresMaterials_capacity = uint32_t( spheresMaterials.capacity() ); assert( spheresMaterials.capacity() < maxAllowedSize );
  m_uboData.m_vPos4f_size     = uint32_t( m_vPos4f.size() );    assert( m_vPos4f.size() < maxAllowedSize );
  m_uboData.m_vPos4f_capacity = uint32_t( m_vPos4f.capacity() ); assert( m_vPos4f.capacity() < maxAllowedSize );
  m_uboData.m_intervals_size     = uint32_t( m_intervals.size() );    assert( m_intervals.size() < maxAllowedSize );
  m_uboData.m_intervals_capacity = uint32_t( m_intervals.capacity() ); assert( m_intervals.capacity() < maxAllowedSize );
  m_uboData.m_vNorm4f_size     = uint32_t( m_vNorm4f.size() );    assert( m_vNorm4f.size() < maxAllowedSize );
  m_uboData.m_vNorm4f_capacity = uint32_t( m_vNorm4f.capacity() ); assert( m_vNorm4f.capacity() < maxAllowedSize );
  m_uboData.m_nodes_size     = uint32_t( m_nodes.size() );    assert( m_nodes.size() < maxAllowedSize );
  m_uboData.m_nodes_capacity = uint32_t( m_nodes.capacity() ); assert( m_nodes.capacity() < maxAllowedSize );
  m_uboData.m_indicesReordered_size     = uint32_t( m_indicesReordered.size() );    assert( m_indicesReordered.size() < maxAllowedSize );
  m_uboData.m_indicesReordered_capacity = uint32_t( m_indicesReordered.capacity() ); assert( m_indicesReordered.capacity() < maxAllowedSize );

  a_pCopyEngine->UpdateBuffer(m_classDataBuffer, 0, &m_uboData, sizeof(m_uboData));
}

void TestClass_Generated_RTX::UpdateVectorMembers(std::shared_ptr<vkfw::ICopyEngine> a_pCopyEngine)
{
  a_pCopyEngine->UpdateBuffer(m_vdata.m_randomGensBuffer, 0, m_randomGens.data(), m_randomGens.size()*sizeof(struct RandomGenT) );
  a_pCopyEngine->UpdateBuffer(m_vdata.spheresMaterialsBuffer, 0, spheresMaterials.data(), spheresMaterials.size()*sizeof(struct MaterialT) );
  a_pCopyEngine->UpdateBuffer(m_vdata.m_vPos4fBuffer, 0, m_vPos4f.data(), m_vPos4f.size()*sizeof(struct LiteMath::float4) );
  a_pCopyEngine->UpdateBuffer(m_vdata.m_intervalsBuffer, 0, m_intervals.data(), m_intervals.size()*sizeof(struct Interval) );
  a_pCopyEngine->UpdateBuffer(m_vdata.m_vNorm4fBuffer, 0, m_vNorm4f.data(), m_vNorm4f.size()*sizeof(struct LiteMath::float4) );
  a_pCopyEngine->UpdateBuffer(m_vdata.m_nodesBuffer, 0, m_nodes.data(), m_nodes.size()*sizeof(struct BVHNode) );
  a_pCopyEngine->UpdateBuffer(m_vdata.m_indicesReorderedBuffer, 0, m_indicesReordered.data(), m_indicesReordered.size()*sizeof(unsigned int) );
  a_pCopyEngine->UpdateBuffer(m_vdata.m_indicesBuffer, 0, m_indices.data(), m_indices.size()*sizeof(unsigned int) );

  a_pCopyEngine->UpdateBuffer(m_vdata.m_matrixBuffer, 0, &m_identityMatrix, sizeof(VkTransformMatrixKHR));
}

void TestClass_Generated_RTX::InitBuffers(size_t a_maxThreadsCount)
{
  std::vector<VkBuffer> allBuffers;

  StupidPathTrace_local.hitBuffer = vkfw::CreateBuffer(device, sizeof(Lite_Hit)*a_maxThreadsCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  allBuffers.push_back(StupidPathTrace_local.hitBuffer);
  StupidPathTrace_local.threadFlagsBuffer = vkfw::CreateBuffer(device, sizeof(uint)*a_maxThreadsCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  allBuffers.push_back(StupidPathTrace_local.threadFlagsBuffer);
  StupidPathTrace_local.rayDirAndFarBuffer = vkfw::CreateBuffer(device, sizeof(struct LiteMath::float4)*a_maxThreadsCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  allBuffers.push_back(StupidPathTrace_local.rayDirAndFarBuffer);
  StupidPathTrace_local.rayPosAndNearBuffer = vkfw::CreateBuffer(device, sizeof(struct LiteMath::float4)*a_maxThreadsCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  allBuffers.push_back(StupidPathTrace_local.rayPosAndNearBuffer);
  StupidPathTrace_local.accumThoroughputBuffer = vkfw::CreateBuffer(device, sizeof(struct LiteMath::float4)*a_maxThreadsCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  allBuffers.push_back(StupidPathTrace_local.accumThoroughputBuffer);
  StupidPathTrace_local.accumColorBuffer = vkfw::CreateBuffer(device, sizeof(struct LiteMath::float4)*a_maxThreadsCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  allBuffers.push_back(StupidPathTrace_local.accumColorBuffer);
  CastSingleRay_local.hitBuffer = vkfw::CreateBuffer(device, sizeof(Lite_Hit)*a_maxThreadsCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  allBuffers.push_back(CastSingleRay_local.hitBuffer);
  CastSingleRay_local.threadFlagsBuffer = vkfw::CreateBuffer(device, sizeof(uint)*a_maxThreadsCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  allBuffers.push_back(CastSingleRay_local.threadFlagsBuffer);
  CastSingleRay_local.rayDirAndFarBuffer = vkfw::CreateBuffer(device, sizeof(struct LiteMath::float4)*a_maxThreadsCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  allBuffers.push_back(CastSingleRay_local.rayDirAndFarBuffer);
  CastSingleRay_local.rayPosAndNearBuffer = vkfw::CreateBuffer(device, sizeof(struct LiteMath::float4)*a_maxThreadsCount, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
  allBuffers.push_back(CastSingleRay_local.rayPosAndNearBuffer);

  m_classDataBuffer = vkfw::CreateBuffer(device, sizeof(m_uboData),  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  allBuffers.push_back(m_classDataBuffer);
  
  if(allBuffers.size() > 0)
    m_allMem = vkfw::AllocateAndBindWithPadding(device, physicalDevice, allBuffers);
}

void TestClass_Generated_RTX::InitMemberBuffers()
{
  std::vector<VkBuffer> memberVectors;
  m_vdata.m_randomGensBuffer = vkfw::CreateBuffer(device, m_randomGens.capacity()*sizeof(struct RandomGenT), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_randomGensBuffer);
  m_vdata.spheresMaterialsBuffer = vkfw::CreateBuffer(device, spheresMaterials.capacity()*sizeof(struct MaterialT), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.spheresMaterialsBuffer);
  m_vdata.m_vPos4fBuffer = vkfw::CreateBuffer(device, m_vPos4f.capacity()*sizeof(struct LiteMath::float4),
                                   VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                              VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR);
  memberVectors.push_back(m_vdata.m_vPos4fBuffer);
  m_vdata.m_intervalsBuffer = vkfw::CreateBuffer(device, m_intervals.capacity()*sizeof(struct Interval), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_intervalsBuffer);
  m_vdata.m_vNorm4fBuffer = vkfw::CreateBuffer(device, m_vNorm4f.capacity()*sizeof(struct LiteMath::float4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_vNorm4fBuffer);
  m_vdata.m_nodesBuffer = vkfw::CreateBuffer(device, m_nodes.capacity()*sizeof(struct BVHNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_nodesBuffer);
  m_vdata.m_indicesReorderedBuffer = vkfw::CreateBuffer(device, m_indicesReordered.capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_indicesReorderedBuffer);
  m_vdata.m_indicesBuffer = vkfw::CreateBuffer(device, m_indices.capacity()*sizeof(unsigned int),
                                               VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                               VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR);
  memberVectors.push_back(m_vdata.m_indicesBuffer);

  m_vdata.m_matrixBuffer = vkfw::CreateBuffer(device, sizeof(VkTransformMatrixKHR),
                                              VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR);

  if(!memberVectors.empty())
    m_vdata.m_vecMem = vkfw::AllocateAndBindWithPadding(device, physicalDevice, memberVectors);
//  createScratchBuffer();

}

void TestClass_Generated_RTX::ContributeToImageCmd(uint tid, const float4* a_accumColor, const uint* in_pakedXY, float4* out_color)
{
  uint32_t blockSizeX = m_blockSize[0];
  uint32_t blockSizeY = m_blockSize[1];
  uint32_t blockSizeZ = m_blockSize[2];

  auto ex = m_kernelExceptions.find("kernel_ContributeToImage");
  if(ex != m_kernelExceptions.end())
  {
    blockSizeX = ex->second.blockSize[0];
    blockSizeY = ex->second.blockSize[1];
    blockSizeZ = ex->second.blockSize[2];
  }

  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, ContributeToImagePipeline);
  
  struct KernelArgsPC
  {
    
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;
  
  pcData.m_sizeX  = tid;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  
  vkCmdPushConstants(m_currCmdBuffer, ContributeToImageLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  vkCmdDispatch(
    m_currCmdBuffer,
    (tid + blockSizeX - 1) / blockSizeX,
    (1 + blockSizeY - 1) / blockSizeY,
    (1 + blockSizeZ - 1) / blockSizeZ);

  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_DEPENDENCY_BY_REGION_BIT, 1, &memoryBarrier, 0, nullptr, 0, nullptr);  
}

void TestClass_Generated_RTX::NextBounceCmd(uint tid, const Lite_Hit* in_hit, 
                                  float4* rayPosAndNear, float4* rayDirAndFar, float4* accumColor, float4* accumThoroughput)
{
  uint32_t blockSizeX = m_blockSize[0];
  uint32_t blockSizeY = m_blockSize[1];
  uint32_t blockSizeZ = m_blockSize[2];

  auto ex = m_kernelExceptions.find("kernel_NextBounce");
  if(ex != m_kernelExceptions.end())
  {
    blockSizeX = ex->second.blockSize[0];
    blockSizeY = ex->second.blockSize[1];
    blockSizeZ = ex->second.blockSize[2];
  }

  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, NextBouncePipeline);
  
  struct KernelArgsPC
  {
    
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;
  
  pcData.m_sizeX  = tid;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  
  vkCmdPushConstants(m_currCmdBuffer, NextBounceLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  vkCmdDispatch(
    m_currCmdBuffer,
    (tid + blockSizeX - 1) / blockSizeX,
    (1 + blockSizeY - 1) / blockSizeY,
    (1 + blockSizeZ - 1) / blockSizeZ);

  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_DEPENDENCY_BY_REGION_BIT, 1, &memoryBarrier, 0, nullptr, 0, nullptr);  
}

void TestClass_Generated_RTX::InitAccumDataCmd(uint tid, float4* accumColor, float4* accumuThoroughput)
{
  uint32_t blockSizeX = m_blockSize[0];
  uint32_t blockSizeY = m_blockSize[1];
  uint32_t blockSizeZ = m_blockSize[2];

  auto ex = m_kernelExceptions.find("kernel_InitAccumData");
  if(ex != m_kernelExceptions.end())
  {
    blockSizeX = ex->second.blockSize[0];
    blockSizeY = ex->second.blockSize[1];
    blockSizeZ = ex->second.blockSize[2];
  }

  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, InitAccumDataPipeline);
  
  struct KernelArgsPC
  {
    
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;
  
  pcData.m_sizeX  = tid;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  
  vkCmdPushConstants(m_currCmdBuffer, InitAccumDataLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  vkCmdDispatch(
    m_currCmdBuffer,
    (tid + blockSizeX - 1) / blockSizeX,
    (1 + blockSizeY - 1) / blockSizeY,
    (1 + blockSizeZ - 1) / blockSizeZ);

  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_DEPENDENCY_BY_REGION_BIT, 1, &memoryBarrier, 0, nullptr, 0, nullptr);  
}

void TestClass_Generated_RTX::RayTraceCmd(uint tid, const float4* rayPosAndNear, float4* rayDirAndFar,
                                Lite_Hit* out_hit, const uint* indicesReordered, const float4* meshVerts)
{
  uint32_t blockSizeX = m_blockSize[0];
  uint32_t blockSizeY = m_blockSize[1];
  uint32_t blockSizeZ = m_blockSize[2];

  auto ex = m_kernelExceptions.find("kernel_RayTrace");
  if(ex != m_kernelExceptions.end())
  {
    blockSizeX = ex->second.blockSize[0];
    blockSizeY = ex->second.blockSize[1];
    blockSizeZ = ex->second.blockSize[2];
  }

  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, RayTracePipeline);
  
  struct KernelArgsPC
  {
    
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;
  
  pcData.m_sizeX  = tid;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  
  vkCmdPushConstants(m_currCmdBuffer, RayTraceLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  vkCmdDispatch(
    m_currCmdBuffer,
    (tid + blockSizeX - 1) / blockSizeX,
    (1 + blockSizeY - 1) / blockSizeY,
    (1 + blockSizeZ - 1) / blockSizeZ);

  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_DEPENDENCY_BY_REGION_BIT, 1, &memoryBarrier, 0, nullptr, 0, nullptr);  
}

void TestClass_Generated_RTX::InitEyeRayCmd(uint tid, const uint* packedXY, float4* rayPosAndNear, float4* rayDirAndFar)
{
  uint32_t blockSizeX = m_blockSize[0];
  uint32_t blockSizeY = m_blockSize[1];
  uint32_t blockSizeZ = m_blockSize[2];

  auto ex = m_kernelExceptions.find("kernel_InitEyeRay");
  if(ex != m_kernelExceptions.end())
  {
    blockSizeX = ex->second.blockSize[0];
    blockSizeY = ex->second.blockSize[1];
    blockSizeZ = ex->second.blockSize[2];
  }

  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, InitEyeRayPipeline);
  
  struct KernelArgsPC
  {
    
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;
  
  pcData.m_sizeX  = tid;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  
  vkCmdPushConstants(m_currCmdBuffer, InitEyeRayLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  vkCmdDispatch(
    m_currCmdBuffer,
    (tid + blockSizeX - 1) / blockSizeX,
    (1 + blockSizeY - 1) / blockSizeY,
    (1 + blockSizeZ - 1) / blockSizeZ);

  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_DEPENDENCY_BY_REGION_BIT, 1, &memoryBarrier, 0, nullptr, 0, nullptr);  
}

void TestClass_Generated_RTX::GetMaterialColorCmd(uint tid, const Lite_Hit* in_hit, 
                                        uint* out_color)
{
  uint32_t blockSizeX = m_blockSize[0];
  uint32_t blockSizeY = m_blockSize[1];
  uint32_t blockSizeZ = m_blockSize[2];

  auto ex = m_kernelExceptions.find("kernel_GetMaterialColor");
  if(ex != m_kernelExceptions.end())
  {
    blockSizeX = ex->second.blockSize[0];
    blockSizeY = ex->second.blockSize[1];
    blockSizeZ = ex->second.blockSize[2];
  }

  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, GetMaterialColorPipeline);
  
  struct KernelArgsPC
  {
    
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;
  
  pcData.m_sizeX  = tid;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  
  vkCmdPushConstants(m_currCmdBuffer, GetMaterialColorLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  vkCmdDispatch(
    m_currCmdBuffer,
    (tid + blockSizeX - 1) / blockSizeX,
    (1 + blockSizeY - 1) / blockSizeY,
    (1 + blockSizeZ - 1) / blockSizeZ);

  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_DEPENDENCY_BY_REGION_BIT, 1, &memoryBarrier, 0, nullptr, 0, nullptr);  
}

void TestClass_Generated_RTX::PackXYCmd(uint tidX, uint tidY, uint* out_pakedXY)
{
  uint32_t blockSizeX = m_blockSize[0];
  uint32_t blockSizeY = m_blockSize[1];
  uint32_t blockSizeZ = m_blockSize[2];

  auto ex = m_kernelExceptions.find("kernel_PackXY");
  if(ex != m_kernelExceptions.end())
  {
    blockSizeX = ex->second.blockSize[0];
    blockSizeY = ex->second.blockSize[1];
    blockSizeZ = ex->second.blockSize[2];
  }

  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, PackXYPipeline);
  
  struct KernelArgsPC
  {
    
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;
  
  pcData.m_sizeX  = tidX;
  pcData.m_sizeY  = tidY;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  
  vkCmdPushConstants(m_currCmdBuffer, PackXYLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  vkCmdDispatch(
    m_currCmdBuffer,
    (tidX + blockSizeX - 1) / blockSizeX,
    (tidY + blockSizeY - 1) / blockSizeY,
    (1 + blockSizeZ - 1) / blockSizeZ);

  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_DEPENDENCY_BY_REGION_BIT, 1, &memoryBarrier, 0, nullptr, 0, nullptr);  
}


void TestClass_Generated_RTX::copyKernelFloatCmd(uint32_t length)
{
  uint32_t blockSizeX = MEMCPY_BLOCK_SIZE;

  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, copyKernelFloatPipeline);

  vkCmdPushConstants(m_currCmdBuffer, copyKernelFloatLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(uint32_t), &length);
  vkCmdDispatch(m_currCmdBuffer, (length + blockSizeX - 1) / blockSizeX, 1, 1);

  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_DEPENDENCY_BY_REGION_BIT, 1, &memoryBarrier, 0, nullptr, 0, nullptr);
}

void TestClass_Generated_RTX::StupidPathTraceCmd(VkCommandBuffer a_commandBuffer, uint tid, uint a_maxDepth, uint* in_pakedXY, float4* out_color)
{
  m_currCmdBuffer = a_commandBuffer;
  const uint32_t outOfForFlags  = KGEN_FLAG_RETURN;
  const uint32_t inForFlags     = KGEN_FLAG_RETURN | KGEN_FLAG_BREAK;
  const uint32_t outOfForFlagsN = KGEN_FLAG_RETURN | KGEN_FLAG_SET_EXIT_NEGATIVE;
  const uint32_t inForFlagsN    = KGEN_FLAG_RETURN | KGEN_FLAG_BREAK | KGEN_FLAG_SET_EXIT_NEGATIVE;
  const uint32_t outOfForFlagsD = KGEN_FLAG_RETURN | KGEN_FLAG_DONT_SET_EXIT;
  const uint32_t inForFlagsD    = KGEN_FLAG_RETURN | KGEN_FLAG_BREAK | KGEN_FLAG_DONT_SET_EXIT;
  vkCmdFillBuffer(a_commandBuffer, StupidPathTrace_local.threadFlagsBuffer , 0, VK_WHOLE_SIZE, 0); // zero thread flags, mark all threads to be active
  VkMemoryBarrier fillBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT }; 
  vkCmdPipelineBarrier(a_commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_DEPENDENCY_BY_REGION_BIT, 1, &fillBarrier, 0, nullptr, 0, nullptr); 

  float4 accumColor, accumThoroughput;
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, InitAccumDataLayout, 0, 1, &m_allGeneratedDS[0], 0, nullptr);
  m_currThreadFlags = outOfForFlags;
  InitAccumDataCmd(tid, &accumColor, &accumThoroughput);

  float4 rayPosAndNear, rayDirAndFar;
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, InitEyeRayLayout, 0, 1, &m_allGeneratedDS[1], 0, nullptr);
  m_currThreadFlags = outOfForFlags;
  InitEyeRayCmd(tid, in_pakedXY, &rayPosAndNear, &rayDirAndFar);

  for(int depth = 0; depth < a_maxDepth; depth++) 
  {
    Lite_Hit hit;
    vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, RayTraceLayout, 0, 1, &m_allGeneratedDS[2], 0, nullptr);
  m_currThreadFlags = inForFlagsN;
  RayTraceCmd(tid, &rayPosAndNear, &rayDirAndFar, &hit, m_indicesReordered.data(), m_vPos4f.data());

    vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, NextBounceLayout, 0, 1, &m_allGeneratedDS[3], 0, nullptr);
  m_currThreadFlags = inForFlags;
  NextBounceCmd(tid, &hit, 
                      &rayPosAndNear, &rayDirAndFar, &accumColor, &accumThoroughput);
  }

  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, ContributeToImageLayout, 0, 1, &m_allGeneratedDS[4], 0, nullptr);
  m_currThreadFlags = outOfForFlags;
  ContributeToImageCmd(tid, &accumColor, in_pakedXY, 
                           out_color);
}

void TestClass_Generated_RTX::CastSingleRayCmd(VkCommandBuffer a_commandBuffer, uint tid, uint* in_pakedXY, uint* out_color)
{
  m_currCmdBuffer = a_commandBuffer;
  const uint32_t outOfForFlags  = KGEN_FLAG_RETURN;
  const uint32_t inForFlags     = KGEN_FLAG_RETURN | KGEN_FLAG_BREAK;
  const uint32_t outOfForFlagsN = KGEN_FLAG_RETURN | KGEN_FLAG_SET_EXIT_NEGATIVE;
  const uint32_t inForFlagsN    = KGEN_FLAG_RETURN | KGEN_FLAG_BREAK | KGEN_FLAG_SET_EXIT_NEGATIVE;
  const uint32_t outOfForFlagsD = KGEN_FLAG_RETURN | KGEN_FLAG_DONT_SET_EXIT;
  const uint32_t inForFlagsD    = KGEN_FLAG_RETURN | KGEN_FLAG_BREAK | KGEN_FLAG_DONT_SET_EXIT;
  vkCmdFillBuffer(a_commandBuffer, CastSingleRay_local.threadFlagsBuffer , 0, VK_WHOLE_SIZE, 0); // zero thread flags, mark all threads to be active
  VkMemoryBarrier fillBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT }; 
  vkCmdPipelineBarrier(a_commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_DEPENDENCY_BY_REGION_BIT, 1, &fillBarrier, 0, nullptr, 0, nullptr); 

  float4 rayPosAndNear, rayDirAndFar;
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, InitEyeRayLayout, 0, 1, &m_allGeneratedDS[5], 0, nullptr);
  m_currThreadFlags = outOfForFlags;
  InitEyeRayCmd(tid, in_pakedXY, &rayPosAndNear, &rayDirAndFar);

  Lite_Hit hit;
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, RayTraceLayout, 0, 1, &m_allGeneratedDS[6], 0, nullptr);
  m_currThreadFlags = outOfForFlagsN;
  RayTraceCmd(tid, &rayPosAndNear, &rayDirAndFar,
                      &hit, m_indicesReordered.data(), m_vPos4f.data());
  
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, GetMaterialColorLayout, 0, 1, &m_allGeneratedDS[7], 0, nullptr);
  m_currThreadFlags = outOfForFlags;
  GetMaterialColorCmd(tid, &hit, out_color);
}

void TestClass_Generated_RTX::PackXYCmd(VkCommandBuffer a_commandBuffer, uint tidX, uint tidY, uint* out_pakedXY)
{
  m_currCmdBuffer = a_commandBuffer;
  const uint32_t outOfForFlags  = KGEN_FLAG_RETURN;
  const uint32_t inForFlags     = KGEN_FLAG_RETURN | KGEN_FLAG_BREAK;

  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, PackXYLayout, 0, 1, &m_allGeneratedDS[8], 0, nullptr);
  m_currThreadFlags = outOfForFlags;
  PackXYCmd(tidX, tidY, out_pakedXY);
}

ScratchBuffer TestClass_Generated_RTX::createScratchBuffer(VkDeviceSize size)
{
  ScratchBuffer scratchBuffer{};

  scratchBuffer.handle = vkfw::CreateBuffer(device, size,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);

  VkMemoryRequirements memoryRequirements{};
  vkGetBufferMemoryRequirements(device, scratchBuffer.handle, &memoryRequirements);

  VkMemoryAllocateFlagsInfo memoryAllocateFlagsInfo{};
  memoryAllocateFlagsInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
  memoryAllocateFlagsInfo.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT_KHR;

  VkMemoryAllocateInfo memoryAllocateInfo = {};
  memoryAllocateInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  memoryAllocateInfo.pNext = &memoryAllocateFlagsInfo;
  memoryAllocateInfo.allocationSize = memoryRequirements.size;
  memoryAllocateInfo.memoryTypeIndex = vk_utils::FindMemoryType(memoryRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, physicalDevice);
  VK_CHECK_RESULT(vkAllocateMemory(device, &memoryAllocateInfo, nullptr, &scratchBuffer.memory));
  VK_CHECK_RESULT(vkBindBufferMemory(device, scratchBuffer.handle, scratchBuffer.memory, 0));

  VkBufferDeviceAddressInfoKHR bufferDeviceAddressInfo{};
  bufferDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
  bufferDeviceAddressInfo.buffer = scratchBuffer.handle;
  scratchBuffer.deviceAddress = vkGetBufferDeviceAddressKHR(device, &bufferDeviceAddressInfo);

  return scratchBuffer;
}

void TestClass_Generated_RTX::deleteScratchBuffer(ScratchBuffer& scratchBuffer)
{
  if (scratchBuffer.memory != VK_NULL_HANDLE) {
    vkFreeMemory(device, scratchBuffer.memory, nullptr);
  }
  if (scratchBuffer.handle != VK_NULL_HANDLE) {
    vkDestroyBuffer(device, scratchBuffer.handle, nullptr);
  }
}

void TestClass_Generated_RTX::createAccelerationStructures()
{
  VkDeviceOrHostAddressConstKHR vertexBufferDeviceAddress{};
  VkDeviceOrHostAddressConstKHR indexBufferDeviceAddress{};
  VkDeviceOrHostAddressConstKHR transformBufferDeviceAddress{};


  vertexBufferDeviceAddress.deviceAddress = getBufferDeviceAddress(m_vdata.m_vPos4fBuffer);
  indexBufferDeviceAddress.deviceAddress = getBufferDeviceAddress(m_vdata.m_indicesBuffer);
  transformBufferDeviceAddress.deviceAddress = getBufferDeviceAddress(m_vdata.m_matrixBuffer);

  // Build
  VkAccelerationStructureGeometryKHR accelerationStructureGeometry{};
  accelerationStructureGeometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
  accelerationStructureGeometry.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
  accelerationStructureGeometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
  accelerationStructureGeometry.geometry.triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
  accelerationStructureGeometry.geometry.triangles.vertexFormat = VK_FORMAT_R32G32B32A32_SFLOAT;
  accelerationStructureGeometry.geometry.triangles.vertexData = vertexBufferDeviceAddress;
  accelerationStructureGeometry.geometry.triangles.maxVertex = 3;
  accelerationStructureGeometry.geometry.triangles.vertexStride = sizeof(float4);
  accelerationStructureGeometry.geometry.triangles.indexType = VK_INDEX_TYPE_UINT32;
  accelerationStructureGeometry.geometry.triangles.indexData = indexBufferDeviceAddress;
  accelerationStructureGeometry.geometry.triangles.transformData.deviceAddress = 0;
  accelerationStructureGeometry.geometry.triangles.transformData.hostAddress = nullptr;
  accelerationStructureGeometry.geometry.triangles.transformData = transformBufferDeviceAddress;

  // Get size info
  VkAccelerationStructureBuildGeometryInfoKHR accStructureBuildGeometryInfo{};
  accStructureBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
  accStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
  accStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
  accStructureBuildGeometryInfo.geometryCount = 1;
  accStructureBuildGeometryInfo.pGeometries = &accelerationStructureGeometry;

  const uint32_t numTriangles = m_indices.size() / 3;
  VkAccelerationStructureBuildSizesInfoKHR accStructureBuildSizesInfo{};
  accStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
  vkGetAccelerationStructureBuildSizesKHR(
      device,
      VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
      &accStructureBuildGeometryInfo,
      &numTriangles,
      &accStructureBuildSizesInfo);

  m_blas.buffer = vkfw::CreateBuffer(device, accStructureBuildSizesInfo.accelerationStructureSize,
          VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);

  VkAccelerationStructureCreateInfoKHR accelerationStructureCreateInfo{};
  accelerationStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
  accelerationStructureCreateInfo.buffer = m_blas.buffer;
  accelerationStructureCreateInfo.size = accStructureBuildSizesInfo.accelerationStructureSize;
  accelerationStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
  vkCreateAccelerationStructureKHR(device, &accelerationStructureCreateInfo, nullptr, &m_blas.handle);

  ScratchBuffer scratchBuffer = createScratchBuffer(accStructureBuildSizesInfo.buildScratchSize);

  VkAccelerationStructureBuildGeometryInfoKHR accelerationBuildGeometryInfo{};
  accelerationBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
  accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
  accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
  accelerationBuildGeometryInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
  accelerationBuildGeometryInfo.dstAccelerationStructure = m_blas.handle;
  accelerationBuildGeometryInfo.geometryCount = 1;
  accelerationBuildGeometryInfo.pGeometries = &accelerationStructureGeometry;
  accelerationBuildGeometryInfo.scratchData.deviceAddress = scratchBuffer.deviceAddress;

  VkAccelerationStructureBuildRangeInfoKHR accelerationStructureBuildRangeInfo{};
  accelerationStructureBuildRangeInfo.primitiveCount = numTriangles;
  accelerationStructureBuildRangeInfo.primitiveOffset = 0;
  accelerationStructureBuildRangeInfo.firstVertex = 0;
  accelerationStructureBuildRangeInfo.transformOffset = 0;
  std::vector<VkAccelerationStructureBuildRangeInfoKHR*> accelerationBuildStructureRangeInfos = { &accelerationStructureBuildRangeInfo };

  if (accStructureFeatures.accelerationStructureHostCommands)
  {
    // Implementation supports building acceleration structure building on host
    vkBuildAccelerationStructuresKHR(
        device,
        VK_NULL_HANDLE,
        1,
        &accelerationBuildGeometryInfo,
        accelerationBuildStructureRangeInfos.data());
  }
  else
  {
    // Acceleration structure needs to be build on the device
    std::vector<VkCommandBuffer> commandBuffers = vk_utils::CreateCommandBuffers(device, m_commandPool, 1);

    vkCmdBuildAccelerationStructuresKHR(
        commandBuffers[0],
        1,
        &accelerationBuildGeometryInfo,
        accelerationBuildStructureRangeInfos.data());
    vk_utils::ExecuteCommandBufferNow(commandBuffers[0], m_queue, device);
  }

  VkAccelerationStructureDeviceAddressInfoKHR accelerationDeviceAddressInfo{};
  accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
  accelerationDeviceAddressInfo.accelerationStructure = m_blas.handle;
  m_blas.deviceAddress = vkGetAccelerationStructureDeviceAddressKHR(device, &accelerationDeviceAddressInfo);

  deleteScratchBuffer(scratchBuffer);

  /* TLAS */

  VkAccelerationStructureInstanceKHR instance{};
  instance.transform = m_identityMatrix;
  instance.instanceCustomIndex = 0;
  instance.mask = 0xFF;
  instance.instanceShaderBindingTableRecordOffset = 0;
  instance.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
  instance.accelerationStructureReference = m_blas.deviceAddress;

  VkDeviceOrHostAddressConstKHR instanceDataDeviceAddress = transformBufferDeviceAddress;
  //instanceDataDeviceAddress.deviceAddress = getBufferDeviceAddress(m_vdata.m_matrixBuffer);

  accelerationStructureGeometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
  accelerationStructureGeometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
  accelerationStructureGeometry.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
  accelerationStructureGeometry.geometry.instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
  accelerationStructureGeometry.geometry.instances.arrayOfPointers = VK_FALSE;
  accelerationStructureGeometry.geometry.instances.data = instanceDataDeviceAddress;


  VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeometryInfo{};
  accelerationStructureBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
  accelerationStructureBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
  accelerationStructureBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
  accelerationStructureBuildGeometryInfo.geometryCount = 1;
  accelerationStructureBuildGeometryInfo.pGeometries = &accelerationStructureGeometry;

  uint32_t primitive_count = 1;

  VkAccelerationStructureBuildSizesInfoKHR accelerationStructureBuildSizesInfo{};
  accelerationStructureBuildSizesInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
  vkGetAccelerationStructureBuildSizesKHR(
      device,
      VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
      &accelerationStructureBuildGeometryInfo,
      &primitive_count,
      &accelerationStructureBuildSizesInfo);

  m_tlas.buffer = vkfw::CreateBuffer(device, accelerationStructureBuildSizesInfo.accelerationStructureSize,
                                     VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);

  VkAccelerationStructureCreateInfoKHR accStructureCreateInfo{};
  accStructureCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
  accStructureCreateInfo.buffer = m_tlas.buffer;
  accStructureCreateInfo.size = accelerationStructureBuildSizesInfo.accelerationStructureSize;
  accStructureCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
  vkCreateAccelerationStructureKHR(device, &accStructureCreateInfo, nullptr, &m_tlas.handle);

  // Create a small scratch buffer used during build of the top level acceleration structure
  scratchBuffer = createScratchBuffer(accStructureBuildSizesInfo.buildScratchSize);

  accelerationBuildGeometryInfo = {};
  accelerationBuildGeometryInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
  accelerationBuildGeometryInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
  accelerationBuildGeometryInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
  accelerationBuildGeometryInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
  accelerationBuildGeometryInfo.dstAccelerationStructure = m_tlas.handle;
  accelerationBuildGeometryInfo.geometryCount = 1;
  accelerationBuildGeometryInfo.pGeometries = &accelerationStructureGeometry;
  accelerationBuildGeometryInfo.scratchData.deviceAddress = scratchBuffer.deviceAddress;

  accelerationStructureBuildRangeInfo = {};
  accelerationStructureBuildRangeInfo.primitiveCount = 1;
  accelerationStructureBuildRangeInfo.primitiveOffset = 0;
  accelerationStructureBuildRangeInfo.firstVertex = 0;
  accelerationStructureBuildRangeInfo.transformOffset = 0;

  accelerationBuildStructureRangeInfos = { &accelerationStructureBuildRangeInfo };

  if (accStructureFeatures.accelerationStructureHostCommands)
  {
    // Implementation supports building acceleration structure building on host
    vkBuildAccelerationStructuresKHR(
        device,
        VK_NULL_HANDLE,
        1,
        &accelerationBuildGeometryInfo,
        accelerationBuildStructureRangeInfos.data());
  }
  else
  {
    // Acceleration structure needs to be build on the device
    std::vector<VkCommandBuffer> commandBuffers = vk_utils::CreateCommandBuffers(device, m_commandPool, 1);

    vkCmdBuildAccelerationStructuresKHR(
        commandBuffers[0],
        1,
        &accelerationBuildGeometryInfo,
        accelerationBuildStructureRangeInfos.data());
    vk_utils::ExecuteCommandBufferNow(commandBuffers[0], m_queue, device);
  }

  accelerationDeviceAddressInfo = {};
  accelerationDeviceAddressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
  accelerationDeviceAddressInfo.accelerationStructure = m_tlas.handle;
  m_tlas.deviceAddress = vkGetAccelerationStructureDeviceAddressKHR(device, &accelerationDeviceAddressInfo);

  deleteScratchBuffer(scratchBuffer);
}

void TestClass_Generated_RTX::createRTXPipeline()
{
  VkPipelineLayoutCreateInfo pipelineLayoutCI{};
  pipelineLayoutCI.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  pipelineLayoutCI.setLayoutCount = 1;
  pipelineLayoutCI.pSetLayouts = &rtxDSLayout;
  VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pipelineLayoutCI, nullptr, &m_RTXpipelineLayout));

  std::vector<VkPipelineShaderStageCreateInfo> shaderStages;

  // Ray generation
  {
    VkPipelineShaderStageCreateInfo stage_info = {};
    stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stage_info.stage = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    auto shaderCode = vk_utils::ReadFile("raygen.rgen.spv");
    VkShaderModule shaderModule = vk_utils::CreateShaderModule(device, shaderCode);
    stage_info.module = shaderModule;
    stage_info.pName  = "main";
    stage_info.pSpecializationInfo = VK_NULL_HANDLE;
    shaderStages.push_back(stage_info);

    VkRayTracingShaderGroupCreateInfoKHR shaderGroup{};
    shaderGroup.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
    shaderGroup.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
    shaderGroup.generalShader = static_cast<uint32_t>(shaderStages.size()) - 1;
    shaderGroup.closestHitShader = VK_SHADER_UNUSED_KHR;
    shaderGroup.anyHitShader = VK_SHADER_UNUSED_KHR;
    shaderGroup.intersectionShader = VK_SHADER_UNUSED_KHR;
    shaderGroups.push_back(shaderGroup);
  }

  // Miss
  {
    VkPipelineShaderStageCreateInfo stage_info = {};
    stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stage_info.stage = VK_SHADER_STAGE_MISS_BIT_KHR;
    auto shaderCode = vk_utils::ReadFile("miss.rmiss.spv");
    VkShaderModule shaderModule = vk_utils::CreateShaderModule(device, shaderCode);
    stage_info.module = shaderModule;
    stage_info.pName  = "main";
    stage_info.pSpecializationInfo = VK_NULL_HANDLE;
    shaderStages.push_back(stage_info);

    VkRayTracingShaderGroupCreateInfoKHR shaderGroup{};
    shaderGroup.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
    shaderGroup.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
    shaderGroup.generalShader = static_cast<uint32_t>(shaderStages.size()) - 1;
    shaderGroup.closestHitShader = VK_SHADER_UNUSED_KHR;
    shaderGroup.anyHitShader = VK_SHADER_UNUSED_KHR;
    shaderGroup.intersectionShader = VK_SHADER_UNUSED_KHR;
    shaderGroups.push_back(shaderGroup);
  }

  // Closest hit
  {
    VkPipelineShaderStageCreateInfo stage_info = {};
    stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stage_info.stage = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
    auto shaderCode = vk_utils::ReadFile("closesthit.rchit.spv");
    VkShaderModule shaderModule = vk_utils::CreateShaderModule(device, shaderCode);
    stage_info.module = shaderModule;
    stage_info.pName  = "main";
    stage_info.pSpecializationInfo = VK_NULL_HANDLE;
    shaderStages.push_back(stage_info);

    VkRayTracingShaderGroupCreateInfoKHR shaderGroup{};
    shaderGroup.sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
    shaderGroup.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
    shaderGroup.generalShader = VK_SHADER_UNUSED_KHR;
    shaderGroup.closestHitShader = static_cast<uint32_t>(shaderStages.size()) - 1;
    shaderGroup.anyHitShader = VK_SHADER_UNUSED_KHR;
    shaderGroup.intersectionShader = VK_SHADER_UNUSED_KHR;
    shaderGroups.push_back(shaderGroup);
  }

  /*
    Create the ray tracing pipeline
  */
  VkRayTracingPipelineCreateInfoKHR rayTracingPipelineCI{};
  rayTracingPipelineCI.sType = VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR;
  rayTracingPipelineCI.stageCount = static_cast<uint32_t>(shaderStages.size());
  rayTracingPipelineCI.pStages = shaderStages.data();
  rayTracingPipelineCI.groupCount = static_cast<uint32_t>(shaderGroups.size());
  rayTracingPipelineCI.pGroups = shaderGroups.data();
  rayTracingPipelineCI.maxPipelineRayRecursionDepth = 1;
  rayTracingPipelineCI.layout = m_RTXpipelineLayout;
  VK_CHECK_RESULT(vkCreateRayTracingPipelinesKHR(device, VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &rayTracingPipelineCI, nullptr, &m_RTXpipeline));
}

VkStridedDeviceAddressRegionKHR TestClass_Generated_RTX::getSbtStridedDeviceAddressRegion(VkBuffer buffer, uint32_t handleCount, uint32_t handleSizeAligned)
{
  VkStridedDeviceAddressRegionKHR stridedDeviceAddressRegionKHR{};
  stridedDeviceAddressRegionKHR.deviceAddress = getBufferDeviceAddress(buffer);
  stridedDeviceAddressRegionKHR.stride = handleSizeAligned;
  stridedDeviceAddressRegionKHR.size = handleCount * handleSizeAligned;
  return stridedDeviceAddressRegionKHR;
}

void TestClass_Generated_RTX::createShaderBindingTable()
{
  const uint32_t handleSize = rtxPipelineProperties.shaderGroupHandleSize;
  const uint32_t handleSizeAligned = vk_utils::Padding(rtxPipelineProperties.shaderGroupHandleSize, rtxPipelineProperties.shaderGroupHandleAlignment);
  const uint32_t groupCount = static_cast<uint32_t>(shaderGroups.size());
  const uint32_t sbtSize = groupCount * handleSizeAligned;

  std::vector<uint8_t> shaderHandleStorage(sbtSize);
  VK_CHECK_RESULT(vkGetRayTracingShaderGroupHandlesKHR(device, m_RTXpipeline, 0, groupCount, sbtSize, shaderHandleStorage.data()));

  const VkBufferUsageFlags bufferUsageFlags = VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
  const VkMemoryPropertyFlags memoryUsageFlags = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;


  m_tables.raygenSBTBuffer  = vkfw::CreateBuffer(device, handleSize, bufferUsageFlags);
  m_tables.raymissSBTBuffer = vkfw::CreateBuffer(device, handleSize, bufferUsageFlags);
  m_tables.rayhitSBTBuffer  = vkfw::CreateBuffer(device, handleSize, bufferUsageFlags);

  m_tables.raygenStridedDeviceAddressRegion  = getSbtStridedDeviceAddressRegion(m_tables.raygenSBTBuffer, 1, handleSizeAligned);
  m_tables.raymissStridedDeviceAddressRegion = getSbtStridedDeviceAddressRegion(m_tables.raymissSBTBuffer, 1, handleSizeAligned);
  m_tables.rayhitStridedDeviceAddressRegion  = getSbtStridedDeviceAddressRegion(m_tables.rayhitSBTBuffer, 1, handleSizeAligned);

  std::vector<VkBuffer> table_buffers = {m_tables.raygenSBTBuffer, m_tables.raymissSBTBuffer, m_tables.rayhitSBTBuffer};
  std::vector<VkDeviceSize> table_buffer_offsets = {m_tables.raygenOffset, m_tables.raymissOffset, m_tables.rayhitOffset};

  m_tables.memory = vkfw::AllocateAndBindWithPadding(device, physicalDevice, memoryUsageFlags, table_buffers, table_buffer_offsets);

  vkMapMemory(device, m_tables.memory, m_tables.raygenOffset, handleSizeAligned, 0, &m_tables.raygenMapped);
  vkMapMemory(device, m_tables.memory, m_tables.raymissOffset, handleSizeAligned, 0, &m_tables.raymissMapped);
  vkMapMemory(device, m_tables.memory, m_tables.rayhitOffset, handleSizeAligned, 0, &m_tables.rayhitMapped);

  memcpy(m_tables.raygenMapped, shaderHandleStorage.data(), handleSize);
  memcpy(m_tables.raymissMapped, shaderHandleStorage.data() + handleSizeAligned, handleSize);
  memcpy(m_tables.rayhitMapped, shaderHandleStorage.data() + handleSizeAligned * 2, handleSize);
}