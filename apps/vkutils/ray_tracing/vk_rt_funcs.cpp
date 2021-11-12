#include "vk_rt_funcs.h"

//PFN_vkGetBufferDeviceAddressKHR vkGetBufferDeviceAddressKHR;
//PFN_vkCreateAccelerationStructureKHR vkCreateAccelerationStructureKHR;
//PFN_vkDestroyAccelerationStructureKHR vkDestroyAccelerationStructureKHR;
//PFN_vkGetAccelerationStructureBuildSizesKHR vkGetAccelerationStructureBuildSizesKHR;
//PFN_vkGetAccelerationStructureDeviceAddressKHR vkGetAccelerationStructureDeviceAddressKHR;
//PFN_vkCmdBuildAccelerationStructuresKHR vkCmdBuildAccelerationStructuresKHR;
//PFN_vkBuildAccelerationStructuresKHR vkBuildAccelerationStructuresKHR;
//PFN_vkCmdTraceRaysKHR vkCmdTraceRaysKHR;
//PFN_vkGetRayTracingShaderGroupHandlesKHR vkGetRayTracingShaderGroupHandlesKHR;
//PFN_vkCreateRayTracingPipelinesKHR vkCreateRayTracingPipelinesKHR;


void vk_rt_utils::LoadRayTracingFunctions(VkDevice a_device)
{
  //vkGetBufferDeviceAddressKHR = reinterpret_cast<PFN_vkGetBufferDeviceAddressKHR>(vkGetDeviceProcAddr(a_device, "vkGetBufferDeviceAddressKHR"));
  //vkCmdBuildAccelerationStructuresKHR = reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(vkGetDeviceProcAddr(a_device, "vkCmdBuildAccelerationStructuresKHR"));
  //vkBuildAccelerationStructuresKHR = reinterpret_cast<PFN_vkBuildAccelerationStructuresKHR>(vkGetDeviceProcAddr(a_device, "vkBuildAccelerationStructuresKHR"));
  //vkCreateAccelerationStructureKHR = reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(vkGetDeviceProcAddr(a_device, "vkCreateAccelerationStructureKHR"));
  //vkDestroyAccelerationStructureKHR = reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(vkGetDeviceProcAddr(a_device, "vkDestroyAccelerationStructureKHR"));
  //vkGetAccelerationStructureBuildSizesKHR = reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(vkGetDeviceProcAddr(a_device, "vkGetAccelerationStructureBuildSizesKHR"));
  //vkGetAccelerationStructureDeviceAddressKHR = reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(vkGetDeviceProcAddr(a_device, "vkGetAccelerationStructureDeviceAddressKHR"));
  //vkCmdTraceRaysKHR = reinterpret_cast<PFN_vkCmdTraceRaysKHR>(vkGetDeviceProcAddr(a_device, "vkCmdTraceRaysKHR"));
  //vkGetRayTracingShaderGroupHandlesKHR = reinterpret_cast<PFN_vkGetRayTracingShaderGroupHandlesKHR>(vkGetDeviceProcAddr(a_device, "vkGetRayTracingShaderGroupHandlesKHR"));
  //vkCreateRayTracingPipelinesKHR = reinterpret_cast<PFN_vkCreateRayTracingPipelinesKHR>(vkGetDeviceProcAddr(a_device, "vkCreateRayTracingPipelinesKHR"));
}