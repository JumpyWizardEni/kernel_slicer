#ifndef MAIN_CLASS_DECL_TestClassRTX_H
#define MAIN_CLASS_DECL_TestClassRTX_H

#include "test_class_generated.h"


struct ScratchBuffer
{
  uint64_t deviceAddress = 0;
  VkBuffer handle = VK_NULL_HANDLE;
  VkDeviceMemory memory = VK_NULL_HANDLE;
};

struct AccStructure
{
  VkAccelerationStructureKHR handle = VK_NULL_HANDLE;
  uint64_t deviceAddress = 0;
  VkDeviceMemory memory = VK_NULL_HANDLE;
  VkBuffer buffer = VK_NULL_HANDLE;
};


class TestClass_Generated_RTX : public TestClass_Generated
{
public:
  PFN_vkGetBufferDeviceAddressKHR vkGetBufferDeviceAddressKHR;
  PFN_vkCreateAccelerationStructureKHR vkCreateAccelerationStructureKHR;
  PFN_vkDestroyAccelerationStructureKHR vkDestroyAccelerationStructureKHR;
  PFN_vkGetAccelerationStructureBuildSizesKHR vkGetAccelerationStructureBuildSizesKHR;
  PFN_vkGetAccelerationStructureDeviceAddressKHR vkGetAccelerationStructureDeviceAddressKHR;
  PFN_vkCmdBuildAccelerationStructuresKHR vkCmdBuildAccelerationStructuresKHR;
  PFN_vkBuildAccelerationStructuresKHR vkBuildAccelerationStructuresKHR;
  PFN_vkCmdTraceRaysKHR vkCmdTraceRaysKHR;
  PFN_vkGetRayTracingShaderGroupHandlesKHR vkGetRayTracingShaderGroupHandlesKHR;
  PFN_vkCreateRayTracingPipelinesKHR vkCreateRayTracingPipelinesKHR;

  uint64_t getBufferDeviceAddress(VkBuffer buffer);
  VkStridedDeviceAddressRegionKHR getSbtStridedDeviceAddressRegion(VkBuffer buffer, uint32_t handleCount,
                                                                   uint32_t handleSizeAligned);

  TestClass_Generated_RTX() {}

  struct KernelConfig
  {
    std::string kernelName;
    uint32_t    blockSize[3] = {1,1,1};
  };

  virtual void InitVulkanObjects(VkDevice a_device, VkPhysicalDevice a_physicalDevice, size_t a_maxThreadsCount,
                                 uint32_t a_blockSizeX, uint32_t a_blockSizeY, uint32_t a_blockSizeZ, 
                                 KernelConfig* a_kernelConfigs = nullptr, size_t a_configSize = 0)
  {
    physicalDevice = a_physicalDevice;
    device         = a_device;
    
    m_blockSize[0] = a_blockSizeX;
    m_blockSize[1] = a_blockSizeY;
    m_blockSize[2] = a_blockSizeZ;

    InitHelpers();
    InitBuffers(a_maxThreadsCount);
    InitKernels("z_generated.cl.spv", a_blockSizeX, a_blockSizeY, a_blockSizeZ, a_kernelConfigs, a_configSize);
    AllocateAllDescriptorSets();
  }

  virtual void SetVulkanInOutFor_StupidPathTrace(
    VkBuffer a_in_pakedXYBuffer,
    size_t   a_in_pakedXYOffset,
    VkBuffer a_out_colorBuffer,
    size_t   a_out_colorOffset,
    uint32_t dummyArgument = 0)
  {
    StupidPathTrace_local.in_pakedXYBuffer = a_in_pakedXYBuffer;
    StupidPathTrace_local.in_pakedXYOffset = a_in_pakedXYOffset;
    StupidPathTrace_local.out_colorBuffer = a_out_colorBuffer;
    StupidPathTrace_local.out_colorOffset = a_out_colorOffset;
    InitAllGeneratedDescriptorSets_StupidPathTrace();
  }

  virtual void SetVulkanInOutFor_CastSingleRay(
    VkBuffer a_in_pakedXYBuffer,
    size_t   a_in_pakedXYOffset,
    VkBuffer a_out_colorBuffer,
    size_t   a_out_colorOffset,
    uint32_t dummyArgument = 0)
  {
    CastSingleRay_local.in_pakedXYBuffer = a_in_pakedXYBuffer;
    CastSingleRay_local.in_pakedXYOffset = a_in_pakedXYOffset;
    CastSingleRay_local.out_colorBuffer = a_out_colorBuffer;
    CastSingleRay_local.out_colorOffset = a_out_colorOffset;
    InitAllGeneratedDescriptorSets_CastSingleRay();
  }

  virtual void SetVulkanInOutFor_PackXY(
    VkBuffer a_out_pakedXYBuffer,
    size_t   a_out_pakedXYOffset,
    uint32_t dummyArgument = 0)
  {
    PackXY_local.out_pakedXYBuffer = a_out_pakedXYBuffer;
    PackXY_local.out_pakedXYOffset = a_out_pakedXYOffset;
    InitAllGeneratedDescriptorSets_PackXY();
  }

  virtual ~TestClass_Generated_RTX();

  virtual void InitMemberBuffers();

  virtual void UpdateAll(std::shared_ptr<vkfw::ICopyEngine> a_pCopyEngine)
  {
    UpdatePlainMembers(a_pCopyEngine);
    UpdateVectorMembers(a_pCopyEngine);
    createAccelerationStructures();
  }

  virtual void StupidPathTraceCmd(VkCommandBuffer a_commandBuffer, uint tid, uint a_maxDepth, uint* in_pakedXY, float4* out_color);
  virtual void CastSingleRayCmd(VkCommandBuffer a_commandBuffer, uint tid, uint* in_pakedXY, uint* out_color);
  virtual void PackXYCmd(VkCommandBuffer a_commandBuffer, uint tidX, uint tidY, uint* out_pakedXY);

  virtual void copyKernelFloatCmd(uint32_t length);
  
  virtual void ContributeToImageCmd(uint tid, const float4* a_accumColor, const uint* in_pakedXY, float4* out_color);
  virtual void NextBounceCmd(uint tid, const Lite_Hit* in_hit, 
                                  float4* rayPosAndNear, float4* rayDirAndFar, float4* accumColor, float4* accumThoroughput);
  virtual void InitAccumDataCmd(uint tid, float4* accumColor, float4* accumuThoroughput);
  virtual void RayTraceCmd(uint tid, const float4* rayPosAndNear, float4* rayDirAndFar,
                                Lite_Hit* out_hit, const uint* indicesReordered, const float4* meshVerts);
  virtual void InitEyeRayCmd(uint tid, const uint* packedXY, float4* rayPosAndNear, float4* rayDirAndFar);
  virtual void GetMaterialColorCmd(uint tid, const Lite_Hit* in_hit, 
                                        uint* out_color);
  virtual void PackXYCmd(uint tidX, uint tidY, uint* out_pakedXY);

  void InitRTXDS();

  ScratchBuffer createScratchBuffer(VkDeviceSize size);
  void deleteScratchBuffer(ScratchBuffer& scratchBuffer);
  void createAccelerationStructures();
  void createShaderBindingTable();
  void createRTXPipeline();

  VkCommandPool    m_commandPool;
  VkQueue m_queue;
protected:

  AccStructure m_blas{};
  AccStructure m_tlas{};

  VkTransformMatrixKHR m_identityMatrix = {
      1.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 1.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 1.0f, 0.0f
  };

  std::vector<VkRayTracingShaderGroupCreateInfoKHR> shaderGroups{};

  VkPhysicalDeviceRayTracingPipelinePropertiesKHR  rtxPipelineProperties{};
  VkPhysicalDeviceAccelerationStructureFeaturesKHR accStructureFeatures{};

  VkPhysicalDeviceBufferDeviceAddressFeatures enabledBufferDeviceAddressFeatures{};
  VkPhysicalDeviceRayTracingPipelineFeaturesKHR enabledRTXPipelineFeatures{};
  VkPhysicalDeviceAccelerationStructureFeaturesKHR enabledAccStructureFeatures{};

  struct BindingTables
  {
    VkBuffer raygenSBTBuffer = VK_NULL_HANDLE;
    VkStridedDeviceAddressRegionKHR raygenStridedDeviceAddressRegion;
    void * raygenMapped = nullptr;
    size_t raygenOffset = 0;

    VkBuffer raymissSBTBuffer = VK_NULL_HANDLE;
    VkStridedDeviceAddressRegionKHR raymissStridedDeviceAddressRegion;
    void * raymissMapped = nullptr;
    size_t raymissOffset = 0;

    VkBuffer rayhitSBTBuffer = VK_NULL_HANDLE;
    VkStridedDeviceAddressRegionKHR rayhitStridedDeviceAddressRegion;
    void * rayhitMapped = nullptr;
    size_t rayhitOffset = 0;

    VkDeviceMemory memory = VK_NULL_HANDLE;
  } m_tables;

  VkPipeline m_RTXpipeline;
  VkPipelineLayout m_RTXpipelineLayout;
  VkDescriptorSetLayout rtxDSLayout = VK_NULL_HANDLE;
  VkDescriptorSet m_rtxDS;
  VkDescriptorSetLayout CreateRayTraceDSLayoutRTX();

  VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
  VkDevice         device         = VK_NULL_HANDLE;

  VkCommandBuffer  m_currCmdBuffer   = VK_NULL_HANDLE;
  uint32_t         m_currThreadFlags = 0;

  std::unique_ptr<vkfw::ComputePipelineMaker> m_pMaker = nullptr;
  VkPhysicalDeviceProperties m_devProps;

  virtual void InitHelpers();
  virtual void InitBuffers(size_t a_maxThreadsCount);
  virtual void InitKernels(const char* a_filePath, uint32_t a_blockSizeX, uint32_t a_blockSizeY, uint32_t a_blockSizeZ,
                           KernelConfig* a_kernelConfigs, size_t a_configSize);
  virtual void AllocateAllDescriptorSets();

  virtual void InitAllGeneratedDescriptorSets_StupidPathTrace();
  virtual void InitAllGeneratedDescriptorSets_CastSingleRay();
  virtual void InitAllGeneratedDescriptorSets_PackXY();

  virtual void UpdatePlainMembers(std::shared_ptr<vkfw::ICopyEngine> a_pCopyEngine);
  virtual void UpdateVectorMembers(std::shared_ptr<vkfw::ICopyEngine> a_pCopyEngine);


  struct StupidPathTrace_Data
  {
    VkBuffer hitBuffer = VK_NULL_HANDLE;
    size_t   hitOffset = 0;

    VkBuffer threadFlagsBuffer = VK_NULL_HANDLE;
    size_t   threadFlagsOffset = 0;

    VkBuffer rayDirAndFarBuffer = VK_NULL_HANDLE;
    size_t   rayDirAndFarOffset = 0;

    VkBuffer rayPosAndNearBuffer = VK_NULL_HANDLE;
    size_t   rayPosAndNearOffset = 0;

    VkBuffer accumThoroughputBuffer = VK_NULL_HANDLE;
    size_t   accumThoroughputOffset = 0;

    VkBuffer accumColorBuffer = VK_NULL_HANDLE;
    size_t   accumColorOffset = 0;

    VkBuffer in_pakedXYBuffer = VK_NULL_HANDLE;
    size_t   in_pakedXYOffset = 0;

    VkBuffer out_colorBuffer = VK_NULL_HANDLE;
    size_t   out_colorOffset = 0;

  } StupidPathTrace_local;

  struct CastSingleRay_Data
  {
    VkBuffer hitBuffer = VK_NULL_HANDLE;
    size_t   hitOffset = 0;

    VkBuffer threadFlagsBuffer = VK_NULL_HANDLE;
    size_t   threadFlagsOffset = 0;

    VkBuffer rayDirAndFarBuffer = VK_NULL_HANDLE;
    size_t   rayDirAndFarOffset = 0;

    VkBuffer rayPosAndNearBuffer = VK_NULL_HANDLE;
    size_t   rayPosAndNearOffset = 0;

    VkBuffer in_pakedXYBuffer = VK_NULL_HANDLE;
    size_t   in_pakedXYOffset = 0;

    VkBuffer out_colorBuffer = VK_NULL_HANDLE;
    size_t   out_colorOffset = 0;

  } CastSingleRay_local;

  struct PackXY_Data
  {
    VkBuffer out_pakedXYBuffer = VK_NULL_HANDLE;
    size_t   out_pakedXYOffset = 0;

  } PackXY_local;


  struct StdVectorMembersGPUData
  {
    VkBuffer m_randomGensBuffer = VK_NULL_HANDLE;
    size_t   m_randomGensOffset = 0;
    VkBuffer spheresMaterialsBuffer = VK_NULL_HANDLE;
    size_t   spheresMaterialsOffset = 0;
    VkBuffer m_vPos4fBuffer = VK_NULL_HANDLE;
    size_t   m_vPos4fOffset = 0;
    VkBuffer m_intervalsBuffer = VK_NULL_HANDLE;
    size_t   m_intervalsOffset = 0;
    VkBuffer m_vNorm4fBuffer = VK_NULL_HANDLE;
    size_t   m_vNorm4fOffset = 0;
    VkBuffer m_nodesBuffer = VK_NULL_HANDLE;
    size_t   m_nodesOffset = 0;
    VkBuffer m_indicesReorderedBuffer = VK_NULL_HANDLE;
    size_t   m_indicesReorderedOffset = 0;
    VkBuffer m_indicesBuffer = VK_NULL_HANDLE;
    size_t   m_indicesOffset = 0;
    VkBuffer m_matrixBuffer = VK_NULL_HANDLE;
    size_t   m_matrixBufferOffset = 0;
    VkDeviceMemory m_vecMem = VK_NULL_HANDLE;
  } m_vdata;

  std::vector<VkTransformMatrixKHR> m_instances;

  VkBuffer m_classDataBuffer = VK_NULL_HANDLE;
  VkDeviceMemory m_allMem    = VK_NULL_HANDLE;

  VkPipelineLayout      ContributeToImageLayout   = VK_NULL_HANDLE;
  VkPipeline            ContributeToImagePipeline = VK_NULL_HANDLE; 
  VkDescriptorSetLayout ContributeToImageDSLayout = VK_NULL_HANDLE;  
  VkDescriptorSetLayout CreateContributeToImageDSLayout();

  VkPipelineLayout      NextBounceLayout   = VK_NULL_HANDLE;
  VkPipeline            NextBouncePipeline = VK_NULL_HANDLE; 
  VkDescriptorSetLayout NextBounceDSLayout = VK_NULL_HANDLE;  
  VkDescriptorSetLayout CreateNextBounceDSLayout();

  VkPipelineLayout      InitAccumDataLayout   = VK_NULL_HANDLE;
  VkPipeline            InitAccumDataPipeline = VK_NULL_HANDLE; 
  VkDescriptorSetLayout InitAccumDataDSLayout = VK_NULL_HANDLE;  
  VkDescriptorSetLayout CreateInitAccumDataDSLayout();

  VkPipelineLayout      RayTraceLayout   = VK_NULL_HANDLE;
  VkPipeline            RayTracePipeline = VK_NULL_HANDLE; 
  VkDescriptorSetLayout RayTraceDSLayout = VK_NULL_HANDLE;  
  VkDescriptorSetLayout CreateRayTraceDSLayout();

  VkPipelineLayout      InitEyeRayLayout   = VK_NULL_HANDLE;
  VkPipeline            InitEyeRayPipeline = VK_NULL_HANDLE; 
  VkDescriptorSetLayout InitEyeRayDSLayout = VK_NULL_HANDLE;  
  VkDescriptorSetLayout CreateInitEyeRayDSLayout();

  VkPipelineLayout      GetMaterialColorLayout   = VK_NULL_HANDLE;
  VkPipeline            GetMaterialColorPipeline = VK_NULL_HANDLE; 
  VkDescriptorSetLayout GetMaterialColorDSLayout = VK_NULL_HANDLE;  
  VkDescriptorSetLayout CreateGetMaterialColorDSLayout();

  VkPipelineLayout      PackXYLayout   = VK_NULL_HANDLE;
  VkPipeline            PackXYPipeline = VK_NULL_HANDLE; 
  VkDescriptorSetLayout PackXYDSLayout = VK_NULL_HANDLE;  
  VkDescriptorSetLayout CreatePackXYDSLayout();


  VkPipelineLayout      copyKernelFloatLayout   = VK_NULL_HANDLE;
  VkPipeline            copyKernelFloatPipeline = VK_NULL_HANDLE;
  VkDescriptorSetLayout copyKernelFloatDSLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout CreatecopyKernelFloatDSLayout();

  VkDescriptorPool m_dsPool = VK_NULL_HANDLE;
  VkDescriptorSet m_allGeneratedDS[9];
  uint32_t m_blockSize[3];
  std::unordered_map<std::string, KernelConfig> m_kernelExceptions;

  TestClass_UBO_Data m_uboData;
  
};

#endif

