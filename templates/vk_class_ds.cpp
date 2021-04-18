#include <vector>
#include <array>
#include <memory>
#include <limits>

#include <cassert>

#include "vulkan_basics.h"
#include "{{IncludeClassDecl}}"

void {{MainClassName}}_Generated::AllocateAllDescriptorSets()
{
  // allocate pool
  //
  VkDescriptorPoolSize buffersSize;
  buffersSize.type            = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  buffersSize.descriptorCount = {{TotalDSNumber}}*4 + 10; // mul 4 and add 10 because of AMD bug

  VkDescriptorPoolCreateInfo descriptorPoolCreateInfo = {};
  descriptorPoolCreateInfo.sType         = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  descriptorPoolCreateInfo.maxSets       = {{TotalDSNumber}} + 1; // add 1 to prevent zero case
  descriptorPoolCreateInfo.poolSizeCount = 1;
  descriptorPoolCreateInfo.pPoolSizes    = &buffersSize;
  
  VK_CHECK_RESULT(vkCreateDescriptorPool(device, &descriptorPoolCreateInfo, NULL, &m_dsPool));
  
  // allocate all descriptor sets
  //
  VkDescriptorSetLayout layouts[{{TotalDSNumber}}] = {};
## for DescriptorSet in DescriptorSetsAll
  layouts[{{DescriptorSet.Id}}] = {{DescriptorSet.Layout}};
## endfor

  VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
  descriptorSetAllocateInfo.sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  descriptorSetAllocateInfo.descriptorPool     = m_dsPool;  
  descriptorSetAllocateInfo.descriptorSetCount = {{TotalDSNumber}};     
  descriptorSetAllocateInfo.pSetLayouts        = layouts;

  auto tmpRes = vkAllocateDescriptorSets(device, &descriptorSetAllocateInfo, m_allGeneratedDS);
  VK_CHECK_RESULT(tmpRes); 
}

## for MainFunc in MainFunctions
void {{MainClassName}}_Generated::InitAllGeneratedDescriptorSets_{{MainFunc.Name}}()
{
  // now create actual bindings
  //
## for DescriptorSet in MainFunc.DescriptorSets
  // descriptor set #{{DescriptorSet.Id}}: {{DescriptorSet.KernelName}}Cmd ({{DescriptorSet.ArgNames}})
  {
    {% if not DescriptorSet.IsServiceCall and UseSeparateUBO %}
    constexpr uint additionalSize = 2;
    {% else if not DescriptorSet.IsServiceCall or UseSeparateUBO %}
    constexpr uint additionalSize = 1;
    {% else %}
    constexpr uint additionalSize = 0;
    {% endif %}

    std::array<VkDescriptorBufferInfo, {{DescriptorSet.ArgNumber}} + additionalSize> descriptorBufferInfo;
    std::array<VkWriteDescriptorSet,   {{DescriptorSet.ArgNumber}} + additionalSize> writeDescriptorSet;

## for Arg in DescriptorSet.Args
    descriptorBufferInfo[{{Arg.Id}}]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[{{Arg.Id}}].buffer = {{Arg.Name}}Buffer;
    descriptorBufferInfo[{{Arg.Id}}].offset = {{Arg.Name}}Offset;
    descriptorBufferInfo[{{Arg.Id}}].range  = VK_WHOLE_SIZE;  

    writeDescriptorSet[{{Arg.Id}}]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[{{Arg.Id}}].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[{{Arg.Id}}].dstSet           = m_allGeneratedDS[{{DescriptorSet.Id}}];
    writeDescriptorSet[{{Arg.Id}}].dstBinding       = {{Arg.Id}};
    writeDescriptorSet[{{Arg.Id}}].descriptorCount  = 1;
    writeDescriptorSet[{{Arg.Id}}].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[{{Arg.Id}}].pBufferInfo      = &descriptorBufferInfo[{{Arg.Id}}];
    writeDescriptorSet[{{Arg.Id}}].pImageInfo       = nullptr;
    writeDescriptorSet[{{Arg.Id}}].pTexelBufferView = nullptr; 

## endfor
    {% if not DescriptorSet.IsServiceCall %}
    descriptorBufferInfo[{{DescriptorSet.ArgNumber}}]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[{{DescriptorSet.ArgNumber}}].buffer = {% if DescriptorSet.IsVirtual %}FFFF{% else %}m_classDataBuffer{% endif %};
    descriptorBufferInfo[{{DescriptorSet.ArgNumber}}].offset = 0;
    descriptorBufferInfo[{{DescriptorSet.ArgNumber}}].range  = VK_WHOLE_SIZE;  

    writeDescriptorSet[{{DescriptorSet.ArgNumber}}]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[{{DescriptorSet.ArgNumber}}].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[{{DescriptorSet.ArgNumber}}].dstSet           = m_allGeneratedDS[{{DescriptorSet.Id}}];
    writeDescriptorSet[{{DescriptorSet.ArgNumber}}].dstBinding       = {{DescriptorSet.ArgNumber}};
    writeDescriptorSet[{{DescriptorSet.ArgNumber}}].descriptorCount  = 1;
    writeDescriptorSet[{{DescriptorSet.ArgNumber}}].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[{{DescriptorSet.ArgNumber}}].pBufferInfo      = &descriptorBufferInfo[{{DescriptorSet.ArgNumber}}];
    writeDescriptorSet[{{DescriptorSet.ArgNumber}}].pImageInfo       = nullptr;
    writeDescriptorSet[{{DescriptorSet.ArgNumber}}].pTexelBufferView = nullptr;
    {% endif %}
    {% if UseSeparateUBO %} 
    
    const size_t uboId = descriptorBufferInfo.size()-1;
    descriptorBufferInfo[uboId]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[uboId].buffer = m_uboArgsBuffer;
    descriptorBufferInfo[uboId].offset = 0;
    descriptorBufferInfo[uboId].range  = VK_WHOLE_SIZE;  

    writeDescriptorSet[uboId]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[uboId].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[uboId].dstSet           = m_allGeneratedDS[{{DescriptorSet.Id}}];
    writeDescriptorSet[uboId].dstBinding       = uboId;
    writeDescriptorSet[uboId].descriptorCount  = 1;
    writeDescriptorSet[uboId].descriptorType   = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writeDescriptorSet[uboId].pBufferInfo      = &descriptorBufferInfo[uboId];
    writeDescriptorSet[uboId].pImageInfo       = nullptr;
    writeDescriptorSet[uboId].pTexelBufferView = nullptr;
    {% endif %}
   
    vkUpdateDescriptorSets(device, uint32_t(writeDescriptorSet.size()), writeDescriptorSet.data(), 0, NULL);
  }
## endfor
}

## endfor

{% if length(IndirectDispatches) > 0 %}
void {{MainClassName}}_Generated::InitIndirectDescriptorSets()
{
  if(m_indirectUpdateDS != VK_NULL_HANDLE)
    return;

  // (m_classDataBuffer, m_indirectBuffer) ==> m_indirectUpdateDS
  //
  VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
  descriptorSetAllocateInfo.sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  descriptorSetAllocateInfo.descriptorPool     = m_dsPool;  
  descriptorSetAllocateInfo.descriptorSetCount = 1;     
  descriptorSetAllocateInfo.pSetLayouts        = &m_indirectUpdateDSLayout;
  
  auto tmpRes = vkAllocateDescriptorSets(device, &descriptorSetAllocateInfo, &m_indirectUpdateDS);
  VK_CHECK_RESULT(tmpRes); 

  VkDescriptorBufferInfo descriptorBufferInfo[2];
  VkWriteDescriptorSet   writeDescriptorSet[2];

  descriptorBufferInfo[0]        = VkDescriptorBufferInfo{};
  descriptorBufferInfo[0].buffer = m_classDataBuffer;
  descriptorBufferInfo[0].offset = 0;
  descriptorBufferInfo[0].range  = VK_WHOLE_SIZE;  

  descriptorBufferInfo[1]        = VkDescriptorBufferInfo{};
  descriptorBufferInfo[1].buffer = m_indirectBuffer;
  descriptorBufferInfo[1].offset = 0;
  descriptorBufferInfo[1].range  = VK_WHOLE_SIZE;  

  writeDescriptorSet[0]                  = VkWriteDescriptorSet{};
  writeDescriptorSet[0].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  writeDescriptorSet[0].dstSet           = m_indirectUpdateDS;
  writeDescriptorSet[0].dstBinding       = 0;
  writeDescriptorSet[0].descriptorCount  = 1;
  writeDescriptorSet[0].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  writeDescriptorSet[0].pBufferInfo      = &descriptorBufferInfo[0];
  writeDescriptorSet[0].pImageInfo       = nullptr;
  writeDescriptorSet[0].pTexelBufferView = nullptr; 

  writeDescriptorSet[1]                  = VkWriteDescriptorSet{};
  writeDescriptorSet[1].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  writeDescriptorSet[1].dstSet           = m_indirectUpdateDS;
  writeDescriptorSet[1].dstBinding       = 1;
  writeDescriptorSet[1].descriptorCount  = 1;
  writeDescriptorSet[1].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  writeDescriptorSet[1].pBufferInfo      = &descriptorBufferInfo[1];
  writeDescriptorSet[1].pImageInfo       = nullptr;
  writeDescriptorSet[1].pTexelBufferView = nullptr; 

  vkUpdateDescriptorSets(device, 2, writeDescriptorSet, 0, NULL);
}
{% endif %}
