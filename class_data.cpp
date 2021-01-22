#include "kslicer.h"

static inline size_t Padding(size_t a_size, size_t a_alignment)
{
  if (a_size % a_alignment == 0)
    return a_size;
  else
  {
    size_t sizeCut = a_size - (a_size % a_alignment);
    return sizeCut + a_alignment;
  }
}

struct less_than_key
{
  inline bool operator() (const kslicer::DataMemberInfo& struct1, const kslicer::DataMemberInfo& struct2)
  {
    if(struct1.sizeInBytes != struct2.sizeInBytes)
      return (struct1.sizeInBytes > struct2.sizeInBytes);
    else if(struct1.isContainerInfo && !struct2.isContainerInfo)
      return false;
    else if(!struct1.isContainerInfo && struct2.isContainerInfo)
      return true;
    else
      return struct1.name < struct2.name;
  }
};

static inline size_t AlignedSize(const size_t a_size)
{
  size_t currSize = 4;
  while(a_size > currSize)
    currSize = currSize*2;
  return currSize;
}

std::vector<kslicer::DataMemberInfo> kslicer::MakeClassDataListAndCalcOffsets(std::unordered_map<std::string, DataMemberInfo>& a_vars)
{
  std::vector<kslicer::DataMemberInfo> resVars;
  resVars.reserve(a_vars.size());
  
  // (1) get variables and sort them by size
  //
  for(const auto& keyval : a_vars)
  {
    if(!keyval.second.isContainer && keyval.second.usedInKernel)
      resVars.push_back(keyval.second);
    else if((keyval.second.usedInKernel || keyval.second.usedInMainFn) && keyval.second.isContainer)
    {
      kslicer::DataMemberInfo size;
      size.type         = "unsigned int";
      size.sizeInBytes  = sizeof(unsigned int);
      size.name         = keyval.second.name + "_size";
      size.usedInKernel = true;
      size.isContainerInfo = true;
      kslicer::DataMemberInfo capacity = size;
      capacity.name     = keyval.second.name + "_capacity";

      resVars.push_back(size);
      resVars.push_back(capacity);
    }
  }

  std::sort(resVars.begin(), resVars.end(), less_than_key());

  // (2) now assign correct offsets taking in mind align
  //
  size_t offsetInBytes = 0;
  for(auto& var : resVars)
  {
    const size_t alignedSize = AlignedSize(var.sizeInBytes);
    var.offsetInTargetBuffer = offsetInBytes;
    var.alignedSizeInBytes   = alignedSize;
    assert(var.offsetInTargetBuffer % alignedSize == 0); // this is guaranteed due to we sort variables by their size
    offsetInBytes += alignedSize;
  }

  for(const auto& keyval : a_vars)
  {
    if((keyval.second.usedInKernel || keyval.second.usedInMainFn) && keyval.second.isContainer)
      resVars.push_back(keyval.second);
  }

  return resVars;
}