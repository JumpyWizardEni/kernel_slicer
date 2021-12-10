#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <cstdint>

#include "test_class.h"

#include "vk_context.h"
std::shared_ptr<Numbers> CreateNumbers_Generated(vk_utils::VulkanContext a_ctx, size_t a_maxThreadsGenerated); 

int main(int argc, const char** argv)
{
  #ifndef NDEBUG
  bool enableValidationLayers = true;
  #else
  bool enableValidationLayers = false;
  #endif

  std::vector<int32_t> array(1024);
  for(size_t i=0;i<array.size();i++)
  {
    if(i%3 == 0)
      array[i] = i;
    else
      array[i] = -i;
  }
  
  bool isGPU = true;
  std::shared_ptr<Numbers> pImpl = nullptr;
  if(isGPU)
  {
    auto ctx = vk_utils::globalContextGet(enableValidationLayers);
    pImpl = CreateNumbers_Generated(ctx, array.size());
  }
  else
    pImpl = std::make_shared<Numbers>();

  pImpl->CalcArraySumm(array.data(), unsigned(array.size()));

  if(isGPU)
    std::cout << "[gpu]: array summ  = " << pImpl->m_summ << std::endl;
  else
    std::cout << "[cpu]: array summ  = " << pImpl->m_summ << std::endl;
  
  float timings[4] = {0,0,0,0};
  pImpl->GetExecutionTime("CalcArraySumm", timings);
  std::cout << "CalcArraySumm(exec) = " << timings[0]              << " ms " << std::endl;
  std::cout << "CalcArraySumm(copy) = " << timings[1] + timings[2] << " ms " << std::endl;
  std::cout << "CalcArraySumm(ovrh) = " << timings[3]              << " ms " << std::endl;
  return 0;
}
