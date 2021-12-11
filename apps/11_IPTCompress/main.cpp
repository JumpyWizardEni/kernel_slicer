#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <cstdint>
#include <cassert>

#include "test_class.h"
#include "Bitmap.h"

bool LoadHDRImageFromFile(const char* a_fileName, int* pW, int* pH, std::vector<float>& a_data); // defined in imageutils.cpp

#include "vk_context.h"
std::shared_ptr<ToneMapping> CreateToneMapping_Generated(vk_utils::VulkanContext a_ctx, size_t a_maxThreadsGenerated); 

int main(int argc, const char** argv)
{
  #ifndef NDEBUG
  bool enableValidationLayers = true;
  #else
  bool enableValidationLayers = false;
  #endif

  std::vector<float> hdrData;
  int w,h;
  
  if(!LoadHDRImageFromFile("../images/kitchen.hdr", &w, &h, hdrData))
  {
    std::cout << "can't open input file 'kitchen.hdr' " << std::endl;
    return 0;
  }

  uint64_t addressToCkeck = reinterpret_cast<uint64_t>(hdrData.data());
  assert(addressToCkeck % 16 == 0); // check if address is aligned!!!
  
  bool onGPU = true;
  std::shared_ptr<ToneMapping> pImpl = nullptr;
  if(onGPU)
  {
    auto ctx = vk_utils::globalContextGet(enableValidationLayers);
    pImpl    = CreateToneMapping_Generated(ctx, w*h);
  }
  else
    pImpl = std::make_shared<ToneMapping>();

  std::vector<uint> ldrData(w*h);
  pImpl->SetMaxImageSize(w,h);
  pImpl->CommitDeviceData();

  pImpl->IPTcompress(w,h, (const float4*)hdrData.data(), ldrData.data());
  
  if(onGPU)
    SaveBMP("zout_gpu.bmp", ldrData.data(), w, h);
  else
    SaveBMP("zout_cpu.bmp", ldrData.data(), w, h);
  
  float timings[4] = {0,0,0,0};
  pImpl->GetExecutionTime("IPTcompress", timings);
  std::cout << "IPTcompress(exec) = " << timings[0]              << " ms " << std::endl;
  std::cout << "IPTcompress(copy) = " << timings[1] + timings[2] << " ms " << std::endl;
  std::cout << "IPTcompress(ovrh) = " << timings[3]              << " ms " << std::endl; 
  return 0;
}
