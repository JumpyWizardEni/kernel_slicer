#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <cstdint>
#include <cassert>

#include "test_class.h"
#include "Bitmap.h"

#define MEASURE_TIME
#ifdef MEASURE_TIME
#include "test_class_generated.h"
#endif

bool LoadHDRImageFromFile(const char* a_fileName, 
                          int* pW, int* pH, std::vector<float>& a_data); // defined in imageutils.cpp

std::shared_ptr<ToneMapping> CreateToneMapping_Generated();

int main(int argc, const char** argv)
{
  std::vector<float> hdrData;
  int w,h;
  if(!LoadHDRImageFromFile("../images/nancy_church_2.hdr", &w, &h, hdrData))
  {
    std::cout << "can't open input file 'nancy_church_2.hdr' " << std::endl;
    return 0;
  }
  std::vector<uint> ldrData(w*h);

  uint64_t addressToCkeck = reinterpret_cast<uint64_t>(hdrData.data());
  assert(addressToCkeck % 16 == 0); // check if address is aligned!!!
  
  std::shared_ptr<ToneMapping> pImpl = nullptr;
  bool onGPU = true;
  
  if(onGPU)
    pImpl = CreateToneMapping_Generated();
  else
    pImpl = std::make_shared<ToneMapping>();

  pImpl->SetMaxImageSize(w,h);
  pImpl->Bloom(w, h, (const LiteMath::float4*)hdrData.data(), ldrData.data());

  if(onGPU)
    SaveBMP("zout_gpu.bmp", ldrData.data(), w, h);
  else
    SaveBMP("zout_cpu.bmp", ldrData.data(), w, h);
  
  #ifdef MEASURE_TIME
  ToneMapping_Generated* pGPUImpl = dynamic_cast<ToneMapping_Generated*>(pImpl.get());
  if(pGPUImpl != nullptr)
  {
    auto timings = pGPUImpl->GetBloomExecutionTime();
    std::cout << "Bloom(exec) = " << timings.msExecuteOnGPU                      << " ms " << std::endl;
    std::cout << "Bloom(copy) = " << timings.msCopyToGPU + timings.msCopyFromGPU << " ms " << std::endl;
  }
  #endif
  pImpl = nullptr;  
  return 0;
}
