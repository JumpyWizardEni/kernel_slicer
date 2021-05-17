#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <cassert>
#include <cmath>


void Denoise_cpu(const int w, const int h, const float* a_hdrData, int32_t* a_inTexColor, const int32_t* a_inNormal, const float* a_inDepth, 
                 const int a_windowRadius, const int a_blockRadius, const float a_noiseLevel, const char* a_outName);

void Denoise_gpu(const int w, const int h, const float* a_hdrData, int32_t* a_inTexColor, const int32_t* a_inNormal, const float* a_inDepth, 
                 const int a_windowRadius, const int a_blockRadius, const float a_noiseLevel, const char* a_outName);

bool LoadHDRImageFromFile(const char* a_fileName, int* pW, int* pH, std::vector<float>& a_data);   // defined in imageutils.cpp
bool LoadLDRImageFromFile(const char* a_fileName, int* pW, int* pH, std::vector<int32_t>& a_data); // defined in imageutils.cpp



int main(int argc, const char** argv)
{
  std::vector<float>   hdrData;
  std::vector<int32_t> texColor;
  std::vector<int32_t> normal;
  std::vector<float>   depth;

  int w, h, w2, h2, w3, h3, w4, h4;
  
  bool hasError = false;

  if(!LoadHDRImageFromFile("../images/WasteWhite_1024sample.hdr", &w, &h, hdrData))
  {
    std::cout << "can't open input file 'WasteWhite_1024sample.hdr' " << std::endl;
    hasError = true;
  }

  if(!LoadHDRImageFromFile("../images/WasteWhite_depth.hdr", &w2, &h2, depth))
  {
    std::cout << "can't open input file 'WasteWhite_depth.hdr' " << std::endl;
    hasError = true;
  }

  if(!LoadLDRImageFromFile("../images/WasteWhite_diffcolor.png", &w3, &h3, texColor))
  {
    std::cout << "can't open input file 'WasteWhite_diffcolor.png' " << std::endl;
    hasError = true;
  }

  if(!LoadLDRImageFromFile("../images/WasteWhite_normals.png", &w4, &h4, normal))
  {
    std::cout << "can't open input file 'WasteWhite_normals.png' " << std::endl;
    hasError = true;
  }


  if(w != w2 || h != h2)
  {
    std::cout << "size source image and depth pass not equal.' " << std::endl;
    hasError = true;
  }
  
  if(w != w3 || h != h3)
  {
    std::cout << "size source image and color pass not equal.' " << std::endl;
    hasError = true;
  }
  
  if(w != w4 || h != h4)
  {
    std::cout << "size source image and normal pass not equal.' " << std::endl;
    hasError = true;
  }

  if (hasError)
    return 0;


  uint64_t addressToCkeck = reinterpret_cast<uint64_t>(hdrData.data());
  assert(addressToCkeck % 16 == 0); // check if address is aligned!!!

  addressToCkeck = reinterpret_cast<uint64_t>(depth.data());
  assert(addressToCkeck % 16 == 0); // check if address is aligned!!!
  
  const int   windowRadius = 7;
  const int   blockRadius  = 3;
  const float noiseLevel   = 0.1F;

  Denoise_cpu(w, h, hdrData.data(), texColor.data(), normal.data(), depth.data(), windowRadius, blockRadius, noiseLevel, "zout_cpu.bmp");
  //Denoise_gpu(w, h, hdrData.data(), texColor.data(), normal.data(), depth.data(), windowRadius, blockRadius, noiseLevel, "zout_gpu.bmp");  
              
  return 0;
}