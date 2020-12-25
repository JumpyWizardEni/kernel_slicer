#ifndef TEST_CLASS_H
#define TEST_CLASS_H

#include "include/BasicLogic.h" // We assume that all code that should pe passed to kernels will be just included both for CPU and OpenCL
#include "include/crandom.h"

#include <vector>
#include <iostream>
#include <fstream>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<float> createGaussKernelWeights1D_HDRImage(int size, float a_sigma)
{
  std::vector<float> gKernel;
  gKernel.resize(size);
  // set standard deviation to 1.0
  //
  float sigma = a_sigma;
  float s = 2.0f * sigma * sigma;
  // sum is for normalization
  float sum = 0.0;
  int halfSize = size / 2;
  for (int x = -halfSize; x <= halfSize; x++)
  {
    float r = sqrtf((float)(x*x));
    int index = x + halfSize;
    gKernel[index] = (exp(-(r) / s)) / (3.141592654f * s);
    sum += gKernel[index];
  }
  // normalize the Kernel
  for (int i = 0; i < size; ++i)
    gKernel[i] /= sum;
  return gKernel;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ToneMapping 
{
public:

  ToneMapping()
  {
    // init weights for gaussian blur
    //
    m_blurRadius    = 7;
    m_filterWeights = createGaussKernelWeights1D_HDRImage(m_blurRadius*2 + 1, 1.25f);
    m_gammaInv      = 1.0f / 2.2f;
  }
  
  void SetMaxImageSize(int w, int h);

  void Bloom(int w, int h, const float4* inData4f, unsigned int* outData1ui);
  
  void ExtractBrightPixels(int tidX, int tidY, const float4* inData4f, float4* a_brightPixels);
  void DownSample4x(int tidX, int tidY, const float4* a_daraFullRes, float4* a_dataSmallRes);
  void BlurX(int tidX, int tidY, const float4* a_dataIn, float4* a_dataOut);
  void BlurY(int tidX, int tidY, const float4* a_dataIn, float4* a_dataOut);
  void MixAndToneMap(int tidX, int tidY, const float4* inData4f, const float4* inBrightPixels, unsigned int* outData1ui);

protected:

  void kernel_ExtractBrightPixels(int tidX, int tidY, const float4* inData4f, float4* a_brightPixels);
  void kernel_DownSample4x(int x, int y, const float4* a_daraFullRes, float4* a_dataSmallRes);
  void kernel_BlurX(int tidX, int tidY, const float4* a_dataIn, float4* a_dataOut);
  void kernel_BlurY(int tidX, int tidY, const float4* a_dataIn, float4* a_dataOut);
  void kernel_MixAndToneMap(int tidX, int tidY, const float4* inData4f, const float4* inBrightPixels, unsigned int* outData1ui);

  std::vector<float4> m_brightPixels;
  std::vector<float4> m_downsampledImage;
  std::vector<float4> m_tempImage;
  std::vector<float>  m_filterWeights;
  int m_blurRadius;
  
  int m_width;
  int m_height;

  int m_widthSmall;
  int m_heightSmall;
  float m_gammaInv;

};

#endif