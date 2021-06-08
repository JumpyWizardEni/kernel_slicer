#include "test_class.h"
#include "Bitmap.h"
#include <cassert>
#include <algorithm>
#include <array>
#include <chrono>

using namespace LiteMath;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline float sqr(float x) { return x * x; }

void SphHarm::kernel2D_IntegrateSphHarm(uint32_t* a_data, uint32_t a_width, uint32_t a_height)
{
  coefs[0] = float3(0,0,0); 
  coefs[1] = float3(0,0,0); 
  coefs[2] = float3(0,0,0); 
  coefs[3] = float3(0,0,0); 
  coefs[4] = float3(0,0,0); 
  coefs[5] = float3(0,0,0); 
  coefs[6] = float3(0,0,0); 
  coefs[7] = float3(0,0,0); 
  coefs[8] = float3(0,0,0); 

  //Iterate over height
  //#pragma omp parallel for
  for (uint32_t i = 0; i < a_height; ++i) {
    //Iterate over width
    for (uint32_t j = 0; j < a_width; ++j) {
      //Create a direction to texel
      const float phi = (i + 0.5f) / a_height * PI * 2.0f;
      const float theta = 2.0f * std::acos(std::sqrt(1.0f - (j + 0.5f) / a_width));
      const float3 direction = float3(std::cos(phi) * std::sin(theta), std::sin(phi) * std::sin(theta), std::cos(theta));
      //Extract color
      const uint32_t texelIdx = i * a_width + j;
      const uint32_t texelData = a_data[texelIdx];
      const float3 color = float3(texelData & 0xFF, (texelData >> 8) & 0xFF, (texelData >> 16) & 0xFF) / 255.0f;
      //Add new coefficients to integration
      coefs[0] += color;
      coefs[1] += color * direction.y;
      coefs[2] += color * direction.z;
      coefs[3] += color * direction.x;
      coefs[4] += color * direction.x * direction.y;
      coefs[5] += color * direction.y * direction.z;
      coefs[6] += color * (2 * direction.z * direction.z - direction.x * direction.x - direction.y * direction.y);
      coefs[7] += color * direction.x * direction.z;
      coefs[8] += color * (direction.x * direction.x - direction.y * direction.y);
    }
  }
  
  //Finalize coefficients
  //
  const float normConst = 4.0f * PI/(float)(a_height * a_width);

  coefs[0] *= normConst*0.5f / std::sqrt(PI);
  coefs[1] *= normConst*-0.5f * std::sqrt(3.0f / PI);
  coefs[2] *= normConst*0.5f * std::sqrt(3.0f / PI);
  coefs[3] *= normConst*-0.5f * std::sqrt(3.0f / PI);
  coefs[4] *= normConst*0.5f * std::sqrt(15.0f / PI);
  coefs[5] *= normConst*-0.5f * std::sqrt(15.0f / PI);
  coefs[6] *= normConst*0.25f * std::sqrt(5.0f / PI);
  coefs[7] *= normConst*-0.5f * std::sqrt(15.0f / PI);
  coefs[8] *= normConst*0.25f * std::sqrt(15.0f / PI);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SphHarm::ProcessPixels(uint32_t* a_data, uint32_t a_width, uint32_t a_height)
{
  kernel2D_IntegrateSphHarm(a_data, a_width, a_height);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::array<LiteMath::float3, 9> process_image_cpu(std::vector<uint32_t>& a_inPixels, uint32_t a_width, uint32_t a_height)
{
  SphHarm integrator;
  std::array<LiteMath::float3, 9> resCoeffs;
  
  for(int i=0;i<10;i++) {
  auto start = std::chrono::high_resolution_clock::now();
  integrator.ProcessPixels(a_inPixels.data(), a_width, a_height);
  
  auto stop = std::chrono::high_resolution_clock::now();
  auto ms   = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  std::cout << ms << " ms for 'ProcessPixels' " << std::endl;
  }
  integrator.GetCoefficients(resCoeffs.data());
  return resCoeffs;
}
