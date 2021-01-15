#include <vector>
#include "test_class.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TestClass::kernel_InitEyeRay(uint* flags, float4* rayPosAndNear, float4* rayDirAndFar, uint tidX, uint tidY) // (tid,tidX,tidY,tidZ) are SPECIAL PREDEFINED NAMES!!!
{
  const float3 rayDir = EyeRayDir((float)tidX, (float)tidY, (float)WIN_WIDTH, (float)WIN_HEIGHT, m_worldViewProjInv); 
  const float3 rayPos = make_float3(0.0f, 0.0f, 0.0f);
  
  *(rayPosAndNear) = to_float4(rayPos, 0.0f);
  *(rayDirAndFar ) = to_float4(rayDir, MAXFLOAT);
  *flags           = 0;
}

void TestClass::kernel_RayTrace(const float4* rayPosAndNear, float4* rayDirAndFar, 
                                Lite_Hit* out_hit, uint tidX, uint tidY)
{
  *out_hit = RayTraceImpl(to_float3(*rayPosAndNear), to_float3(*rayDirAndFar));
}

void TestClass::kernel_TestColor(const Lite_Hit* in_hit, uint* out_color, uint tidX, uint tidY, uint sphereColor, uint backColor)
{
  float x = 2.0f;

  if(in_hit->primId != -1)
    out_color[pitchOffset(tidX,tidY)] = sphereColor;
  else
    out_color[pitchOffset(tidX,tidY)] = backColor;
}


void TestClass::MainFunc(uint tidX, uint tidY, uint* out_color)
{
  const uint32_t mySphereColor = 0x000000FF;
  
  float4 rayPosAndNear, rayDirAndFar;
  uint   flags;

  kernel_InitEyeRay(&flags, &rayPosAndNear, &rayDirAndFar, tidX, tidY);

  Lite_Hit hit;
  kernel_RayTrace(&rayPosAndNear, &rayDirAndFar, 
                  &hit, tidX, tidY);
  
  kernel_TestColor(&hit, out_color, tidX, tidY, mySphereColor, 0x00FF0000);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Bitmap.h"

void test_class_cpu()
{
  TestClass test;
  std::vector<uint> pixelData(WIN_WIDTH*WIN_HEIGHT);
  
  for(int y=0;y<WIN_HEIGHT;y++)
  {
    for(int x=0;x<WIN_WIDTH;x++)
      test.MainFunc(x,y,pixelData.data());
  }

  SaveBMP("zout_cpu.bmp", pixelData.data(), WIN_WIDTH, WIN_HEIGHT);

  //std::cout << resultingColor.x << " " << resultingColor.y << " " << resultingColor.z << std::endl;
  return;
}