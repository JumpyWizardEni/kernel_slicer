#include "test_class.h"
#include "include/crandom.h"

using std::max;
using std::min;

void TestClass::InitRandomGens(int a_maxThreads)
{
  m_randomGens.resize(a_maxThreads);
  #pragma omp parallel for default(shared)
  for(int i=0;i<a_maxThreads;i++)
    m_randomGens[i] = RandomGenInit(i);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __private
#define __private
#endif

static inline float3 mul3x3(float4x4 m, float3 v)
{ 
  return to_float3(m*to_float4(v, 0.0f));
}

static inline float3 mul4x3(float4x4 m, float3 v)
{
  return to_float3(m*to_float4(v, 1.0f));
}

static inline void transform_ray3f(float4x4 a_mWorldViewInv, 
                                   __private float3* ray_pos, __private float3* ray_dir) 
{
  float3 pos  = mul4x3(a_mWorldViewInv, (*ray_pos));
  float3 pos2 = mul4x3(a_mWorldViewInv, ((*ray_pos) + 100.0f*(*ray_dir)));

  float3 diff = pos2 - pos;

  (*ray_pos)  = pos;
  (*ray_dir)  = normalize(diff);
}


void TestClass::kernel_InitEyeRay(uint tid, const uint* packedXY, float4* rayPosAndNear, float4* rayDirAndFar) // (tid,tidX,tidY,tidZ) are SPECIAL PREDEFINED NAMES!!!
{
  const uint XY = packedXY[tid];

  const uint x = (XY & 0x0000FFFF);
  const uint y = (XY & 0xFFFF0000) >> 16;

  float3 rayDir = EyeRayDir(x, y, WIN_WIDTH, WIN_HEIGHT, m_projInv); 
  float3 rayPos = float3(0,0,0);

  transform_ray3f(m_worldViewInv, 
                  &rayPos, &rayDir);
  
  *rayPosAndNear = to_float4(rayPos, 0.0f);
  *rayDirAndFar  = to_float4(rayDir, MAXFLOAT);
}

void TestClass::kernel_InitEyeRay2(uint tid, const uint* packedXY, float4* rayPosAndNear, float4* rayDirAndFar,
                                                                   float4* accumColor,    float4* accumuThoroughput) // (tid,tidX,tidY,tidZ) are SPECIAL PREDEFINED NAMES!!!
{
  *accumColor        = make_float4(0,0,0,0);
  *accumuThoroughput = make_float4(1,1,1,0);

  const uint XY = packedXY[tid];

  const uint x = (XY & 0x0000FFFF);
  const uint y = (XY & 0xFFFF0000) >> 16;

  float3 rayDir = EyeRayDir(x, y, WIN_WIDTH, WIN_HEIGHT, m_projInv); 
  float3 rayPos = float3(0,0,0);

  transform_ray3f(m_worldViewInv, 
                  &rayPos, &rayDir);
  
  *rayPosAndNear = to_float4(rayPos, 0.0f);
  *rayDirAndFar  = to_float4(rayDir, MAXFLOAT);
}


bool TestClass::kernel_RayTrace(uint tid, const float4* rayPosAndNear, float4* rayDirAndFar,
                                Lite_Hit* out_hit, float2* out_bars)
{
  const float4 rayPos = *rayPosAndNear;
  const float4 rayDir = *rayDirAndFar ;

  CRT_Hit hit = m_pAccelStruct->RayQuery_NearestHit(rayPos, rayDir);
  
  Lite_Hit res;
  res.primId = hit.primId;
  res.instId = hit.instId;
  res.geomId = hit.geomId;
  res.t      = hit.t;

  float2 baricentrics = float2(hit.coords[0], hit.coords[1]);
 
  *out_hit  = res;
  *out_bars = baricentrics;
  return (res.primId != -1);
}

void TestClass::kernel_PackXY(uint tidX, uint tidY, uint* out_pakedXY)
{
  out_pakedXY[pitchOffset(tidX,tidY)] = ((tidY << 16) & 0xFFFF0000) | (tidX & 0x0000FFFF);
}

void TestClass::kernel_RealColorToUint32(uint tid, float4* a_accumColor, uint* out_color)
{
  out_color[tid] = RealColorToUint32(*a_accumColor);
}

void TestClass::kernel_GetRayColor(uint tid, const Lite_Hit* in_hit, uint* out_color)
{ 
  const Lite_Hit lhit = *in_hit;
  if(lhit.geomId == -1)
  {
    out_color[tid] = 0;
    return;
  }

  //int selector = lhit.geomId;
  //if(selector % 8 == 0)
  //  out_color[tid] = 0x00A0A0A0;
  //else if(selector % 8 == 1)
  //  out_color[tid] = 0x00A00000;
  //else if(selector % 8 == 2)
  //  out_color[tid] = 0x0000A000;
  //else if(selector % 8 == 3)
  //  out_color[tid] = 0x000000A0;
  //else if(selector % 8 == 4)
  //  out_color[tid] = 0x00A0A000;
  //else if(selector % 8 == 5)
  //  out_color[tid] = 0x00A000A0;
  //else if(selector % 8 == 6)
  //  out_color[tid] = 0x0000A0A0;
  //else if(selector % 8 == 7)
  //  out_color[tid] = 0x00070707;

  const uint32_t matId = m_matIdByPrimId[m_matIdOffsets[lhit.geomId] + lhit.primId];
  const float4 mdata   = m_materials[matId];
  const float3 color = mdata.w > 0.0f ? clamp(float3(mdata.w,mdata.w,mdata.w), 0.0f, 1.0f) : to_float3(mdata);
  out_color[tid] = RealColorToUint32_f3(color); 
}

void TestClass::kernel_NextBounce(uint tid, const Lite_Hit* in_hit, const float2* in_bars, 
                                  float4* rayPosAndNear, float4* rayDirAndFar, float4* accumColor, float4* accumThoroughput)
{
  const Lite_Hit lhit = *in_hit;
  if(lhit.geomId == -1)
    *accumColor = float4(0,0,0,0);

  const uint32_t matId = m_matIdByPrimId[m_matIdOffsets[lhit.geomId] + lhit.primId];
  const float4 mdata   = m_materials[matId];

  // process light hit case
  //
  if(mdata.w > 0.0f)
  {
    const float lightIntensity = mdata.w;
    if(dot(to_float3(*rayDirAndFar), float3(0,-1,0)) < 0.0f)
      *accumColor = (*accumThoroughput)*lightIntensity;
    else
      *accumColor = float4(0,0,0,0);
    return;
  }
  
  // process surcase hit case
  //
  const float3 ray_dir = to_float3(*rayDirAndFar);
  
  SurfaceHit hit;
  hit.pos  = to_float3(*rayPosAndNear) + lhit.t*ray_dir;
  hit.norm = EvalSurfaceNormal(lhit.primId, lhit.geomId, *in_bars, m_matIdOffsets.data(), m_vertOffset.data(), m_triIndices.data(), m_vNorm4f.data());
  
  // transform surface point with matrix and flip normal if needed
  {
    hit.norm = normalize(mul3x3(m_normMatrices[lhit.instId], hit.norm));
    const float flipNorm = dot(ray_dir,  hit.norm) > 0.001f ? -1.0f : 1.0f;
    hit.norm = flipNorm*hit.norm;
  }

  RandomGen gen     = m_randomGens[tid];
  const float2 uv   = rndFloat2_Pseudo(&gen);
  m_randomGens[tid] = gen;

  const float3 newDir   = MapSampleToCosineDistribution(uv.x, uv.y, hit.norm, hit.norm, 1.0f);
  const float  cosTheta = dot(newDir, hit.norm);


  const float  pdfVal  = cosTheta * INV_PI;
  const float3 brdfVal = (cosTheta > 1e-5f) ? to_float3(mdata) * INV_PI : float3(0,0,0);
  const float3 bxdfVal = brdfVal * (1.0f / fmax(pdfVal, 1e-10f));
  
  *rayPosAndNear    = to_float4(OffsRayPos(hit.pos, hit.norm, newDir), 0.0f);
  *rayDirAndFar     = to_float4(newDir, MAXFLOAT);
  *accumThoroughput *= cosTheta*to_float4(bxdfVal, 0.0f); 
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TestClass::PackXY(uint tidX, uint tidY, uint* out_pakedXY)
{
  kernel_PackXY(tidX, tidY, out_pakedXY);
}

void TestClass::CastSingleRay(uint tid, uint* in_pakedXY, uint* out_color)
{
  float4 rayPosAndNear, rayDirAndFar;
  kernel_InitEyeRay(tid, in_pakedXY, &rayPosAndNear, &rayDirAndFar);

  Lite_Hit hit; 
  float2   baricentrics; 
  if(!kernel_RayTrace(tid, &rayPosAndNear, &rayDirAndFar, &hit, &baricentrics))
    return;
  
  kernel_GetRayColor(tid, &hit, out_color);
}

void TestClass::kernel_ContributeToImage(uint tid, const float4* a_accumColor, const uint* in_pakedXY, float4* out_color)
{
  const uint XY = in_pakedXY[tid];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;
 
  out_color[y*WIN_WIDTH+x] += *a_accumColor;
}

void TestClass::NaivePathTrace(uint tid, uint a_maxDepth, uint* in_pakedXY, float4* out_color)
{
  float4 accumColor, accumThoroughput;
  float4 rayPosAndNear, rayDirAndFar;
  kernel_InitEyeRay2(tid, in_pakedXY, &rayPosAndNear, &rayDirAndFar, &accumColor, &accumThoroughput);

  for(int depth = 0; depth < a_maxDepth; depth++) 
  {
    Lite_Hit hit;
    float2   baricentrics; 

    if(!kernel_RayTrace(tid, &rayPosAndNear, &rayDirAndFar, &hit, &baricentrics))
      break;
    
    kernel_NextBounce(tid, &hit, &baricentrics, 
                      &rayPosAndNear, &rayDirAndFar, &accumColor, &accumThoroughput);
  }

  kernel_ContributeToImage(tid, &accumColor, in_pakedXY, 
                           out_color);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Bitmap.h"

void test_class_cpu()
{
  TestClass test(WIN_WIDTH*WIN_HEIGHT);

  std::vector<uint32_t> pixelData(WIN_WIDTH*WIN_HEIGHT);
  std::vector<uint32_t> packedXY(WIN_WIDTH*WIN_HEIGHT);
  std::vector<float4>   realColor(WIN_WIDTH*WIN_HEIGHT);
  
  // remember pitch-linear (x,y) for each thread to make our threading 1D
  //
  for(int y=0;y<WIN_HEIGHT;y++)
  {
    for(int x=0;x<WIN_WIDTH;x++)
      test.PackXY(x, y, packedXY.data());
  }
  
  test.LoadScene("/home/frol/PROG/HydraRepos/HydraCore/hydra_app/tests/test_42/statex_00001.xml");
  
  // test simple ray casting
  //
  for(int i=0;i<WIN_HEIGHT*WIN_HEIGHT;i++)
    test.CastSingleRay(i, packedXY.data(), pixelData.data());

  SaveBMP("zout_cpu.bmp", pixelData.data(), WIN_WIDTH, WIN_HEIGHT);
  //return;

  // now test path tracing
  //
  const int PASS_NUMBER           = 100;
  const int ITERS_PER_PASS_NUMBER = 4;
  for(int passId = 0; passId < PASS_NUMBER; passId++)
  {
    #pragma omp parallel for default(shared)
    for(int i=0;i<WIN_HEIGHT*WIN_HEIGHT;i++)
    {
      for(int j=0;j<ITERS_PER_PASS_NUMBER;j++)
        test.NaivePathTrace(i, 6, packedXY.data(), realColor.data());
    }

    if(passId%10 == 0)
    {
      const float progress = 100.0f*float(passId)/float(PASS_NUMBER);
      std::cout << "progress = " << progress << "%   \r";
      std::cout.flush();
    }
  }
  
  //std::cout << std::endl;

  const float normConst = 1.0f/float(PASS_NUMBER*ITERS_PER_PASS_NUMBER);
  const float invGamma  = 1.0f / 2.2f;

  for(int i=0;i<WIN_HEIGHT*WIN_HEIGHT;i++)
  {
    float4 color = realColor[i]*normConst;
    color.x      = powf(color.x, invGamma);
    color.y      = powf(color.y, invGamma);
    color.z      = powf(color.z, invGamma);
    color.w      = 1.0f;
    pixelData[i] = RealColorToUint32(clamp(color, 0.0f, 1.0f));
  }
  SaveBMP("zout_cpu2.bmp", pixelData.data(), WIN_WIDTH, WIN_HEIGHT);
  

}
