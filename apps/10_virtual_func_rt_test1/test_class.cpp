#include "test_class.h"
#include "include/crandom.h"

void TestClass::InitRandomGens(int a_maxThreads)
{
  m_randomGens.resize(a_maxThreads);
  #pragma omp parallel for default(shared)
  for(int i=0;i<a_maxThreads;i++)
    m_randomGens[i] = RandomGenInit(i);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void TestClass::kernel_InitEyeRay(uint tid, const uint* packedXY, float4* rayPosAndNear, float4* rayDirAndFar) // (tid,tidX,tidY,tidZ) are SPECIAL PREDEFINED NAMES!!!
{
  const uint XY = packedXY[tid];

  const uint x = (XY & 0x0000FFFF);
  const uint y = (XY & 0xFFFF0000) >> 16;

  const float3 rayDir = EyeRayDir((float)x, (float)y, (float)WIN_WIDTH, (float)WIN_HEIGHT, m_worldViewProjInv); 
  const float3 rayPos = camPos;
  
  *rayPosAndNear = to_float4(rayPos, 0.0f);
  *rayDirAndFar  = to_float4(rayDir, MAXFLOAT);
}

static float2 RayBoxIntersectionLite(const float3 ray_pos, const float3 ray_dir_inv, const float boxMin[3], const float boxMax[3])
{
  const float lo = ray_dir_inv.x*(boxMin[0] - ray_pos.x);
  const float hi = ray_dir_inv.x*(boxMax[0] - ray_pos.x);

  float tmin = std::min(lo, hi);
  float tmax = std::max(lo, hi);

  const float lo1 = ray_dir_inv.y*(boxMin[1] - ray_pos.y);
  const float hi1 = ray_dir_inv.y*(boxMax[1] - ray_pos.y);

  tmin = std::max(tmin, std::min(lo1, hi1));
  tmax = std::min(tmax, std::max(lo1, hi1));

  const float lo2 = ray_dir_inv.z*(boxMin[2] - ray_pos.z);
  const float hi2 = ray_dir_inv.z*(boxMax[2] - ray_pos.z);

  tmin = std::max(tmin, std::min(lo2, hi2));
  tmax = std::min(tmax, std::max(lo2, hi2));

  return make_float2(tmin, tmax); //(tmin <= tmax) && (tmax > 0.f);
}

static void IntersectAllPrimitivesInLeaf(const float4 rayPosAndNear, const float4 rayDirAndFar,
                                         __global const uint* a_indices, uint a_start, uint a_count, __global const float4* a_vert,
                                         Lite_Hit* pHit)
{
  const uint triAddressEnd = a_start + a_count;
  for (uint triAddress = a_start; triAddress < triAddressEnd; triAddress = triAddress + 3u)
  {
    const uint A = a_indices[triAddress + 0];
    const uint B = a_indices[triAddress + 1];
    const uint C = a_indices[triAddress + 2];

    const float4 A_pos = a_vert[A];
    const float4 B_pos = a_vert[B];
    const float4 C_pos = a_vert[C];

    const float4 edge1 = B_pos - A_pos;
    const float4 edge2 = C_pos - A_pos;
    const float4 pvec  = cross(rayDirAndFar, edge2);
    const float4 tvec  = rayPosAndNear - A_pos;
    const float4 qvec  = cross(tvec, edge1);
    const float dotTmp = dot(to_float3(edge1), to_float3(pvec));
    const float invDet = 1.0f / (dotTmp > 1e-6f ? dotTmp : 1e-6f);

    const float v = dot(to_float3(tvec), to_float3(pvec))*invDet;
    const float u = dot(to_float3(qvec), to_float3(rayDirAndFar))*invDet;
    const float t = dot(to_float3(edge2), to_float3(qvec))*invDet;

    if (v > -1e-6f && u > -1e-6f && (u + v < 1.0f + 1e-6f) && t > rayPosAndNear.w && t < pHit->t)
    {
      pHit->t      = t;
      pHit->primId = triAddress/3;
    }
  }

}

static inline float3 SafeInverse_4to3(float4 d)
{
  const float ooeps = 1.0e-36f; // Avoid div by zero.
  float3 res;
  res.x = 1.0f / (fabs(d.x) > ooeps ? d.x : copysign(ooeps, d.x));
  res.y = 1.0f / (fabs(d.y) > ooeps ? d.y : copysign(ooeps, d.y));
  res.z = 1.0f / (fabs(d.z) > ooeps ? d.z : copysign(ooeps, d.z));
  return res;
}

bool TestClass::kernel_RayTrace(uint tid, const float4* rayPosAndNear, float4* rayDirAndFar,
                                const uint* indicesReordered, const float4* meshVerts,
                                Lite_Hit* out_hit)
{
  const float4 rayPos = *rayPosAndNear;
  const float4 rayDir = *rayDirAndFar ;

  const float3 rayDirInv = SafeInverse_4to3(rayDir);

  Lite_Hit res;
  res.primId = -1;
  res.instId = -1;
  res.geomId = -1;
  res.t      = rayDir.w;

  uint nodeIdx = 0;
  while(nodeIdx < 0xFFFFFFFE)
  {
    const struct BVHNode currNode = m_nodes[nodeIdx];
    const float2 boxHit           = RayBoxIntersectionLite(to_float3(rayPos), rayDirInv, currNode.boxMin, currNode.boxMax);
    const bool   intersects       = (boxHit.x <= boxHit.y) && (boxHit.y > rayPos.w) && (boxHit.x < res.t); // (tmin <= tmax) && (tmax > 0.f) && (tmin < curr_t)

    if(intersects && currNode.leftOffset == 0xFFFFFFFF) //leaf
    {
      struct Interval startCount = m_intervals[nodeIdx];
      IntersectAllPrimitivesInLeaf(rayPos, rayDir, indicesReordered, startCount.start*3, startCount.count*3, meshVerts, 
                                   &res);
    }

    nodeIdx = (currNode.leftOffset == 0xFFFFFFFF || !intersects) ? currNode.escapeIndex : currNode.leftOffset;
    nodeIdx = (nodeIdx == 0) ? 0xFFFFFFFE : nodeIdx;
  }
  
  *out_hit = res;
  return (res.primId != -1);
}

void TestClass::kernel_PackXY(uint tidX, uint tidY, uint* out_pakedXY)
{
  out_pakedXY[pitchOffset(tidX,tidY)] = ((tidY << 16) & 0xFFFF0000) | (tidX & 0x0000FFFF);
}

void TestClass::kernel_InitAccumData(uint tid, float4* accumColor, float4* accumuThoroughput)
{
  *accumColor        = make_float4(0,0,0,0);
  *accumuThoroughput = make_float4(1,1,1,0);
}

void TestClass::kernel_RealColorToUint32(uint tid, float4* a_accumColor, uint* out_color)
{
  out_color[tid] = RealColorToUint32(*a_accumColor);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

IMaterial* MakeObjPtr(const uint32_t objectPtr, __global const uint32_t* a_data)
{
  const uint32_t objectOffset = (objectPtr & IMaterial::OFS_MASK);
  const uint32_t objectTag    = (objectPtr & IMaterial::TAG_MASK) >> (32 - IMaterial::TAG_BITS);
  return (__global IMaterial*)(a_data + objectOffset);
}

IMaterial* TestClass::kernel_MakeMaterial(uint tid, const Lite_Hit* in_hit)
{
  uint32_t objPtr = 0;
  int primId = in_hit->primId;
  if(primId != -1)
  {
    const uint32_t mtId = m_materialIds[primId];
    objPtr = m_materialOffsets[mtId];
  }
  return MakeObjPtr(objPtr, m_materialData.data());
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
  if(!kernel_RayTrace(tid, &rayPosAndNear, &rayDirAndFar, m_indicesReordered.data(), m_vPos4f.data(), 
                      &hit))
    return;
  
  IMaterial* pMaterial = kernel_MakeMaterial(tid, &hit);

  pMaterial->kernel_GetColor(tid, out_color);
}

//void TestClass::StupidPathTrace(uint tid, uint a_maxDepth, uint* in_pakedXY, float4* out_color)
//{
//  float4 accumColor, accumThoroughput;
//  kernel_InitAccumData(tid, &accumColor, &accumThoroughput);
//
//  float4 rayPosAndNear, rayDirAndFar;
//  kernel_InitEyeRay(tid, in_pakedXY, &rayPosAndNear, &rayDirAndFar);
//
//  for(int depth = 0; depth < a_maxDepth; depth++) 
//  {
//    Lite_Hit hit;
//    if(!kernel_RayTrace(tid, &rayPosAndNear, &rayDirAndFar, &hit, m_indicesReordered.data(), m_vPos4f.data()))
//      break;
//
//    kernel_NextBounce(tid, &hit, 
//                      &rayPosAndNear, &rayDirAndFar, &accumColor, &accumThoroughput);
//  }
//
//  kernel_ContributeToImage(tid, &accumColor, in_pakedXY, 
//                           out_color);
//}

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

  //test.LoadScene("lucy.bvh", "lucy.vsgf");
  test.LoadScene("cornell_collapsed.bvh", "cornell_collapsed.vsgf");
  // test simple ray casting
  //
  #pragma omp parallel for default(shared)
  for(int i=0;i<WIN_HEIGHT*WIN_HEIGHT;i++)
    test.CastSingleRay(i, packedXY.data(), pixelData.data());

  SaveBMP("zout_cpu.bmp", pixelData.data(), WIN_WIDTH, WIN_HEIGHT);
  //return;

  /*
  // now test path tracing
  //
  const int PASS_NUMBER = 100;
  const int ITERS_PER_PASS_NUMBER = 4;
  for(int passId = 0; passId < PASS_NUMBER; passId++)
  {
    #pragma omp parallel for default(shared)
    for(int i=0;i<WIN_HEIGHT*WIN_HEIGHT;i++)
    {
      for(int j=0;j<ITERS_PER_PASS_NUMBER;j++)
        test.StupidPathTrace(i, 6, packedXY.data(), realColor.data());
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
  */

  return;
}