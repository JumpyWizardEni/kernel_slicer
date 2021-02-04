/////////////////////////////////////////////////////////////////////
/////////////////// include files ///////////////////////////////////
/////////////////////////////////////////////////////////////////////

#include "include/bvh.h"
#include "include/crandom.h"
#include "include/BasicLogic.h"

/////////////////////////////////////////////////////////////////////
/////////////////// declarations in class ///////////////////////////
/////////////////////////////////////////////////////////////////////

#include "include/TestClass_ubo.h"

/////////////////////////////////////////////////////////////////////
/////////////////// local functions /////////////////////////////////
/////////////////////////////////////////////////////////////////////

static bool RayBoxIntersection(float3 ray_pos, float3 ray_dir, float3 boxMin, float3 boxMax, float tmin, float tmax)
{
  ray_dir.x = 1.0f/ray_dir.x;
  ray_dir.y = 1.0f/ray_dir.y;
  ray_dir.z = 1.0f/ray_dir.z;

  float lo = ray_dir.x*(boxMin.x - ray_pos.x);
  float hi = ray_dir.x*(boxMax.x - ray_pos.x);

  tmin = fmin(lo, hi);
  tmax = fmax(lo, hi);

  float lo1 = ray_dir.y*(boxMin.y - ray_pos.y);
  float hi1 = ray_dir.y*(boxMax.y - ray_pos.y);

  tmin = fmax(tmin, fmin(lo1, hi1));
  tmax = fmin(tmax, fmax(lo1, hi1));

  float lo2 = ray_dir.z*(boxMin.z - ray_pos.z);
  float hi2 = ray_dir.z*(boxMax.z - ray_pos.z);

  tmin = fmax(tmin, fmin(lo2, hi2));
  tmax = fmin(tmax, fmax(lo2, hi2));

  return (tmin <= tmax) && (tmax > 0.f);
}

uint fakeOffset(uint x, uint y, uint pitch) { return y*pitch + x; }                                      // for 2D threading

uint fakeOffset3(uint x, uint y, uint z, uint sizeY, uint sizeX) { return z*sizeY*sizeX + y*sizeX + x; } // for 3D threading

#define KGEN_FLAG_RETURN 1
#define KGEN_FLAG_BREAK  2
#define KGEN_FLAG_DONT_SET_EXIT 4
#define KGEN_FLAG_SET_EXIT_NEGATIVE 8

/////////////////////////////////////////////////////////////////////
/////////////////// kernels /////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

__kernel void kernel_ContributeToImage(
  __global const float4 * restrict a_accumColor,
  __global const uint * restrict in_pakedXY,
  __global float4 * restrict out_color,
  __global uint* restrict kgen_threadFlags,
   __global struct TestClass_UBO_Data* restrict ubo,
  const uint kgen_iNumElementsX, 
  const uint kgen_iNumElementsY,
  const uint kgen_iNumElementsZ,
  const uint kgen_tFlagsMask)
{
  /////////////////////////////////////////////////////////////////
  const uint tid = get_global_id(0); 
  if(tid >= kgen_iNumElementsX)
    return;
  if((kgen_threadFlags[tid] & kgen_tFlagsMask) != 0) 
    return;
  /////////////////////////////////////////////////////////////////

  const uint XY = in_pakedXY[tid];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;
 
  out_color[y*WIN_WIDTH+x] += a_accumColor[tid];

}

__kernel void kernel_InitAccumData(
  __global float4 * restrict accumColor,
  __global float4 * restrict accumuThoroughput,
  __global uint* restrict kgen_threadFlags,
   __global struct TestClass_UBO_Data* restrict ubo,
  const uint kgen_iNumElementsX, 
  const uint kgen_iNumElementsY,
  const uint kgen_iNumElementsZ,
  const uint kgen_tFlagsMask)
{
  /////////////////////////////////////////////////////////////////
  const uint tid = get_global_id(0); 
  if(tid >= kgen_iNumElementsX)
    return;
  if((kgen_threadFlags[tid] & kgen_tFlagsMask) != 0) 
    return;
  /////////////////////////////////////////////////////////////////

  accumColor[tid]        = make_float4(0,0,0,0);
  accumuThoroughput[tid] = make_float4(1,1,1,0);

}


__kernel void kernel_InitEyeRay(
  __global const uint * restrict packedXY,
  __global float4 * restrict rayPosAndNear,
  __global float4 * restrict rayDirAndFar,
  __global uint* restrict kgen_threadFlags,
   __global struct TestClass_UBO_Data* restrict ubo,
  const uint kgen_iNumElementsX, 
  const uint kgen_iNumElementsY,
  const uint kgen_iNumElementsZ,
  const uint kgen_tFlagsMask)
{
  /////////////////////////////////////////////////////////////////
  const uint tid = get_global_id(0); 
  if(tid >= kgen_iNumElementsX)
    return;
  if((kgen_threadFlags[tid] & kgen_tFlagsMask) != 0) 
    return;
  const float3 camPos = ubo->camPos;
  /////////////////////////////////////////////////////////////////

  const uint XY = packedXY[tid];

  const uint x = (XY & 0x0000FFFF);
  const uint y = (XY & 0xFFFF0000) >> 16;

  const float3 rayDir = EyeRayDir((float)x, (float)y, (float)WIN_WIDTH, (float)WIN_HEIGHT, ubo->m_worldViewProjInv); 
  const float3 rayPos = camPos;
  
  rayPosAndNear[tid] = to_float4(rayPos, 0.0f);
  rayDirAndFar[tid]  = to_float4(rayDir, MAXFLOAT);

}

__kernel void kernel_GetMaterialColor(
  __global const Lite_Hit * restrict in_hit,
  __global uint * restrict out_color,
  __global uint* restrict kgen_threadFlags,
  __global struct MaterialT* restrict spheresMaterials,
   __global struct TestClass_UBO_Data* restrict ubo,
  const uint kgen_iNumElementsX, 
  const uint kgen_iNumElementsY,
  const uint kgen_iNumElementsZ,
  const uint kgen_tFlagsMask)
{
  /////////////////////////////////////////////////////////////////
  const uint tid = get_global_id(0); 
  if(tid >= kgen_iNumElementsX)
    return;
  if((kgen_threadFlags[tid] & kgen_tFlagsMask) != 0) 
    return;
  const uint spheresMaterials_size = ubo->spheresMaterials_size; 
  /////////////////////////////////////////////////////////////////

  if(in_hit[tid].primId != -1)
  {
    out_color[tid] = RealColorToUint32_f3(to_float3(spheresMaterials[in_hit[tid].primId % 3].color));
  }
  else
    out_color[tid] = 0x00700000;

}

__kernel void kernel_PackXY(
  __global uint * restrict out_pakedXY,
   __global struct TestClass_UBO_Data* restrict ubo,
  const uint kgen_iNumElementsX, 
  const uint kgen_iNumElementsY,
  const uint kgen_iNumElementsZ,
  const uint kgen_tFlagsMask)
{
  /////////////////////////////////////////////////////////////////
  const uint tidX = get_global_id(0); 
  const uint tidY = get_global_id(1); 
  if(tidX >= kgen_iNumElementsX || tidY >= kgen_iNumElementsY)
    return;
  
  /////////////////////////////////////////////////////////////////

  out_pakedXY[pitchOffset(tidX,tidY)] = ((tidY << 16) & 0xFFFF0000) | (tidX & 0x0000FFFF);

}


__kernel void copyKernelFloat(
  __global float* restrict out_data,
  __global float* restrict in_data,
  const uint length)
{
  const uint i = get_global_id(0);
  if(i >= length)
    return;
  out_data[i] = in_data[i];
}

