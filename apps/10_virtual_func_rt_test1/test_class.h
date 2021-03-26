#ifndef TEST_CLASS_H
#define TEST_CLASS_H

#include "include/BasicLogic.h" // We assume that all code that should pe passed to kernels will be just included both for CPU and OpenCL
#include "include/crandom.h"

#include <vector>
#include <iostream>
#include <fstream>

#include "include/bvh.h"

static inline float4x4 perspectiveMatrix(float fovy, float aspect, float zNear, float zFar)
{
  const float ymax = zNear * tanf(fovy * 3.14159265358979323846f / 360.0f);
  const float xmax = ymax * aspect;
  const float left = -xmax;
  const float right = +xmax;
  const float bottom = -ymax;
  const float top = +ymax;
  const float temp = 2.0f * zNear;
  const float temp2 = right - left;
  const float temp3 = top - bottom;
  const float temp4 = zFar - zNear;
  float4x4 res;
  res.m_col[0] = float4{ temp / temp2, 0.0f, 0.0f, 0.0f };
  res.m_col[1] = float4{ 0.0f, temp / temp3, 0.0f, 0.0f };
  res.m_col[2] = float4{ (right + left) / temp2,  (top + bottom) / temp3, (-zFar - zNear) / temp4, -1.0 };
  res.m_col[3] = float4{ 0.0f, 0.0f, (-temp * zFar) / temp4, 0.0f };
  return res;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct IMaterial
{
  static constexpr uint32_t TYPE_BITS    = 4;          // number bits for type encoding in index; 
  static constexpr uint32_t TYPE_ID_MASK = 0xF0000000; // mask which we can get from TYPE_BITS
  static constexpr uint32_t OBJ_ID_MASK  = 0x0FFFFFFF; // (32 - TYPE_BITS) is left for object/thread id.

  enum {TYPE_ID_LAMBERT    = 0, 
        TYPE_ID_MIRROR     = 1, 
        TYPE_ID_EMISSIVE   = 2, 
        TYPE_ID_GGX_GLOSSY = 3};

  IMaterial(){}
  virtual ~IMaterial() {}

  virtual uint32_t GetTypeId() const = 0;
  virtual size_t   GetSizeOf() const = 0;

  virtual void   kernel_GetColor(uint tid, uint* out_color) = 0;

  virtual void   kernel_SampleNextBounce(uint tid, const Lite_Hit* in_hit, 
                                         float4* rayPosAndNear, float4* rayDirAndFar, float4* accumColor, float4* accumThoroughput) {}
};

struct LambertMaterial : public IMaterial
{
  LambertMaterial(float3 a_color) { m_color[0] = a_color[0]; m_color[1] = a_color[1]; m_color[2] = a_color[2]; }
  float m_color[3];

  void  kernel_GetColor(uint tid, uint* out_color) override 
  { 
    out_color[tid] = RealColorToUint32_f3(float3(m_color[0], m_color[1], m_color[2])); 
  }
  
  uint32_t GetTypeId() const override { return TYPE_ID_LAMBERT; }
  size_t   GetSizeOf() const override { return sizeof(LambertMaterial); }
};

struct PerfectMirrorMaterial : public IMaterial
{
  void kernel_GetColor(uint tid, uint* out_color) override 
  { 
    out_color[tid] = RealColorToUint32_f3(float3(0,0,0)); 
  }
  uint32_t GetTypeId() const override { return TYPE_ID_MIRROR; }
  size_t   GetSizeOf() const override { return sizeof(PerfectMirrorMaterial); }
};

struct EmissiveMaterial : public IMaterial
{
  void   kernel_GetColor(uint tid, uint* out_color) override 
  { 
    out_color[tid] = RealColorToUint32_f3(intensity*float3(1,1,1)); 
  }

  float  intensity;
  uint32_t GetTypeId() const override { return TYPE_ID_EMISSIVE; }
  size_t   GetSizeOf() const override { return sizeof(EmissiveMaterial); }
};

struct GGXGlossyMaterial : public IMaterial
{
  GGXGlossyMaterial(float3 a_color) { color[0] = a_color[0]; color[1] = a_color[1]; color[2] = a_color[2]; roughness = 0.5f; }
  void  kernel_GetColor(uint tid, uint* out_color) override 
  { 
    out_color[tid] = RealColorToUint32_f3(float3(color[0], color[1], color[2])); 
  }

  float color[3];
  float roughness;
  uint32_t GetTypeId() const override { return TYPE_ID_GGX_GLOSSY; }
  size_t   GetSizeOf() const override { return sizeof(GGXGlossyMaterial); }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class TestClass // : public DataClass
{
public:

  TestClass(int a_maxThreads = 1)
  {
    const float4x4 proj = perspectiveMatrix(45.0f, 1.0f, 0.01f, 100.0f);
    m_worldViewProjInv  = inverse4x4(proj);
    InitSpheresScene(10);
    InitRandomGens(a_maxThreads);
  }

  void InitRandomGens(int a_maxThreads);
  int LoadScene(const char* bvhPath, const char* meshPath);

  void PackXY(uint tidX, uint tidY, uint* out_pakedXY);
  void CastSingleRay(uint tid, uint* in_pakedXY, uint* out_color);
  void StupidPathTrace(uint tid, uint a_maxDepth, uint* in_pakedXY, float4* out_color);

  void kernel_PackXY(uint tidX, uint tidY, uint* out_pakedXY);

  void kernel_InitEyeRay(uint tid, const uint* packedXY, float4* rayPosAndNear, float4* rayDirAndFar);        // (tid,tidX,tidY,tidZ) are SPECIAL PREDEFINED NAMES!!!

  bool kernel_RayTrace(uint tid, const float4* rayPosAndNear, float4* rayDirAndFar,
                       Lite_Hit* out_hit, const uint* indicesReordered, const float4* meshVerts);
  
  void kernel_GetMaterialColor(uint tid, const Lite_Hit* in_hit, 
                               uint* out_color);

  void kernel_InitAccumData(uint tid, float4* accumColor, float4* accumuThoroughput);
  
  void kernel_NextBounce(uint tid, const Lite_Hit* in_hit, 
                         float4* rayPosAndNear, float4* rayDirAndFar, float4* accumColor, float4* accumThoroughput);

  void kernel_RealColorToUint32(uint tid, float4* a_accumColor, uint* out_color);

  void kernel_ContributeToImage(uint tid, const float4* a_accumColor, const uint* in_pakedXY, 
                                float4* out_color);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  IMaterial* kernel_MakeMaterial(uint tid, const Lite_Hit* in_hit);

protected:
  float3 camPos = float3(0.0f, 0.85f, 4.5f);
  void InitSpheresScene(int a_numSpheres, int a_seed = 0);

  uint32_t PackObject(uint32_t*& pData, IMaterial* a_pObject);

//  BVHTree                      m_bvhTree;
  std::vector<struct BVHNode>  m_nodes;
  std::vector<struct Interval> m_intervals;
  std::vector<uint32_t>        m_indicesReordered;
  std::vector<uint32_t>        m_materialIds;
  std::vector<float4>          m_vPos4f;      // copy from m_mesh
  std::vector<float4>          m_vNorm4f;     // copy from m_mesh

  std::vector<uint32_t>        m_materialData;
  std::vector<uint32_t>        m_materialOffsets;

  float4x4                     m_worldViewProjInv;
  std::vector<SphereMaterial>  spheresMaterials;
  std::vector<RandomGen>       m_randomGens;
};

#endif