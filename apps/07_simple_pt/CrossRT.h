#pragma once

#include <cstdint>
#include <cstddef>

#define LAYOUT_STD140
#include "LiteMath.h"

/**
\brief API to ray-scene intersection on CPU
*/
struct CRT_Hit 
{
  float    t;         ///< intersection distance from ray origin to object
  uint32_t primId; 
  uint32_t instId;
  uint32_t geomId;    ///< use 4 most significant bits for geometry type; thay are zero for triangles 
  float    coords[4]; ///< custom intersection data; for triangles coords[0] and coords[1] stores baricentric coords (u,v)
};

/**
\brief API to ray-scene intersection on CPU
*/
struct ISceneObject
{
  ISceneObject(){}
  virtual ~ISceneObject(){} 

  /**
  \brief clear everything 
  */
  virtual void ClearGeom() = 0; 
  
  /**
  \brief Add geometry opf type 'Triangles' to 'internal geometry library' of scene object and return geometry id
  \param a_vpos4f     - input vertex data; each vertex should be of 4 floats, the fourth coordinate is not used
  \param a_vertNumber - vertices number. The total size of 'a_vpos4f' array is assumed to be qual to 4*a_vertNumber
  \param a_triIndices - triangle indices (standart index buffer)
  \param a_indNumber  - number of indices, shiuld be equal to 3*triaglesNum in your mesh
  \return id of added geometry
  */
  virtual uint32_t AddGeom_Triangles4f(const LiteMath::float4* a_vpos4f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber) = 0;
  
  /**
  \brief Update geometry for triangle mesh to 'internal geometry library' of scene object and return geometry id
  \param a_geomId - geometry id that should be updated. Please refer to 'AddGeom_Triangles4f' for other parameters
  
  Updates geometry. Please note that you can't: 
   * change geometry type with this fuction (from 'Triangles' to 'Spheres' for examples). 
   * increase geometry size (no 'a_vertNumber', neither 'a_indNumber') with this fuction (but it is allowed to make it smaller than original geometry size which was set by 'AddGeom_Triangles4f')
  So if you added 'Triangles' and got geom_id == 3, than you will have triangle mesh on geom_id == 3 forever and with the size.
  */
  virtual void UpdateGeom_Triangles4f(uint32_t a_geomId, const LiteMath::float4* a_vpos4f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber) = 0;
  
  /**
  \brief Vlear all instances, but don't touch geometry
  */
  virtual void BeginScene() = 0; ///< 

  /**
  \brief Vinish instancing and build top accaleration structure
  */
  virtual void EndScene  () = 0; ///< 
  
  /**
  \brief Add instance to scene
  \param a_geomId     - input if of geometry that is supposed to be instanced
  \param a_matrixData - float4x4 matrix, the default layout is column-major
  \param a_rowMajor   - flag that allow to change matrix layout from column-major to row-major

  */
  virtual uint32_t InstanceGeom(uint32_t a_geomId, const float a_matrixData[16], bool a_rowMajor = false) = 0;
  
  /**
  \brief Add instance to scene
  \param a_instanceId
  \param a_geomId     - input if of geometry that is supposed to be instanced
  \param a_matrixData - float4x4 matrix, the default layout is column-major
  \param a_rowMajor   - flag that allow to change matrix layout from column-major to row-major
  */
  virtual void     UpdateInstance(uint32_t a_instanceId, uint32_t a_geomId, const float* a_matrixData, bool a_rowMajor = false) = 0; 
 
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  virtual CRT_Hit RayQuery(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar) = 0;

};

ISceneObject* CreateSceneRT(const char* a_impleName); 
void          DeleteSceneRT(ISceneObject* a_pScene);
