#version 460
#extension GL_EXT_ray_tracing : enable
#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_GOOGLE_include_directive : require
#include "include/glsl_common.h"


layout(location = 0) rayPayloadEXT Lite_Hit hitValue;


void main()
{
  hitValue.primId = gl_PrimitiveID;
  hitValue.geomId = gl_GeometryIndexEXT;
  hitValue.instId = gl_InstanceID;
  hitValue.t      = gl_HitTEXT;
}
