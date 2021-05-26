#ifndef BASIC_PROJ_LOGIC_H
#define BASIC_PROJ_LOGIC_H

#include "OpenCLMath.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct Lite_HitT
{
  float t;
  int   primId; 
  int   instId;
  int   geomId;
} Lite_Hit;


typedef struct SurfaceHitT
{
  float3 pos;
  float3 norm;
}SurfaceHit;

typedef struct LightGeomT
{
  float3 boxMin;
  float3 boxMax;
} LightGeom;

static inline float3 EyeRayDir(float x, float y, float w, float h, float4x4 a_mViewProjInv) // g_mViewProjInv
{
  float4 pos = make_float4( 2.0f * (x + 0.5f) / w - 1.0f, 
                           -2.0f * (y + 0.5f) / h + 1.0f, 
                            0.0f, 
                            1.0f );

  pos = mul4x4x4(a_mViewProjInv, pos);
  pos /= pos.w;

  pos.y *= (-1.0f);

  return normalize(to_float3(pos));
}

static inline float2 RaySphereHit(float3 orig, float3 dir, float4 sphere) // see Ray Tracing Gems Book
{
  const float3 center = to_float3(sphere);
  const float  radius = sphere.w;

  // Hearn and Baker equation 10-72 for when radius^2 << distance between origin and center
	// Also at https://www.cg.tuwien.ac.at/courses/EinfVisComp/Slides/SS16/EVC-11%20Ray-Tracing%20Slides.pdf
	// Assumes ray direction is normalized
	//dir = normalize(dir);
	const float3 deltap   = center - orig;
	const float ddp       = dot(dir, deltap);
	const float deltapdot = dot(deltap, deltap);

	// old way, "standard", though it seems to be worse than the methods above
	//float discriminant = ddp * ddp - deltapdot + radius * radius;
	float3 remedyTerm  = deltap - ddp * dir;
	float discriminant = radius * radius - dot(remedyTerm, remedyTerm);

  float2 result = {0,0};
	if (discriminant >= 0.0f)
	{
		const float sqrtVal = sqrt(discriminant);
		// include Press, William H., Saul A. Teukolsky, William T. Vetterling, and Brian P. Flannery, 
		// "Numerical Recipes in C," Cambridge University Press, 1992.
		const float q = (ddp >= 0) ? (ddp + sqrtVal) : (ddp - sqrtVal);
		// we don't bother testing for division by zero
		const float t1 = q;
		const float t2 = (deltapdot - radius * radius) / q;
    result.x = fmin(t1,t2);
    result.y = fmax(t1,t2);
  }
  
  return result;
}

static inline uint RealColorToUint32_f3(float3 real_color)
{
  float  r = real_color.x*255.0f;
  float  g = real_color.y*255.0f;
  float  b = real_color.z*255.0f;
  unsigned char red = (unsigned char)r, green = (unsigned char)g, blue = (unsigned char)b;
  return red | (green << 8) | (blue << 16) | 0xFF000000;
}

static inline uint RealColorToUint32(float4 real_color)
{
  float  r = real_color.x*255.0f;
  float  g = real_color.y*255.0f;
  float  b = real_color.z*255.0f;
  float  a = real_color.w*255.0f;

  unsigned char red   = (unsigned char)r;
  unsigned char green = (unsigned char)g;
  unsigned char blue  = (unsigned char)b;
  unsigned char alpha = (unsigned char)a;

  return red | (green << 8) | (blue << 16) | (alpha << 24);
}

#define WIN_WIDTH  2048
#define WIN_HEIGHT 2048

static uint pitchOffset(uint x, uint y) { return y*WIN_WIDTH + x; } 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline void CoordinateSystem(float3 v1, float3* v2, float3* v3)
{
  float invLen = 1.0f;

  if (fabs(v1.x) > fabs(v1.y))
  {
    invLen = 1.0f / sqrt(v1.x*v1.x + v1.z*v1.z);
    (*v2)  = make_float3((-1.0f) * v1.z * invLen, 0.0f, v1.x * invLen);
  }
  else
  {
    invLen = 1.0f / sqrt(v1.y * v1.y + v1.z * v1.z);
    (*v2)  = make_float3(0.0f, v1.z * invLen, (-1.0f) * v1.y * invLen);
  }

  (*v3) = cross(v1, (*v2));
}

#ifndef M_PI
#define M_PI          3.14159265358979323846f
#endif

#ifndef M_TWOPI
#define M_TWOPI       6.28318530717958647692f
#endif

#ifndef INV_PI
#define INV_PI        0.31830988618379067154f
#endif

#define GEPSILON      5e-6f
#define DEPSILON      1e-20f

//enum THREAD_FLAGS { THREAD_IS_DEAD = 2147483648};

static inline float3 MapSampleToCosineDistribution(float r1, float r2, float3 direction, float3 hit_norm, float power)
{
  if(power >= 1e6f)
    return direction;

  const float sin_phi = sin(M_TWOPI * r1);
  const float cos_phi = cos(M_TWOPI * r1);

  //sincos(2.0f*r1*3.141592654f, &sin_phi, &cos_phi);

  const float cos_theta = pow(1.0f - r2, 1.0f / (power + 1.0f));
  const float sin_theta = sqrt(1.0f - cos_theta*cos_theta);

  float3 deviation;
  deviation.x = sin_theta*cos_phi;
  deviation.y = sin_theta*sin_phi;
  deviation.z = cos_theta;

  float3 ny = direction, nx, nz;
  CoordinateSystem(ny, &nx, &nz);

  {
    float3 temp = ny;
    ny = nz;
    nz = temp;
  }

  float3 res = nx*deviation.x + ny*deviation.y + nz*deviation.z;

  float invSign = dot(direction, hit_norm) > 0.0f ? 1.0f : -1.0f;

  if (invSign*dot(res, hit_norm) < 0.0f) // reflected ray is below surface #CHECK_THIS
  {
    res = (-1.0f)*nx*deviation.x + ny*deviation.y - nz*deviation.z;
    //belowSurface = true;
  }

  return res;
}

static inline float epsilonOfPos(float3 hitPos) { return fmax(fmax(fabs(hitPos.x), fmax(fabs(hitPos.y), fabs(hitPos.z))), 2.0f*GEPSILON)*GEPSILON; }

/**
\brief offset reflected ray position by epsilon;
\param  a_hitPos      - world space position on surface
\param  a_surfaceNorm - surface normal at a_hitPos
\param  a_sampleDir   - ray direction in which we are going to trace reflected ray
\return offseted ray position
*/
static inline float3 OffsRayPos(const float3 a_hitPos, const float3 a_surfaceNorm, const float3 a_sampleDir)
{
  const float signOfNormal2 = dot(a_sampleDir, a_surfaceNorm) < 0.0f ? -1.0f : 1.0f;
  const float offsetEps     = epsilonOfPos(a_hitPos);
  return a_hitPos + signOfNormal2*offsetEps*a_surfaceNorm;
}

//static inline SurfaceHit EvalSurfaceHit(uint a_primId, float2 uv, __global uint* in_indices, __global const float4* in_vpos, __global const float4* in_vnorm)
//{  
//  const uint A = in_indices[a_primId*3 + 0];
//  const uint B = in_indices[a_primId*3 + 1];
//  const uint C = in_indices[a_primId*3 + 2];
//  
//  const float3 A_pos = to_float3(in_vpos[A]);
//  const float3 B_pos = to_float3(in_vpos[B]);
//  const float3 C_pos = to_float3(in_vpos[C]);
//
//  const float3 A_norm = to_float3(in_vnorm[A]);
//  const float3 B_norm = to_float3(in_vnorm[B]);
//  const float3 C_norm = to_float3(in_vnorm[C]);
// 
//  SurfaceHit hit;
//  hit.pos  = (1.0f - uv.x - uv.y)*A_pos  + uv.y*B_pos  + uv.x*C_pos;
//  hit.norm = (1.0f - uv.x - uv.y)*A_norm + uv.y*B_norm + uv.x*C_norm;
//  return hit;
//}

static inline float3 EvalSurfaceNormal(float3 a_rayDir, uint a_primId, float2 uv, __global const uint* in_indices, __global const float4* in_vnorm)
{  
  const uint A = in_indices[a_primId*3 + 0];
  const uint B = in_indices[a_primId*3 + 1];
  const uint C = in_indices[a_primId*3 + 2];

  const float3 A_norm = to_float3(in_vnorm[A]);
  const float3 B_norm = to_float3(in_vnorm[B]);
  const float3 C_norm = to_float3(in_vnorm[C]);
 
  const float3 norm   = (1.0f - uv.x - uv.y)*A_norm + uv.y*B_norm + uv.x*C_norm;

  const float flipNorm = dot(a_rayDir, norm) > 0.001f ? -1.0f : 1.0f;
  return flipNorm*norm;
}

static inline float3 reflect(float3 dir, float3 normal) 
{ 
  // Do not use this function for "wo" and "wh" microfacets terms. 
  // They need the formula: 2.0f * dot(wo, wh) * wh - wo;
  return normalize((normal * dot(dir, normal) * (-2.0f)) + dir); 
}

static inline float3 SphericalDirectionPBRT(const float sintheta, const float costheta, const float phi) 
{ 
  return make_float3(sintheta * cos(phi), sintheta * sin(phi), costheta); 
}

static inline float GGX_Distribution(const float cosThetaNH, const float alpha)
{
  const float alpha2  = alpha * alpha;
  const float NH_sqr  = clamp(cosThetaNH * cosThetaNH, 0.0f, 1.0f);
  const float den     = NH_sqr * alpha2 + (1.0f - NH_sqr);
  return alpha2 / fmax((float)(M_PI) * den * den, 1e-6f);
}

static inline float GGX_GeomShadMask(const float cosThetaN, const float alpha)
{
  // Height - Correlated G.
  //const float tanNV      = sqrt(1.0f - cosThetaN * cosThetaN) / cosThetaN;
  //const float a          = 1.0f / (alpha * tanNV);
  //const float lambda     = (-1.0f + sqrt(1.0f + 1.0f / (a*a))) / 2.0f;
  //const float G          = 1.0f / (1.0f + lambda);

  // Optimized and equal to the commented-out formulas on top.
  const float cosTheta_sqr = clamp(cosThetaN*cosThetaN, 0.0f, 1.0f);
  const float tan2         = (1.0f - cosTheta_sqr) / fmax(cosTheta_sqr, 1e-6f);
  const float GP           = 2.0f / (1.0f + sqrt(1.0f + alpha * alpha * tan2));
  return GP;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline float2 fract2(float2 v)
{
  return v - floor(v);
}

static inline float fract(float v)
{
  return v - floor(v);
}

static inline float3 floor3(float3 v)
{
  return make_float3(floor(v.x), floor(v.y), floor(v.z));
}

static inline float3 fract3(float3 v)
{
  return v - floor3(v);
}

static inline float4 mix4(float4 x, float4 y, float a)
{
  return x*(1.0f - a) + y*a;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline float2 hash2(float2 p ) 
{
  const float2 p1 = make_float2(dot(p, make_float2(123.4f, 748.6f)), dot(p, make_float2(547.3f, 659.3f)));
  return fract2(make_float2(sin(p1.x)*5232.85324f, sin(p1.y)*5232.85324f));   
}

//Based off of iq's described here: http://www.iquilezles.org/www/articles/voronoilin
//

static inline float voronoi(float2 p, float iTime) 
{
    float2 n = floor(p);
    float2 f = fract2(p);
    float md = 5.0f;
    float2 m = make_float2(0.0f, 0.0f);
    for (int i = -1;i<=1;i++) {
        for (int j = -1;j<=1;j++) {
            float2 g = make_float2(i, j);
            float2 o = hash2(n+g);
            o.x = 0.5f+0.5f*sin(iTime+5.038f*o.x);
            o.y = 0.5f+0.5f*sin(iTime+5.038f*o.y);
            float2 r = g + o - f;
            float d = dot(r, r);
            if (d<md) {
              md = d;
              m = n+g+o;
            }
        }
    }
    return md;
}

static inline float ov(float2 p, float iTime) 
{
    float v = 0.0f;
    float a = 0.4f;
    for (int i = 0;i<3;i++) {
        v+= voronoi(p, iTime)*a;
        p *= 2.0f;
        a *= 0.5f;
    }
    return v;
}

static inline float4 BlueWhiteColor(const float3 pos)
{  
  const float4 a  = make_float4(0.20f, 0.4f, 1.0f, 1.0f);
  const float4 b  = make_float4(0.85f, 0.9f, 1.0f, 1.0f)*2.0f;
  
  return mix4(a, b, smoothstep(0.0f, 0.35f, ov(make_float2(pos.x, pos.y), pos.z)));
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// https://www.shadertoy.com/view/4sfGzS

static inline float hash(float3 p)  // replace this by something better
{
  p  = fract3( p*0.3183099f + make_float3(0.1f,0.1f,0.1f) );
	p *= 17.0f;
  return fract( p.x*p.y*p.z*(p.x+p.y+p.z) );
}

static inline float noise(float3 x )
{
  float3 p = floor3(x);
  float3 f = fract3(x);
  f = f*f*(make_float3(3.0f,3.0f,3.0f) - 2.0f*f);

  return mix(mix(mix( hash(p + make_float3(0,0,0)), 
                      hash(p + make_float3(1,0,0)),f.x),
                 mix( hash(p + make_float3(0,1,0)), 
                      hash(p + make_float3(1,1,0)),f.x),f.y),
             mix(mix( hash(p + make_float3(0,0,1)), 
                      hash(p + make_float3(1,0,1)),f.x),
                 mix( hash(p + make_float3(0,1,1)), 
                      hash(p + make_float3(1,1,1)),f.x),f.y),f.z);
}


static inline float3 WhiteNoise(const float3 pos)
{ 
  const float3x3 m = make_float3x3(make_float3(0.00f,  0.80f,  0.60f),
                                   make_float3(-0.80f,  0.36f, -0.48f),
                                   make_float3(-0.60f, -0.48f,  0.64f));
   
  float f = 0.0f;
  
  float3 q = 10.0f*pos;
  f  = 0.5000f*noise( q ); q = mul3x3x3(m, q*2.01f);
  f += 0.2500f*noise( q ); q = mul3x3x3(m, q*2.02f);
  f += 0.1250f*noise( q ); q = mul3x3x3(m, q*2.03f);
  f += 0.0625f*noise( q ); q = mul3x3x3(m, q*2.01f);
  
  return make_float3(f, f, f);
}


#endif