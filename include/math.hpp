#pragma once

#if !defined(DAXA_RAY_TRACING)
#define DAXA_RAY_TRACING 1
#endif // DAXA_RAY_TRACING
#include "daxa/daxa.inl"

struct Aabb
{
    daxa_f32vec3 minimum;
    daxa_f32vec3 maximum;
#if !defined(__cplusplus)
  daxa_f32vec3 center() {
    return (this.minimum + this.maximum) * 0.5;
  }

  daxa_f32vec3 size() {
    return this.maximum - this.minimum;
  }

  daxa_f32vec3 get_corner(daxa_u32 index) {
    daxa_f32vec3 result;
    result.x = (index & 1) == 0 ? this.minimum.x : this.maximum.x;
    result.y = (index & 2) == 0 ? this.minimum.y : this.maximum.y;
    result.z = (index & 4) == 0 ? this.minimum.z : this.maximum.z;
    return result;
  }
#endif // __cplusplus
};

static const daxa_u32 MAX_INCIDENT_VERTEX_COUNT = 4;
static const daxa_u32 MAX_CONTACT_POINT_COUNT = 8;

struct Transform {
  daxa_f32vec3 position;
  daxa_f32mat3x3 rotation;
};

struct FeaturePair {
  daxa_u32 in_reference;
  daxa_u32 out_reference;
  daxa_u32 in_incident;
  daxa_u32 out_incident;
};

struct ClipVertex {
  daxa_f32vec3 v;
  FeaturePair f;
};

struct Contact {
  daxa_f32vec3 position;
  daxa_f32 penetration;
  daxa_f32 normal_impulse;
  daxa_f32 tangent_impulse[2];
  daxa_f32 bias;
  daxa_f32 normal_mass;
  daxa_f32 tangent_mass[2];
  FeaturePair fp;
};

struct Manifold {
  daxa_u32 obb1_index;
  daxa_u32 obb2_index;
  daxa_i32 key;

  daxa_i32 error;
  // // DEBUG
  // daxa_f32 s_max;
  // Transform rtx;
  // daxa_f32vec3 e_r;
  // Transform itx;
  // daxa_f32vec3 e_i;
  // daxa_f32vec3 e;
  // daxa_f32mat3x3 basis;

  daxa_f32vec3 normal;
  daxa_f32vec3 tangent_vectors[2];
  Contact contacts[MAX_CONTACT_POINT_COUNT];
  daxa_i32 contact_count;
};
DAXA_DECL_BUFFER_PTR(Manifold)

struct Ray {
    daxa_f32vec3 origin;
    daxa_f32vec3 direction;
};

#if defined(__cplusplus) // C++
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#define MAX std::max
#define MIN std::min

daxa_f32vec3 operator-(const daxa_f32vec3& a, const daxa_f32vec3& b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

daxa_f32vec3 operator*(const daxa_f32vec3& a, const daxa_f32vec3& b)
{
    return {a.x * b.x, a.y * b.y, a.z * b.z};
}

daxa_f32vec3 operator/(const daxa_f32& a, const daxa_f32vec3& b)
{
    return {a / b.x, a / b.y, a / b.z};
}

bool operator<(const daxa_f32vec3& a, const daxa_f32vec3& b)
{
  auto lenght_a = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  auto lenght_b = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
  return lenght_a < lenght_b;
}

daxa_f32vec3 operator*(const daxa_f32& a, const daxa_f32vec3& b)
{
    return {a * b.x, a * b.y, a * b.z};
}

daxa_f32mat4x4 daxa_mat4_from_glm_mat4(glm::mat4 const& m)
{
    return {
        m[0][0], m[0][1], m[0][2], m[0][3],
        m[1][0], m[1][1], m[1][2], m[1][3],
        m[2][0], m[2][1], m[2][2], m[2][3],
        m[3][0], m[3][1], m[3][2], m[3][3]
    };
}

daxa_f32mat3x3 daxa_mat3_from_glm_mat3(glm::mat3 const& m)
{
    return {
        m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2]
    };
}

auto cuboid_get_inverse_intertia(daxa_f32 mass, daxa_f32vec3 min, daxa_f32vec3 max) -> daxa_f32mat3x3
{
    daxa_f32vec3 size = max - min; // (width, height, depth)
    daxa_f32 w = size.x;
    daxa_f32 h = size.y;
    daxa_f32 d = size.z;

    daxa_f32 Ixx = (1.0f / 12.0f) * mass * (h * h + d * d);
    daxa_f32 Iyy = (1.0f / 12.0f) * mass * (w * w + d * d);
    daxa_f32 Izz = (1.0f / 12.0f) * mass * (w * w + h * h);

    daxa_f32 inv_Ixx = 1.0f / Ixx;
    daxa_f32 inv_Iyy = 1.0f / Iyy;
    daxa_f32 inv_Izz = 1.0f / Izz;

    return daxa_f32mat3x3(
        daxa_f32vec3(inv_Ixx, 0.0f, 0.0f),
        daxa_f32vec3(0.0f, inv_Iyy, 0.0f),
        daxa_f32vec3(0.0f, 0.0f, inv_Izz)
    );
}

#else 
#define MAX max
#define MIN min

static const daxa_f32 FLT_MAX = 3.402823466e+38F;
static const daxa_f32 FLT_MIN = 1.175494351e-38F;
static const daxa_f32 EPSILON = 1.192092896e-07F;
#endif // __cplusplus


daxa_f32 hitAabb(const Aabb aabb, const Ray r)
{
  daxa_f32vec3  invDir = 1.0f / r.direction;
  daxa_f32vec3  tbot   = invDir * (aabb.minimum - r.origin);
  daxa_f32vec3  ttop   = invDir * (aabb.maximum - r.origin);
  daxa_f32vec3  tmin   = MIN(ttop, tbot);
  daxa_f32vec3  tmax   = MAX(ttop, tbot);
  daxa_f32 t0     = MAX(tmin.x, MAX(tmin.y, tmin.z));
  daxa_f32 t1     = MIN(tmax.x, MIN(tmax.y, tmax.z));
  return t1 > MAX(t0, 0.0f) ? t0 : -1.0f;
  // if (t1 < 0.0f || t0 > t1)
  //   return -1.0f; // No intersection

  // if (t0 < 0.0f)
  //   return t1; // Ray origin inside AABB, return exit point

  // return t0; // Ray intersects AABB, return entry point
}