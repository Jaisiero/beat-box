#pragma once

#if !defined(DAXA_RAY_TRACING)
#define DAXA_RAY_TRACING 1
#endif // DAXA_RAY_TRACING
#include "daxa/daxa.inl"

#if defined(__cplusplus)
#define FORCE_INLINE inline
#else
#define FORCE_INLINE
#endif


#if defined(__cplusplus)
#define MAT_ELEM(mat, row, col) ( \
    ((col) == 0) ? ((&mat.x)->*(row == 0 ? &daxa_f32vec3::x : row == 1 ? &daxa_f32vec3::y : &daxa_f32vec3::z)) : \
    ((col) == 1) ? ((&mat.y)->*(row == 0 ? &daxa_f32vec3::x : row == 1 ? &daxa_f32vec3::y : &daxa_f32vec3::z)) : \
    ((&mat.z)->*(row == 0 ? &daxa_f32vec3::x : row == 1 ? &daxa_f32vec3::y : &daxa_f32vec3::z)) \
)
#else
#define MAT_ELEM(mat, x, y) mat[x][y]
#endif // __cplusplus

#if defined(__cplusplus) // C++
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <limits>
#define MAX std::max
#define MIN std::min
#define MAX_U32 std::numeric_limits<daxa_u32>::max()
#define MAX_U64 std::numeric_limits<daxa_u64>::max()
#define MIN_I32 std::numeric_limits<daxa_i32>::min()
#define MAX_I32 std::numeric_limits<daxa_i32>::max()

FORCE_INLINE daxa_f32vec3 operator-(const daxa_f32vec3& a, const daxa_f32vec3& b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

FORCE_INLINE daxa_f32vec3 operator*(const daxa_f32vec3& a, const daxa_f32vec3& b)
{
    return {a.x * b.x, a.y * b.y, a.z * b.z};
}

FORCE_INLINE daxa_f32vec3 operator/(const daxa_f32& a, const daxa_f32vec3& b)
{
    return {a / b.x, a / b.y, a / b.z};
}

FORCE_INLINE bool operator<(const daxa_f32vec3& a, const daxa_f32vec3& b)
{
  auto lenght_a = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  auto lenght_b = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
  return lenght_a < lenght_b;
}

FORCE_INLINE daxa_f32vec3 operator*(const daxa_f32& a, const daxa_f32vec3& b)
{
    return {a * b.x, a * b.y, a * b.z};
}

FORCE_INLINE daxa_f32vec3 operator*(const daxa_f32vec3& a, const daxa_f32& b)
{
    return {a.x * b, a.y * b, a.z * b};
}

FORCE_INLINE daxa_f32vec3 operator/(const daxa_f32vec3& a, const daxa_f32& b)
{
    return {a.x / b, a.y / b, a.z / b};
}

FORCE_INLINE daxa_b32 operator!=(const daxa_f32vec3& a, const daxa_f32vec3& b)
{
    return a.x != b.x || a.y != b.y || a.z != b.z;
}

FORCE_INLINE daxa_f32mat4x4 daxa_mat4_from_glm_mat4(glm::mat4 const& m)
{
    return {
        m[0][0], m[0][1], m[0][2], m[0][3],
        m[1][0], m[1][1], m[1][2], m[1][3],
        m[2][0], m[2][1], m[2][2], m[2][3],
        m[3][0], m[3][1], m[3][2], m[3][3]
    };
}

FORCE_INLINE daxa_f32mat3x3 daxa_mat3_from_glm_mat3(glm::mat3 const& m)
{
    return {
        m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2]
    };
}

FORCE_INLINE auto cuboid_get_inverse_intertia(daxa_f32 mass, daxa_f32vec3 min, daxa_f32vec3 max) -> daxa_f32mat3x3
{
  if(mass == 0.0f) {
    return daxa_f32mat3x3(daxa_f32vec3(0.0f), daxa_f32vec3(0.0f), daxa_f32vec3(0.0f));
  }

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

static const daxa_f32 FLT_MIN = 1.175494351e-38F;
static const daxa_f32 FLT_MAX = 3.402823466e+38F;
static const daxa_f32 EPSILON = 1.192092896e-07F;
static const daxa_u32 MAX_U32 = 0xFFFFFFFF;
static const daxa::i32 MIN_I32 = 0x80000000;
static const daxa::i32 MAX_I32 = 0x7FFFFFFF;
static const daxa_u64 MAX_U64 = 0xFFFFFFFFFFFFFFFF;
static const daxa_f32 GOLDEN_RATIO = 1.61803398875F;
#endif // __cplusplus

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

struct Quaternion {
  daxa_f32vec3 v;
  daxa_f32 w;
#if defined(__cplusplus)
  Quaternion(daxa_f32vec3 v, daxa_f32 w) : v(v), w(w) {}
  Quaternion(daxa_f32 x, daxa_f32 y, daxa_f32 z, daxa_f32 w) : v({x, y, z}), w(w) {}
#else 
  __init(daxa_f32vec3 v, daxa_f32 w) {
    this.v = v;
    this.w = w;
  }
  __init(daxa_f32 x, daxa_f32 y, daxa_f32 z, daxa_f32 w) {
    this.v = daxa_f32vec3(x, y, z);
    this.w = w;
  }
#endif // __cplusplus

  daxa_f32 magnitude() {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z + w * w);
  }

  Quaternion normalize() {
    daxa_f32 mag = magnitude();
    return Quaternion(v / mag, w / mag);
  }

  Quaternion conjugate() {
    return Quaternion(-v.x, -v.y, -v.z, w);
  }

  daxa_f32mat3x3 to_matrix() {
    daxa_f32 x2 = v.x + v.x;
    daxa_f32 y2 = v.y + v.y;
    daxa_f32 z2 = v.z + v.z;
    daxa_f32 xx = v.x * x2;
    daxa_f32 xy = v.x * y2;
    daxa_f32 xz = v.x * z2;
    daxa_f32 yy = v.y * y2;
    daxa_f32 yz = v.y * z2;
    daxa_f32 zz = v.z * z2;
    daxa_f32 wx = w * x2;
    daxa_f32 wy = w * y2;
    daxa_f32 wz = w * z2;

    return daxa_f32mat3x3(daxa_f32vec3(1 - (yy + zz), xy - wz, xz + wy),
                          daxa_f32vec3(xy + wz, 1 - (xx + zz), yz - wx),
                          daxa_f32vec3(xz - wy, yz + wx, 1 - (xx + yy)));
  }

  daxa_f32vec3 get_x_axis() {
    return daxa_f32vec3(1 - 2 * (v.y * v.y + v.z * v.z),
                        2 * (v.x * v.y + w * v.z),
                        2 * (v.x * v.z - w * v.y));
  }

  daxa_f32vec3 get_y_axis() {
    return daxa_f32vec3(2 * (v.x * v.y - w * v.z),
                        1 - 2 * (v.x * v.x + v.z * v.z),
                        2 * (v.y * v.z + w * v.x));
  }

  daxa_f32vec3 get_z_axis() {
    return daxa_f32vec3(2 * (v.x * v.z + w * v.y),
                        2 * (v.y * v.z - w * v.x),
                        1 - 2 * (v.x * v.x + v.y * v.y));
  }

  daxa_f32vec3 get_invert_x_axis() {
    return daxa_f32vec3(1 - 2 * (v.y * v.y + v.z * v.z),
                        2 * (v.x * v.y - w * v.z),
                        2 * (v.x * v.z + w * v.y));
  }

  daxa_f32vec3 get_invert_y_axis() {
    return daxa_f32vec3(2 * (v.x * v.y + w * v.z),
                        1 - 2 * (v.x * v.x + v.z * v.z),
                        2 * (v.y * v.z - w * v.x));
  }

  daxa_f32vec3 get_invert_z_axis() {
    return daxa_f32vec3(2 * (v.x * v.z - w * v.y),
                        2 * (v.y * v.z + w * v.x),
                        1 - 2 * (v.x * v.x + v.y * v.y));
  }
};

FORCE_INLINE Quaternion operator*(Quaternion q1, Quaternion q2) {
  return Quaternion(
    daxa_f32vec3(q1.w * q2.v.x + q1.v.x * q2.w + q1.v.y * q2.v.z - q1.v.z * q2.v.y,
    q1.w * q2.v.y + q1.v.y * q2.w + q1.v.z * q2.v.x - q1.v.x * q2.v.z,
    q1.w * q2.v.z + q1.v.z * q2.w + q1.v.x * q2.v.y - q1.v.y * q2.v.x),
    q1.w * q2.w - q1.v.x * q2.v.x - q1.v.y * q2.v.y - q1.v.z * q2.v.z
  );
}

FORCE_INLINE Quaternion operator*(Quaternion q, daxa_f32vec3 v) {
  return q * Quaternion(v, 0);
}

FORCE_INLINE Quaternion operator*(daxa_f32vec3 v, Quaternion q) {
  return Quaternion(v, 0) * q;
}

FORCE_INLINE Quaternion from_matrix(daxa_f32mat3x3 mat) {
  daxa_f32 s;
  daxa_f32 x, y, z, w;

  daxa_f32 xx = MAT_ELEM(mat, 0, 0);
  daxa_f32 yy = MAT_ELEM(mat, 1, 1);
  daxa_f32 zz = MAT_ELEM(mat, 2, 2);
  daxa_f32 t = xx + yy + zz;

  if (t > 0.0f) {
    s = sqrt(t + 1.0f);
    w = s * 0.5f;
    s = 0.5f / s;
    x = (MAT_ELEM(mat, 2, 1) - MAT_ELEM(mat, 1, 2)) * s;
    y = (MAT_ELEM(mat, 0, 2) - MAT_ELEM(mat, 2, 0)) * s;
    z = (MAT_ELEM(mat, 1, 0) - MAT_ELEM(mat, 0, 1)) * s;
  } else {
    daxa_i32 i = xx < yy ? (yy < zz ? 2 : 1) : (xx < zz ? 2 : 0);
    daxa_i32 j = (i + 1) % 3;
    daxa_i32 k = (i + 2) % 3;

    s = sqrt(MAT_ELEM(mat, i, i) - MAT_ELEM(mat, j, j) - MAT_ELEM(mat, k, k) + 1.0f);
    daxa_f32 q[3];
    q[i] = s * 0.5f;
    s = 0.5f / s;
    w = (MAT_ELEM(mat, k, j) - MAT_ELEM(mat, j, k)) * s;
    q[j] = (MAT_ELEM(mat, j, i) + MAT_ELEM(mat, i, j)) * s;
    q[k] = (MAT_ELEM(mat, k, i) + MAT_ELEM(mat, i, k)) * s;
  }

  return Quaternion(x, y, z, w);
};



struct Transform {
  daxa_f32vec3 position;
  Quaternion quat;
  daxa_f32mat3x3 rotation;
#if defined(__cplusplus)
  Transform(Quaternion quat, daxa_f32vec3 position) : quat(quat), position(position), rotation(quat.to_matrix()) {
  }
  Transform(daxa_f32mat3x3 rotation, daxa_f32vec3 position) : rotation(rotation), position(position), quat(from_matrix(rotation)) {
  }
#else
  __init(Quaternion quat, daxa_f32vec3 position) {
    this.quat = quat;
    this.rotation = transpose(quat.to_matrix());
    this.position = position;
  }
  __init(daxa_f32mat3x3 rotation, daxa_f32vec3 position) {
    this.rotation = rotation;
    this.position = position;
    this.quat = from_matrix(rotation);
  }
  
#endif // __cplusplus
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
  daxa_f32 bias_factor;
  daxa_f32 impulse_coefficient;
  daxa_f32 mass_coefficient;
  daxa_f32 normal_mass;
  daxa_f32 tangent_mass[2];
  FeaturePair fp;
  daxa_u32 warm_start;
};

struct Manifold {
  daxa_u32 obb1_index;
  daxa_u32 obb2_index;
  daxa_f32vec3 anchor_a;
  daxa_f32vec3 anchor_b;
  daxa_i32 key;

  daxa_i32 error;
  // DEBUG
  daxa_u64 frame_count;
  // daxa_f32 s_max;
  // Transform rtx;
  // daxa_f32vec3 e_r;
  // Transform itx;
  // daxa_f32vec3 e_i;
  // daxa_f32vec3 e;
  // daxa_f32mat3x3 basis;

  daxa_f32vec3 normal;
  daxa_f32vec3 tangent_vectors[2];
  daxa_i32 contact_count;
  Contact contacts[MAX_CONTACT_POINT_COUNT];
};
DAXA_DECL_BUFFER_PTR(Manifold)

struct Ray {
    daxa_f32vec3 origin;
    daxa_f32vec3 direction;
};


FORCE_INLINE daxa_f32 hitAabb(const Aabb aabb, const Ray r)
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


#if defined(__cplusplus)
FORCE_INLINE daxa_f32 dot(const daxa_f32vec3 a, const daxa_f32vec3 b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
#endif // __cplusplus


FORCE_INLINE daxa_f32 hitSphere(const daxa_f32vec3 center, const daxa_f32 radius, const Ray r)
{
  daxa_f32vec3 oc = r.origin - center;
  daxa_f32 a = dot(r.direction, r.direction);
  daxa_f32 b = 2.0f * dot(oc, r.direction);
  daxa_f32 c = dot(oc, oc) - radius * radius;
  daxa_f32 discriminant = b * b - 4 * a * c;
  if (discriminant < 0)
    return -1.0f;
  return (-b - sqrt(discriminant)) / (2.0f * a);
}