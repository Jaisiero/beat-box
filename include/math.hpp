#pragma once

struct Aabb
{
    daxa_f32vec3 min;
    daxa_f32vec3 max;
};

struct Ray {
    daxa_f32vec3 origin;
    daxa_f32vec3 direction;
};

#if defined(__cplusplus) // C++
#include <cmath>
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

#else 
#define MAX max
#define MIN min
#endif // __cplusplus


daxa_f32 hitAabb(const Aabb aabb, const Ray r)
{
  daxa_f32vec3  invDir = 1.0f / r.direction;
  daxa_f32vec3  tbot   = invDir * (aabb.min - r.origin);
  daxa_f32vec3  ttop   = invDir * (aabb.max - r.origin);
  daxa_f32vec3  tmin   = MIN(ttop, tbot);
  daxa_f32vec3  tmax   = MAX(ttop, tbot);
  daxa_f32 t0     = MAX(tmin.x, MAX(tmin.y, tmin.z));
  daxa_f32 t1     = MIN(tmax.x, MIN(tmax.y, tmax.z));
  return t1 > MAX(t0, 0.0f) ? t0 : -1.0f;
}