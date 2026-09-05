#pragma once
#include "math.hpp"

// Exactly representable 24-bit conversion: never rounds up to one.
FORCE_INLINE daxa_f32 pt_unit_float(daxa_u32 bits) {
  return daxa_f32(bits >> 8u) * (1.0f / 16777216.0f);
}

FORCE_INLINE daxa_f32vec3 pt_face_normal(daxa_u32 face) {
  daxa_f32 s = (face % 2u == 0u) ? 1.0f : -1.0f;
  return face < 2u ? daxa_f32vec3(s,0,0) :
         face < 4u ? daxa_f32vec3(0,s,0) : daxa_f32vec3(0,0,s);
}

FORCE_INLINE daxa_f32 pt_face_area(daxa_f32vec3 lo, daxa_f32vec3 hi, daxa_u32 face) {
  daxa_f32vec3 d = hi - lo;
  return face < 2u ? d.y*d.z : face < 4u ? d.x*d.z : d.x*d.y;
}

FORCE_INLINE daxa_f32vec3 pt_face_point(daxa_f32vec3 lo, daxa_f32vec3 hi,
                                       daxa_u32 face, daxa_f32 u, daxa_f32 v) {
  if (face < 2u) return daxa_f32vec3(face == 0u ? hi.x : lo.x,
      lo.y+(hi.y-lo.y)*u, lo.z+(hi.z-lo.z)*v);
  if (face < 4u) return daxa_f32vec3(lo.x+(hi.x-lo.x)*u,
      face == 2u ? hi.y : lo.y, lo.z+(hi.z-lo.z)*v);
  return daxa_f32vec3(lo.x+(hi.x-lo.x)*u, lo.y+(hi.y-lo.y)*v,
      face == 4u ? hi.z : lo.z);
}
