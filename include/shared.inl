#pragma once

#define DAXA_RAY_TRACING 1
#include "daxa/daxa.inl"
#include "daxa/utils/task_graph.inl"

#include <math.hpp>

struct Camera
{
  daxa_f32mat4x4 inv_view;
  daxa_f32mat4x4 inv_proj;
  daxa_u32vec2 size;
};
DAXA_DECL_BUFFER_PTR(Camera)

DAXA_DECL_TASK_HEAD_BEGIN(RayTracingTaskHead)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ, daxa_BufferPtr(Camera), camera)
DAXA_TH_IMAGE_ID(RAY_TRACING_SHADER_STORAGE_WRITE_ONLY, REGULAR_2D, swapchain)
// DAXA_TH_TLAS_PTR(RAY_TRACING_SHADER_READ, tlas)
DAXA_DECL_TASK_HEAD_END



struct PushConstants
{
  DAXA_TH_BLOB(RayTracingTaskHead, task_head)
};


// RT STRUCTS
struct HitPayload
{
  daxa_f32vec3 hit_value;
};

struct ShadowRayPayload
{
  daxa_f32 shadow;
};

struct MyAttributes {
    daxa_f32vec3 normal;
    daxa_f32vec3 position;
};

[[vk::push_constant]] PushConstants p;


#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG

RayDesc create_ray(daxa_f32mat4x4 inv_view, daxa_f32mat4x4 inv_proj, daxa_u32vec2 thread_idx, daxa_u32vec2 rt_size, daxa_f32 tmin, daxa_f32 tmax)
{
  const daxa_f32vec2 pixel_center = daxa_f32vec2(thread_idx) + daxa_f32vec2(0.5);
  const daxa_f32vec2 inv_UV = pixel_center / daxa_f32vec2(rt_size);
  daxa_f32vec2 d = inv_UV * 2.0 - 1.0;

  daxa_f32vec4 origin = mul(inv_view, daxa_f32vec4(0, 0, 0, 1));
  daxa_f32vec4 target = mul(inv_proj, daxa_f32vec4(d.x, d.y, 1, 1));
  daxa_f32vec4 direction = mul(inv_view, daxa_f32vec4(normalize(target.xyz), 0));

  RayDesc ray;
  ray.Origin = origin.xyz;
  ray.Direction = direction.xyz;
  ray.TMin = tmin;
  ray.TMax = tmax;
  return ray;
}

#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG