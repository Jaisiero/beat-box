#pragma once

#define DAXA_RAY_TRACING 1
#include "daxa/daxa.inl"
#include "daxa/utils/task_graph.inl"

#include <math.hpp>


struct RigidBody
{
  daxa_u32 primitive_count;
  daxa_u32 primitive_offset;
  daxa_f32vec3 position;
  daxa_f32vec4 rotation;
  // TODO: Add more rigid body properties
};
DAXA_DECL_BUFFER_PTR(RigidBody)

struct CameraView
{
  daxa_f32mat4x4 inv_view;
  daxa_f32mat4x4 inv_proj;
};
DAXA_DECL_BUFFER_PTR(CameraView)

DAXA_DECL_BUFFER_PTR(Aabb)

DAXA_DECL_TASK_HEAD_BEGIN(RayTracingTaskHead)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ, daxa_BufferPtr(CameraView), camera)
DAXA_TH_IMAGE_ID(RAY_TRACING_SHADER_STORAGE_WRITE_ONLY, REGULAR_2D, swapchain)
DAXA_TH_TLAS_ID(RAY_TRACING_SHADER_READ, tlas)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ, daxa_BufferPtr(Aabb), aabbs)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_DECL_TASK_HEAD_END

static const daxa_f32 T_MIN = 1e-3f;
static const daxa_f32 T_MAX = 1e9f;
static const daxa_f32 PI = 3.14159265359f;


struct PushConstants
{
  DAXA_TH_BLOB(RayTracingTaskHead, task_head)
};


#if defined(__cplusplus)
daxa_f32mat3x4 rigid_body_get_transform_matrix(const RigidBody &rigid_body) {
    daxa_f32vec3 translation = rigid_body.position;
    daxa_f32vec4 rotation = rigid_body.rotation;

    // // transform quaternion to matrix
    // daxa_f32 x2 = rotation.x + rotation.x;
    // daxa_f32 y2 = rotation.y + rotation.y;
    // daxa_f32 z2 = rotation.z + rotation.z;
    // daxa_f32 xx = rotation.x * x2;
    // daxa_f32 xy = rotation.x * y2;
    // daxa_f32 xz = rotation.x * z2;
    // daxa_f32 yy = rotation.y * y2;
    // daxa_f32 yz = rotation.y * z2;
    // daxa_f32 zz = rotation.z * z2;
    // daxa_f32 wx = rotation.w * x2;
    // daxa_f32 wy = rotation.w * y2;
    // daxa_f32 wz = rotation.w * z2;

    // daxa_f32mat3x3 rotation_matrix = daxa_f32mat3x3(daxa_f32vec3(1.0f - (yy + zz), xy - wz, xz + wy),
    //                                                 daxa_f32vec3(xy + wz, 1.0f - (xx + zz), yz - wx),
    //                                                 daxa_f32vec3(xz - wy, yz + wx, 1.0f - (xx + yy)));

    daxa_f32mat3x3 rotation_matrix = daxa_f32mat3x3(daxa_f32vec3(1.0f, 0.0f, 0.0f),
                                                    daxa_f32vec3(0.0f, 1.0f, 0.0f),
                                                    daxa_f32vec3(0.0f, 0.0f, 1.0f));

    return daxa_f32mat3x4(daxa_f32vec4(rotation_matrix.x.x, rotation_matrix.y.x, rotation_matrix.z.x, translation.x),
                        daxa_f32vec4(rotation_matrix.x.y, rotation_matrix.y.y, rotation_matrix.z.y, translation.y),
                        daxa_f32vec4(rotation_matrix.x.z, rotation_matrix.y.z, rotation_matrix.z.z, translation.z));
}
#endif // defined(__cplusplus)


// RT STRUCTS
struct HitPayload
{
  daxa_f32vec3 hit_value;
  daxa_f32vec3 position;
  daxa_f32vec3 normal;
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



daxa_f32mat4x4 Convert3x4To4x4(
#if DAXA_SHADERLANG == DAXA_SHADERLANG_GLSL
  daxa_f32mat4x3
#elif DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
  daxa_f32mat3x4 
#endif // DAXA_SHADERLANG
  obj_to_world_4x3)
{
    daxa_f32mat4x4 obj_to_world_4x4;

    obj_to_world_4x4[0] = daxa_f32vec4(obj_to_world_4x3[0], 0.0f);
    obj_to_world_4x4[1] = daxa_f32vec4(obj_to_world_4x3[1], 0.0f);
    obj_to_world_4x4[2] = daxa_f32vec4(obj_to_world_4x3[2], 0.0f);
    obj_to_world_4x4[3] = daxa_f32vec4(obj_to_world_4x3[3], 1.0f);

    return obj_to_world_4x4;
}


daxa_f32vec3 compute_diffuse(daxa_f32vec3 mat_color, daxa_f32vec3 normal, daxa_f32vec3 light_dir)
{
    daxa_f32 NdotL = max(dot(normal, light_dir), 0.0f);
    return mat_color * NdotL;
}

daxa_f32vec3 compute_specular(daxa_f32 shininess, daxa_f32vec3 mat_specular, daxa_f32vec3 view_dir, daxa_f32vec3 normal, daxa_f32vec3 light_dir) {
    
  daxa_f32 _shininess = max(shininess, 4.0f);
  
  const daxa_f32 energy_conservation = (_shininess + 2.0f) / (2.0f * PI);
  daxa_f32vec3 V = normalize(-view_dir);
  daxa_f32vec3 R = reflect(-light_dir, normal);
  daxa_f32 VdotR = max(dot(V, R), 0.0f);
  daxa_f32 specular = pow(VdotR, _shininess) * energy_conservation;
  return mat_specular * specular;
}

#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG