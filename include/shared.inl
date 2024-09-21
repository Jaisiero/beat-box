#pragma once

#define DAXA_RAY_TRACING 1
#include "daxa/daxa.inl"
#include "daxa/utils/task_graph.inl"

#include <math.hpp>

static const daxa_f32 LINEAR_DAMPING = 0.1f;
static const daxa_f32 ANGULAR_DAMPING = 0.1f;

#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
[Flags]
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
enum RigidBodyFlag : daxa_u32 {
  NONE = 0,
  COLLIDING = 1 << 0,
};

#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
RigidBodyFlag  operator|(RigidBodyFlag a, RigidBodyFlag b)
{
    return RigidBodyFlag((daxa_u32)a | (daxa_u32)b);
}
// Define bitwise OR assignment operator
void operator|=(inout RigidBodyFlag a, RigidBodyFlag b)
{
    a = a | b;
}

// Define bitwise AND operator
RigidBodyFlag operator&(RigidBodyFlag a, RigidBodyFlag b)
{
    return RigidBodyFlag((daxa_u32)a & (daxa_u32)b);
}
// Define bitwise AND assignment operator
void operator&=(inout RigidBodyFlag a, RigidBodyFlag b)
{
    a = a & b;
}
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG


// RigidBody struct is 156 bytes
struct RigidBody
{
  RigidBodyFlag flags;
  daxa_u32 primitive_count;
  daxa_u32 primitive_offset;
  daxa_f32vec3 position;
  daxa_f32vec4 rotation;
  daxa_f32vec3 min;
  daxa_f32vec3 max;
  daxa_f32 mass;
  daxa_f32 inv_mass;
  daxa_f32vec3 velocity;
  daxa_f32vec3 omega;
  daxa_f32vec3 tmp_velocity;
  daxa_f32vec3 tmp_omega;
  daxa_f32mat3x3 inv_inertia;
  daxa_f32 restitution;
  daxa_f32 friction;
  // DEBUG
  daxa_u32 iter;
  // TODO: Add more rigid body properties
  // daxa_f32 drag;
  // daxa_f32 angular_drag;

  daxa_f32mat4x4 get_transform_matrix()
  {
    daxa_f32vec3 translation = position;
    daxa_f32vec4 rot = rotation;

    // transform quaternion to matrix
    daxa_f32 x2 = rot.x + rot.x;
    daxa_f32 y2 = rot.y + rot.y;
    daxa_f32 z2 = rot.z + rot.z;
    daxa_f32 xx = rot.x * x2;
    daxa_f32 xy = rot.x * y2;
    daxa_f32 xz = rot.x * z2;
    daxa_f32 yy = rot.y * y2;
    daxa_f32 yz = rot.y * z2;
    daxa_f32 zz = rot.z * z2;
    daxa_f32 wx = rot.w * x2;
    daxa_f32 wy = rot.w * y2;
    daxa_f32 wz = rot.w * z2;

    daxa_f32mat3x3 rotation_matrix = daxa_f32mat3x3(daxa_f32vec3(1.0f - (yy + zz), xy - wz, xz + wy),
                                                    daxa_f32vec3(xy + wz, 1.0f - (xx + zz), yz - wx),
                                                    daxa_f32vec3(xz - wy, yz + wx, 1.0f - (xx + yy)));

#if defined(__cplusplus)
    return daxa_f32mat4x4(daxa_f32vec4(rotation_matrix.x.x, rotation_matrix.y.x, rotation_matrix.z.x, translation.x),
                          daxa_f32vec4(rotation_matrix.x.y, rotation_matrix.y.y, rotation_matrix.z.y, translation.y),
                          daxa_f32vec4(rotation_matrix.x.z, rotation_matrix.y.z, rotation_matrix.z.z, translation.z));
#else // defined(__cplusplus)
    return daxa_f32mat4x4(rotation_matrix[0][0], rotation_matrix[1][0], rotation_matrix[2][0], 0.0f,
                          rotation_matrix[0][1], rotation_matrix[1][1], rotation_matrix[2][1], 0.0f,
                          rotation_matrix[0][2], rotation_matrix[1][2], rotation_matrix[2][2], 0.0f,
                          translation.x, translation.y, translation.z, 1.0f);

#endif // defined(__cplusplus)
  }

#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
  [mutating] bool has_flag(RigidBodyFlag flag)
  {
    return (this.flags & flag) != 0;
  }

  [mutating] void set_flag(RigidBodyFlag flag)
  {
    this.flags |= flag;
  }

  [mutating] void clear_flag(RigidBodyFlag flag)
  {
    this.flags &= ~flag;
  }
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
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
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ_WRITE, daxa_BufferPtr(Aabb), aabbs)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ_WRITE, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_DECL_TASK_HEAD_END

static const daxa_f32 T_MIN = 1e-3f;
static const daxa_f32 T_MAX = 1e9f;
static const daxa_f32 PI = 3.14159265359f;

struct SimConfig
{
  daxa_u32 rigid_body_count;
  daxa_f32 dt;
  daxa_f32 gravity;
  daxa_u32 collision_count;
};
DAXA_DECL_BUFFER_PTR(SimConfig)

struct DispatchBuffer
{
  daxa_u32 dispatch_x;
  daxa_u32 dispatch_y;
  daxa_u32 dispatch_z;
};
DAXA_DECL_BUFFER_PTR(DispatchBuffer)

struct RTPushConstants
{
  DAXA_TH_BLOB(RayTracingTaskHead, task_head)
};

static const daxa_u32 RIGID_BODY_SIM_COMPUTE_X = 64;

struct Collision
{
  daxa_u32 a;
  daxa_u32 b;
  daxa_f32vec3 penetration;
};
DAXA_DECL_BUFFER_PTR(Collision)

DAXA_DECL_TASK_HEAD_BEGIN(GJKTaskHead)
DAXA_TH_BUFFER_PTR(TRANSFER_WRITE, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(TRANSFER_WRITE, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(Aabb), aabbs)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Collision), collisions)
DAXA_DECL_TASK_HEAD_END

struct GJKPushConstants
{
  DAXA_TH_BLOB(GJKTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(RigidBodySimTaskHead)
DAXA_TH_BUFFER_PTR(TRANSFER_WRITE, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(Aabb), aabbs)
DAXA_DECL_TASK_HEAD_END

struct RigidBodySimPushConstants
{
  DAXA_TH_BLOB(RigidBodySimTaskHead, task_head)
};

#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
// TODO: Add Daxa version of this?
typedef struct
{
  daxa_f32mat3x4 transform;
  daxa::u32 instance_custom_index : 24;
  daxa::u32 mask : 8;
  daxa::u32 instance_shader_binding_table_record_offset : 24;
  daxa::u32 flags : 8;
  daxa::u64 blas_device_address;
} daxa_BlasInstanceData;
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
DAXA_DECL_BUFFER_PTR(daxa_BlasInstanceData)

DAXA_DECL_TASK_HEAD_BEGIN(UpdateInstancesTaskHead)
DAXA_TH_BUFFER_PTR(TRANSFER_WRITE, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(TRANSFER_WRITE, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(TRANSFER_WRITE, daxa_RWBufferPtr(daxa_BlasInstanceData), blas_instance_data)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(Aabb), aabbs)
DAXA_DECL_TASK_HEAD_END

struct UpdateInstancesPushConstants
{
  DAXA_TH_BLOB(UpdateInstancesTaskHead, task_head)
};

#if defined(__cplusplus)
daxa_f32mat3x4 rigid_body_get_transform_matrix(const RigidBody &rigid_body)
{
  daxa_f32vec3 translation = rigid_body.position;
  daxa_f32vec4 rotation = rigid_body.rotation;

  // transform quaternion to matrix
  daxa_f32 x2 = rotation.x + rotation.x;
  daxa_f32 y2 = rotation.y + rotation.y;
  daxa_f32 z2 = rotation.z + rotation.z;
  daxa_f32 xx = rotation.x * x2;
  daxa_f32 xy = rotation.x * y2;
  daxa_f32 xz = rotation.x * z2;
  daxa_f32 yy = rotation.y * y2;
  daxa_f32 yz = rotation.y * z2;
  daxa_f32 zz = rotation.z * z2;
  daxa_f32 wx = rotation.w * x2;
  daxa_f32 wy = rotation.w * y2;
  daxa_f32 wz = rotation.w * z2;

  daxa_f32mat3x3 rotation_matrix = daxa_f32mat3x3(daxa_f32vec3(1.0f - (yy + zz), xy - wz, xz + wy),
                                                  daxa_f32vec3(xy + wz, 1.0f - (xx + zz), yz - wx),
                                                  daxa_f32vec3(xz - wy, yz + wx, 1.0f - (xx + yy)));

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

struct MyAttributes
{
  daxa_f32vec3 normal;
  daxa_f32vec3 position;
};

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

daxa_f32vec3 compute_specular(daxa_f32 shininess, daxa_f32vec3 mat_specular, daxa_f32vec3 view_dir, daxa_f32vec3 normal, daxa_f32vec3 light_dir)
{

  daxa_f32 _shininess = max(shininess, 4.0f);

  const daxa_f32 energy_conservation = (_shininess + 2.0f) / (2.0f * PI);
  daxa_f32vec3 V = normalize(-view_dir);
  daxa_f32vec3 R = reflect(-light_dir, normal);
  daxa_f32 VdotR = max(dot(V, R), 0.0f);
  daxa_f32 specular = pow(VdotR, _shininess) * energy_conservation;
  return mat_specular * specular;
}

#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG