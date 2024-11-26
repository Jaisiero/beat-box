#pragma once

#define DAXA_RAY_TRACING 1
#include "daxa/daxa.inl"
#include "daxa/utils/task_graph.inl"

#include <math.hpp>

static const daxa_f32 LINEAR_DAMPING = 0.1f;
static const daxa_f32 ANGULAR_DAMPING = 0.1f;
static const daxa_f32 POINT_SIZE = 0.01f;
static const daxa_f32 PENETRATION_FACTOR = 0.01f;
static const daxa_f32 BIAS_FACTOR = 0.2f;

#define BB_DEBUG 1
#if defined(BB_DEBUG) 
#define BB_RT_DEBUG 1  
#endif // BB_DEBUG

#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
[Flags]
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
enum RigidBodyFlag : daxa_u32 {
  NONE = 0,
  COLLIDING = 1 << 0,
  DYNAMIC = 1 << 1,
  GRAVITY = 1 << 2,
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
#elif defined(__cplusplus)
inline RigidBodyFlag operator|(RigidBodyFlag a, RigidBodyFlag b)
{
    return RigidBodyFlag((daxa_u32)a | (daxa_u32)b);
}

inline void operator|=(RigidBodyFlag &a, RigidBodyFlag b)
{
    a = a | b;
}
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG

#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
[Flags]
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
enum BoxFace : daxa_u32
{
  NO_FACE = 0,
  RIGHT = 1 << 0,
  LEFT = 1 << 1,
  TOP = 1 << 2,
  BOTTOM = 1 << 3,
  BACK = 1 << 4,
  FRONT = 1 << 5,
};

#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
[Flags]
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
enum SimFlag : daxa_u32
{
  NO_SIM_FLAG = 0,
  ACCUM_IMPULSE = 1 << 0,
  FRICTION = 1 << 1,
  WARM_STARTING = 1 << 2,
  ADVECTION = 1 << 3,
  DEBUG_INFO = 1 << 4,
};
#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
SimFlag  operator|(SimFlag a, SimFlag b)
{
    return SimFlag((daxa_u32)a | (daxa_u32)b);
}
// Define bitwise OR assignment operator
void operator|=(inout SimFlag a, SimFlag b)
{
    a = a | b;
}
// Define bitwise AND operator
SimFlag operator&(SimFlag a, SimFlag b)
{
    return SimFlag((daxa_u32)a & (daxa_u32)b);
}
// Define bitwise AND assignment operator
void operator&=(inout SimFlag a, SimFlag b)
{
    a = a & b;
}
#elif defined(__cplusplus)
inline SimFlag operator~(SimFlag a)
{
    return SimFlag(~(daxa_u32)a);
}

inline SimFlag operator|(SimFlag a, SimFlag b)
{
    return SimFlag((daxa_u32)a | (daxa_u32)b);
}

inline void operator|=(SimFlag &a, SimFlag b)
{
    a = a | b;
}

inline SimFlag operator&(SimFlag a, SimFlag b)
{
    return SimFlag((daxa_u32)a & (daxa_u32)b);
}

inline void operator&=(SimFlag &a, SimFlag b)
{
    a = a & b;
}
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG



// RigidBody struct is 156 bytes
struct RigidBody
{
  RigidBodyFlag flags;
#if defined(BB_DEBUG)
  daxa_u32 face_collided;
#endif // BB_DEBUG
  daxa_u32 primitive_count;
  daxa_u32 primitive_offset;
  daxa_f32vec3 position;
  Quaternion rotation;
  daxa_f32vec3 minimum;
  daxa_f32vec3 maximum;
  daxa_f32 mass;
  daxa_f32 inv_mass;
  daxa_f32vec3 velocity;
  daxa_f32vec3 omega;
  daxa_f32vec3 tmp_velocity;
  daxa_f32vec3 tmp_omega;
  daxa_f32mat3x3 inv_inertia;
  daxa_f32 restitution;
  daxa_f32 friction;
  // TODO: Add more rigid body properties
  // daxa_f32 drag;
  // daxa_f32 angular_drag;

  daxa_f32mat3x3 get_rotation_matrix()
  {
#if defined(__cplusplus)
    return rotation.to_matrix();
#else // defined(__cplusplus)
    return transpose(rotation.to_matrix());
    // return rotation.to_matrix();
#endif // defined(__cplusplus)
  }

  daxa_f32mat4x4 get_transform_matrix()
  {
    daxa_f32vec3 translation = position;
    daxa_f32mat3x3 rotation_matrix = rotation.to_matrix();

#if defined(__cplusplus)
    return daxa_f32mat4x4(daxa_f32vec4(rotation_matrix.x.x, rotation_matrix.y.x, rotation_matrix.z.x, translation.x),
                          daxa_f32vec4(rotation_matrix.x.y, rotation_matrix.y.y, rotation_matrix.z.y, translation.y),
                          daxa_f32vec4(rotation_matrix.x.z, rotation_matrix.y.z, rotation_matrix.z.z, translation.z),
                          daxa_f32vec4(0.0f, 0.0f, 0.0f, 1.0f));
#else // defined(__cplusplus)
    return daxa_f32mat4x4(daxa_f32vec4(rotation_matrix[0], translation.x),
                          daxa_f32vec4(rotation_matrix[1], translation.y),
                          daxa_f32vec4(rotation_matrix[2], translation.z),
                          daxa_f32vec4(0.0f, 0.0f, 0.0f, 1.0f));

#endif // defined(__cplusplus)
  }


  daxa_f32mat3x4 get_instance_transform() {
    daxa_f32mat4x4 transform = get_transform_matrix();
#if defined(__cplusplus)
    return daxa_f32mat3x4(transform.x, transform.y, transform.z);
#else // defined(__cplusplus)
  transform = transpose(transform);
  return daxa_f32mat3x4(daxa_f32vec3(transform[0].xyz), daxa_f32vec3(transform[1].xyz), daxa_f32vec3(transform[2].xyz), daxa_f32vec3(transform[3].xyz));
#endif // defined(__cplusplus)
  }

#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
  Aabb get_aabb_by_index(daxa_u32 index, Ptr<Aabb> aabbs)
  {
    return aabbs[primitive_offset + index];
  }

  [mutating] bool is_face_colliding(BoxFace face)
  {
    return (this.face_collided & face) != 0;
  }

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
  
  daxa_f32vec3 rotate_vector(const daxa_f32vec3 v)
  {
    return (rotation * Quaternion(v, 0) * rotation.conjugate()).v;
  }

  daxa_f32vec3 rotate_vector_inverse(const daxa_f32vec3 v)
  {
    return (rotation.conjugate() * Quaternion(v, 0) * rotation).v;
  }

  daxa_f32vec3 object_to_world(const daxa_f32vec3 v)
  {
    return rotate_vector(v) + this.position;
  }

  daxa_f32vec3 world_to_object(const daxa_f32vec3 v)
  {
    return rotate_vector_inverse(v - this.position);
  }
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
};
DAXA_DECL_BUFFER_PTR(RigidBody)

struct CameraView
{
  daxa_f32mat4x4 inv_view;
  daxa_f32mat4x4 inv_proj;
  daxa_f32mat4x4 view;
  daxa_f32mat4x4 proj;
};
DAXA_DECL_BUFFER_PTR(CameraView)

DAXA_DECL_BUFFER_PTR(Aabb)

DAXA_DECL_TASK_HEAD_BEGIN(RayTracingTaskHead)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ, daxa_BufferPtr(CameraView), camera)
DAXA_TH_IMAGE_ID(RAY_TRACING_SHADER_STORAGE_WRITE_ONLY, REGULAR_2D, swapchain)
DAXA_TH_TLAS_ID(RAY_TRACING_SHADER_READ, tlas)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ_WRITE, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ_WRITE, daxa_BufferPtr(Aabb), aabbs)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ_WRITE, daxa_BufferPtr(Aabb), point_aabbs)
DAXA_DECL_TASK_HEAD_END

struct RTPushConstants
{
  DAXA_TH_BLOB(RayTracingTaskHead, task_head)
};

struct GUIVertex
{
  daxa_f32vec3 position;
  daxa_f32vec3 color;
};

DAXA_DECL_BUFFER_PTR(GUIVertex)

struct GUIVertexLine
{
  daxa_f32vec3 position;
};

DAXA_DECL_BUFFER_PTR(GUIVertexLine)

DAXA_DECL_TASK_HEAD_BEGIN(GUITaskHead)
DAXA_TH_IMAGE(COLOR_ATTACHMENT, REGULAR_2D, render_target)
DAXA_TH_BUFFER_PTR(VERTEX_SHADER_READ, daxa_BufferPtr(CameraView), camera)
DAXA_TH_BUFFER_PTR(VERTEX_SHADER_READ, daxa_BufferPtr(GUIVertex), vertex_buffer)
DAXA_DECL_TASK_HEAD_END

struct GUIPushConstants
{
  DAXA_TH_BLOB(GUITaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(GUILineTaskHead)
DAXA_TH_IMAGE(COLOR_ATTACHMENT, REGULAR_2D, render_target)
DAXA_TH_BUFFER_PTR(VERTEX_SHADER_READ, daxa_BufferPtr(CameraView), camera)
DAXA_TH_BUFFER_PTR(VERTEX_SHADER_READ, daxa_BufferPtr(GUIVertexLine), vertex_buffer)
DAXA_DECL_TASK_HEAD_END

struct GUILinePushConstants
{
  DAXA_TH_BLOB(GUILineTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(GUIAxesTaskHead)
DAXA_TH_IMAGE(COLOR_ATTACHMENT, REGULAR_2D, render_target)
DAXA_TH_BUFFER_PTR(VERTEX_SHADER_READ, daxa_BufferPtr(CameraView), camera)
DAXA_TH_BUFFER_PTR(VERTEX_SHADER_READ, daxa_BufferPtr(GUIVertexLine), vertex_buffer)
DAXA_DECL_TASK_HEAD_END

struct GUIAxesPushConstants
{
  DAXA_TH_BLOB(GUIAxesTaskHead, task_head)
};

static const daxa_f32 T_MIN = 1e-3f;
static const daxa_f32 T_MAX = 1e9f;
static const daxa_f32 PI = 3.14159265359f;
static const daxa_f32 COLLISION_GUARD = 1e-3f;
static const daxa_u32 AABB_CORNER_COUNT = 8;

struct GlobalCollisionInfo
{
  daxa_u32 collision_count;
  daxa_u32 collision_point_count;
};

struct SimConfig
{
  daxa_u32 rigid_body_count;
  daxa_f32 dt;
  daxa_f32 gravity;
  SimFlag flags;
  GlobalCollisionInfo g_c_info;
#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
  [mutating] bool has_flag(SimFlag flag)
  {
    return (this.flags & flag) != 0;
  }

  [mutating] void set_flag(SimFlag flag)
  {
    this.flags |= flag;
  }

  [mutating] void clear_flag(SimFlag flag)
  {
    this.flags &= ~flag;
  }
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
};
DAXA_DECL_BUFFER_PTR(SimConfig)

struct DispatchBuffer
{
  daxa_u32vec3 dispatch;
  daxa_u32vec3 solver_dispatch;
};
DAXA_DECL_BUFFER_PTR(DispatchBuffer)

static const daxa_u32 RIGID_BODY_SIM_COMPUTE_X = 64;



DAXA_DECL_BUFFER_PTR(Manifold)


DAXA_DECL_TASK_HEAD_BEGIN(BroadPhaseTaskHead)
DAXA_TH_BUFFER_PTR(HOST_TRANSFER_WRITE, daxa_RWBufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), previous_sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(Aabb), aabbs)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Manifold), collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_RWBufferPtr(Manifold), old_collisions)
DAXA_DECL_TASK_HEAD_END

struct BroadPhasePushConstants
{
  DAXA_TH_BLOB(BroadPhaseTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(CollisionSolverDispatcherTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_DECL_TASK_HEAD_END

struct CollisionSolverDispatcherPushConstants
{
  DAXA_TH_BLOB(CollisionSolverDispatcherTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(CollisionPreSolverTaskHead)
DAXA_TH_BUFFER_PTR(READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Manifold), collisions)
DAXA_DECL_TASK_HEAD_END

struct CollisionPreSolverPushConstants
{
  DAXA_TH_BLOB(CollisionPreSolverTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(CollisionSolverTaskHead)
DAXA_TH_BUFFER_PTR(READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Manifold), collisions)
DAXA_DECL_TASK_HEAD_END

struct CollisionSolverPushConstants
{
  DAXA_TH_BLOB(CollisionSolverTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(RigidBodySimTaskHead)
DAXA_TH_BUFFER_PTR(READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(Aabb), aabbs)
DAXA_DECL_TASK_HEAD_END


struct RigidBodySimPushConstants
{
  DAXA_TH_BLOB(RigidBodySimTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(RigidBodyUpdateTaskHead)
DAXA_TH_BUFFER_PTR(READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies_update)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(GUIVertexLine), axes_vertex_buffer)
DAXA_DECL_TASK_HEAD_END


struct RigidBodyUpdatePushConstants
{
  DAXA_TH_BLOB(RigidBodyUpdateTaskHead, task_head)
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
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(TRANSFER_WRITE, daxa_RWBufferPtr(daxa_BlasInstanceData), blas_instance_data)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(Aabb), aabbs)
DAXA_DECL_TASK_HEAD_END

struct UpdateInstancesPushConstants
{
  DAXA_TH_BLOB(UpdateInstancesTaskHead, task_head)
};


DAXA_DECL_TASK_HEAD_BEGIN(CreatePointsTaskHead)
DAXA_TH_BUFFER_PTR(TRANSFER_WRITE, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_RWBufferPtr(Manifold), collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(Aabb), point_aabbs)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(GUIVertex), vertex_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(GUIVertexLine), line_vertex_buffer)
DAXA_DECL_TASK_HEAD_END

struct CreatePointsPushConstants
{
  DAXA_TH_BLOB(CreatePointsTaskHead, task_head)
};

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