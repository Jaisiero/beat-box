#pragma once

#define DAXA_RAY_TRACING 1
#include "daxa/daxa.inl"
#include "daxa/utils/task_graph.inl"

#if !defined(DAXA_SHADERLANG)
#define DAXA_SHADERLANG DAXA_LANGUAGE
#endif

#if !defined(DAXA_SHADERLANG_SLANG)
#define DAXA_SHADERLANG_SLANG DAXA_LANGUAGE_SLANG
#endif

#if defined(__cplusplus)
namespace daxa
{
namespace TaskAccessConsts
{
static constexpr TaskAccess RAY_TRACING_SHADER_READ = RAY_TRACING_SHADER::READ;
static constexpr TaskAccess RAY_TRACING_SHADER_READ_WRITE = RAY_TRACING_SHADER::READ_WRITE;
static constexpr TaskAccess RAY_TRACING_SHADER_STORAGE_READ_ONLY = RAY_TRACING_SHADER::READ;
static constexpr TaskAccess RAY_TRACING_SHADER_STORAGE_WRITE_ONLY = RAY_TRACING_SHADER::WRITE;
static constexpr TaskAccess VERTEX_SHADER_READ = VERTEX_SHADER::READ;
static constexpr TaskAccess COMPUTE_SHADER_READ = COMPUTE_SHADER::READ;
static constexpr TaskAccess COMPUTE_SHADER_READ_WRITE = COMPUTE_SHADER::READ_WRITE;
static constexpr TaskAccess COMPUTE_SHADER_READ_WRITE_CONCURRENT = COMPUTE_SHADER::READ_WRITE_CONCURRENT;
static constexpr TaskAccess COMPUTE_SHADER_STORAGE_WRITE_ONLY = COMPUTE_SHADER::WRITE;
static constexpr TaskAccess COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ = COMPUTE_SHADER::READ | INDIRECT_COMMAND_READ;
static constexpr TaskAccess TRANSFER_READ = TRANSFER::READ;
static constexpr TaskAccess TRANSFER_WRITE = TRANSFER::WRITE;
static constexpr TaskAccess TRANSFER_READ_WRITE = TRANSFER::READ_WRITE;
static constexpr TaskAccess BUILD_READ = ACCELERATION_STRUCTURE_BUILD::READ;
static constexpr TaskAccess BUILD_WRITE = ACCELERATION_STRUCTURE_BUILD::WRITE;
} // namespace TaskAccessConsts

struct TaskBufferAccess
{
  static constexpr TaskAccess TRANSFER_READ = TaskAccessConsts::TRANSFER::READ;
  static constexpr TaskAccess TRANSFER_WRITE = TaskAccessConsts::TRANSFER::WRITE;
  static constexpr TaskAccess TRANSFER_READ_WRITE = TaskAccessConsts::TRANSFER::READ_WRITE;
  static constexpr TaskAccess COMPUTE_SHADER_READ = TaskAccessConsts::COMPUTE_SHADER::READ;
  static constexpr TaskAccess COMPUTE_SHADER_READ_WRITE = TaskAccessConsts::COMPUTE_SHADER::READ_WRITE;
  static constexpr TaskAccess INDIRECT_COMMAND_READ = TaskAccessConsts::INDIRECT_COMMAND_READ;
  static constexpr TaskAccess COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ = TaskAccessConsts::COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ;
  static constexpr TaskAccess BUILD_READ = TaskAccessConsts::ACCELERATION_STRUCTURE_BUILD::READ;
  static constexpr TaskAccess BUILD_WRITE = TaskAccessConsts::ACCELERATION_STRUCTURE_BUILD::WRITE;
  static constexpr TaskAccess RAY_TRACING_SHADER_READ = TaskAccessConsts::RAY_TRACING_SHADER::READ;
  static constexpr TaskAccess RAY_TRACING_SHADER_READ_WRITE = TaskAccessConsts::RAY_TRACING_SHADER::READ_WRITE;
};

struct TaskImageAccess
{
  static constexpr TaskAccess TRANSFER_READ = TaskAccessConsts::TRANSFER::READ;
  static constexpr TaskAccess TRANSFER_WRITE = TaskAccessConsts::TRANSFER::WRITE;
  static constexpr TaskAccess TRANSFER_READ_WRITE = TaskAccessConsts::TRANSFER::READ_WRITE;
  static constexpr TaskAccess COMPUTE_SHADER_STORAGE_WRITE_ONLY = TaskAccessConsts::COMPUTE_SHADER::WRITE;
  static constexpr TaskAccess RAY_TRACING_SHADER_STORAGE_WRITE_ONLY = TaskAccessConsts::RAY_TRACING_SHADER::WRITE;
};

struct TaskBlasAccess
{
  static constexpr TaskAccess BUILD_READ = TaskAccessConsts::ACCELERATION_STRUCTURE_BUILD::READ;
  static constexpr TaskAccess BUILD_WRITE = TaskAccessConsts::ACCELERATION_STRUCTURE_BUILD::WRITE;
};

struct TaskTlasAccess
{
  static constexpr TaskAccess BUILD_READ = TaskAccessConsts::ACCELERATION_STRUCTURE_BUILD::READ;
  static constexpr TaskAccess BUILD_WRITE = TaskAccessConsts::ACCELERATION_STRUCTURE_BUILD::WRITE;
};
}
#endif

#include <math.hpp>

static const daxa_f32 LINEAR_DAMPING = 0.1f;
static const daxa_f32 ANGULAR_DAMPING = 0.1f;
static const daxa_f32 POINT_SIZE = 0.01f;
static const daxa_f32 MIN_CONTACT_HERTZ = 30.0f;
static const daxa_f32 _PI = 3.14159265359f;
static const daxa_f32 PENETRATION_FACTOR = 0.01f;
static const daxa_f32 BIAS_FACTOR = 0.2f;

#define BB_DEBUG 1
#if defined(BB_DEBUG)
// #define BB_RT_DEBUG 1
// #define BB_SIM_DEBUG 1
#endif // BB_DEBUG

#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
[Flags]
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
enum RigidBodyFlag : daxa_u32 {
  NONE = 0,
  COLLIDING = 1 << 0,
  DYNAMIC = 1 << 1,
  GRAVITY = 1 << 2,
  SLEEPING = 1 << 3, // island at rest: advect/solve/integrate skip this body until its island wakes
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
  USE_GRAPH_COLORING = 1 << 5, // solve contacts by graph color (parallel) instead of by contact-island (serial-per-island)
  DEBUG_GRAPH_COLORS = 1 << 6, // tint contact debug geometry by each manifold's graph color
  SLEEPING_ENABLED = 1 << 7,   // island sleeping: resting islands stop advecting/solving/integrating
  DETERMINISTIC = 1 << 8,      // cross-launch reproducibility mode: skip the alpha=0 post-stab (its
                               // over-constrained depenetration has no unique fixed point, so its
                               // layout-dependent per-sweep noise can't converge out -> ph=cph, which
                               // IS bitwise-deterministic across launches). Trades depenetration depth
                               // for reproducibility; the converged main solve + rest state are unaffected.
  DET_HASHES = 1 << 9,         // compute the determinism debug pose/state hashes (dbg_*). OFF in normal
                               // runs so the per-body/per-manifold InterlockedXor + the chain-walk hash
                               // don't add hot-path atomic contention; set by main.cpp when any of
                               // BB_DET_STEPS / BB_DET_INPROC / BB_DETERMINISTIC is active (the harness).
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


enum SimSolverType : daxa_u32
{
  PGS = 0,
  PGS_SOFT = 1,
  AVBD = 2, // Augmented Vertex Block Descent: per-body 6x6 block descent + augmented Lagrangian
  TGS_SOFT = 3, // Box2D v3 / solver2d: sub-stepped soft constraints (warm start + relax + separation tracking)
  INVALID_SOLVER = 0xFFFFFFFF,
};






struct Material {
  daxa_f32vec3 albedo;
  daxa_f32vec3 emission;
};
DAXA_DECL_BUFFER_PTR(Material)

struct Light {
  daxa_u32 rigid_body_id;
};
DAXA_DECL_BUFFER_PTR(Light)


#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
[Flags]
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
enum RayTracingFlag : daxa_u32
{
  RT_NONE = 0,
  RT_ACCUMULATE = 1 << 0,
  RT_SHOW_NORMALS = 1 << 1,
  RT_SHOW_ISLANDS = 1 << 2,
  RT_SHOW_COLLISIONS = 1 << 3,
};
#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
RayTracingFlag  operator|(RayTracingFlag a, RayTracingFlag b)
{
    return RayTracingFlag((daxa_u32)a | (daxa_u32)b);
}
// Define bitwise OR assignment operator
void operator|=(inout RayTracingFlag a, RayTracingFlag b)
{
    a = a | b;
}
// Define bitwise AND operator
RayTracingFlag operator&(RayTracingFlag a, RayTracingFlag b)
{
    return RayTracingFlag((daxa_u32)a & (daxa_u32)b);
}
// Define bitwise AND assignment operator
void operator&=(inout RayTracingFlag a, RayTracingFlag b)
{
    a = a & b;
}
#elif defined(__cplusplus)
inline RayTracingFlag operator~(RayTracingFlag a)
{
    return RayTracingFlag(~(daxa_u32)a);
}

inline RayTracingFlag operator|(RayTracingFlag a, RayTracingFlag b)
{
    return RayTracingFlag((daxa_u32)a | (daxa_u32)b);
}

inline void operator|=(RayTracingFlag &a, RayTracingFlag b)
{
    a = a | b;
}

inline RayTracingFlag operator&(RayTracingFlag a, RayTracingFlag b)
{
    return RayTracingFlag((daxa_u32)a & (daxa_u32)b);
}

inline void operator&=(RayTracingFlag &a, RayTracingFlag b)
{
    a = a & b;
}
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG

struct RayTracingConfig {
  RayTracingFlag flags;
  daxa_u32 max_bounces;
  daxa_u64 current_frame_index;
  daxa_u64 frame_count;
  daxa_u32 light_count;
  daxa_u32 instance_count;
#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG

  [mutating] bool has_flag(RayTracingFlag flag)
  {
    return (this.flags & flag) != 0;
  }

  [mutating] void set_flag(RayTracingFlag flag)
  {
    this.flags |= flag;
  }

  [mutating] void clear_flag(RayTracingFlag flag)
  {
    this.flags &= ~flag;
  }
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
};
DAXA_DECL_BUFFER_PTR(RayTracingConfig)


// In Slang, a non-[mutating] method receives `this` BY VALUE: a whole-struct load
// through the daxa buffer pointer. Daxa 3.6 lays that load out with vec3 padded to
// 16 bytes, so every field after the first vec3 (rotation, minimum, maximum,
// velocity, omega, inv_inertia) is read from the wrong offset -> corrupted rotation
// matrices, extents and inertia (breaking narrow-phase SAT and the solver).
// BB_REF marks a read-only method [mutating] so `this` is passed by reference and
// fields are read with the correct scalar layout. In C++ the attribute is dropped
// (C++ has no such layout hazard and these methods are already non-const).
#if defined(__cplusplus)
#define BB_REF
#else
#define BB_REF [mutating]
#endif

struct RigidBody
{
  daxa_u32 id;
  RigidBodyFlag flags;
  daxa_u32 island_index;
  daxa_u32 active_index;
  daxa_u32 manifold_node_index;
  daxa_u32 sleep_timer; // consecutive sim steps below the sleep velocity thresholds
  daxa_u32 material_index;
#if defined(BB_DEBUG)
  daxa_u32 face_collided;
#endif // BB_DEBUG
  daxa_u32 primitive_count;
  daxa_u32 primitive_offset;
  // voxel collision shape: 0 = none (legacy OBB body), otherwise voxel_shapes[shape_index - 1]
  daxa_u32 shape_index;
  daxa_f32vec3 position;
  // INVARIANT: unit quaternion (|rotation| == 1). Both rotation->matrix paths assume it:
  // Quaternion::to_matrix() (math.hpp) and the rotate_vector() sandwich below. A non-unit value makes
  // them silently diverge (each scales differently by |q|^2) -> at-rest render mismatch. The invariant
  // is established at scene upload (AccelerationStructureManager::build_accel_structs) and maintained
  // by integrate_positions() (extensions.slang), which renormalizes after every step.
  Quaternion rotation;
  daxa_f32vec3 minimum;
  daxa_f32vec3 maximum;
  daxa_f32 mass;
  daxa_f32 inv_mass;
  daxa_f32vec3 velocity;
  daxa_f32vec3 omega;
  daxa_f32vec3 prev_velocity; // last step's velocity (AVBD adaptive warm start; rides the row reorder)
  daxa_f32vec3 prev_omega;    // last step's angular velocity (E2: exact step-entry approach vel for the impact classifier; rides the row reorder next to prev_velocity)
  daxa_f32mat3x3 inv_inertia;
  daxa_f32 restitution;
  daxa_f32 friction;
  // TODO: Add more rigid body properties
  // daxa_f32 drag;
  // daxa_f32 angular_drag;

  BB_REF daxa_f32mat3x3 get_rotation_matrix()
  {
#if defined(__cplusplus)
    return rotation.to_matrix();
#else // defined(__cplusplus)
    return transpose(rotation.to_matrix());
    // return rotation.to_matrix();
#endif // defined(__cplusplus)
  }

  BB_REF daxa_f32mat4x4 get_transform_matrix()
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

  BB_REF daxa_f32 get_half_size(daxa_u32 index)
  {
    daxa_f32vec3 size = (maximum - minimum) * 0.5f;
    return index == 0 ? size.x : (index == 1 ? size.y : size.z);
  }

  BB_REF daxa_f32vec3 get_center() {
    return (maximum + minimum) * 0.5f;
  }

  BB_REF daxa_f32vec3 half_extent() {
    return (maximum - minimum) * 0.5f;
  }


  BB_REF daxa_f32mat3x4 get_instance_transform() {
    daxa_f32mat4x4 transform = get_transform_matrix();
#if defined(__cplusplus)
    return daxa_f32mat3x4(transform.x, transform.y, transform.z);
#else // defined(__cplusplus)
  transform = transpose(transform);
  return daxa_f32mat3x4(daxa_f32vec3(transform[0].xyz), daxa_f32vec3(transform[1].xyz), daxa_f32vec3(transform[2].xyz), daxa_f32vec3(transform[3].xyz));
#endif // defined(__cplusplus)
  }

#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
  BB_REF Aabb get_aabb_by_index(daxa_u32 index, Aabb* aabbs)
  {
    Aabb aabb;
    daxa_u32 aabb_index = primitive_offset + index;
    aabb.minimum = aabbs[aabb_index].minimum;
    aabb.maximum = aabbs[aabb_index].maximum;
    return aabb;
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
  
  // [mutating] => `this` is passed by reference, not by value. A by-value `this`
  // triggers a whole-struct load through the daxa buffer pointer, which Daxa 3.6
  // pads vec3->16 (corrupting rotation/position). By-ref reads fields with the
  // correct scalar layout. (Method does not actually mutate.)
  //
  // PRECONDITION: |rotation| == 1 (see the rotation field's invariant). The sandwich q*.v.q scales the
  // result by |q|^2 for a non-unit q, so it would diverge from to_matrix(). Deliberately NOT normalized
  // here: this is on the per-pair collision hot path (collision_detection.slang) and per-ray in the path
  // tracer (ray_tracing.slang); the invariant already holds upstream, so a normalize() would be pure cost.
  [mutating] daxa_f32vec3 rotate_vector(const daxa_f32vec3 v)
  {
    return (rotation * Quaternion(v, 0) * rotation.conjugate()).v;
  }

  [mutating] daxa_f32vec3 rotate_vector_inverse(const daxa_f32vec3 v)
  {
    return (rotation.conjugate() * Quaternion(v, 0) * rotation).v;
  }

  [mutating] daxa_f32vec3 object_to_world(const daxa_f32vec3 v)
  {
    return rotate_vector(v) + this.position;
  }

  [mutating] daxa_f32vec3 world_to_object(const daxa_f32vec3 v)
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
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ, daxa_BufferPtr(RayTracingConfig), ray_tracing_config)
DAXA_TH_IMAGE_ID(RAY_TRACING_SHADER_STORAGE_WRITE_ONLY, REGULAR_2D, swapchain)
DAXA_TH_IMAGE_ID(RAY_TRACING_SHADER_STORAGE_READ_ONLY, REGULAR_2D, accumulation_buffer)
DAXA_TH_TLAS_ID(RAY_TRACING_SHADER_READ, tlas)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ, daxa_BufferPtr(RigidBodyEntry), rigid_body_map)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ_WRITE, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ, daxa_BufferPtr(Aabb), aabbs)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ, daxa_BufferPtr(Aabb), lbvh_nodes)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ, daxa_BufferPtr(Light), lights)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ, daxa_BufferPtr(Material), materials)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ, daxa_BufferPtr(Island), islands)
DAXA_TH_BUFFER_PTR(RAY_TRACING_SHADER_READ, daxa_BufferPtr(ContactIsland), contact_islands)
DAXA_TH_IMAGE_ID(RAY_TRACING_SHADER_STORAGE_READ_ONLY, REGULAR_3D, stbn_texture)
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
  SimSolverType solver_type;
  daxa_u32 rigid_body_count;
  daxa_u32 active_rigid_body_count;
  daxa_u32 island_count; // atomic add
  daxa_u32 contact_island_count; // atomic add
  daxa_u32 manifold_node_count; // atomic add
  daxa_u32 broad_phase_collision_count; // atomic add
  daxa_u32 radix_shift;
  daxa_u32 graph_color_count;      // graph-coloring: # colors used this frame (debug/validator)
  daxa_u32 graph_color_violations; // graph-coloring: validator invariant violations (must be 0)
  daxa_u32 graph_color_overflow;   // graph-coloring: # contacts the per-color solver skips (uncolored OR color>=BB_MAX_COLORS_SOLVE)
  daxa_u32 dbg_contact_overflow;   // narrow phase: # collisions DROPPED because g_c_info.collision_count hit BB_MAX_COLLISION_COUNT (0 = healthy; >0 = manifolds silently lost -> a body can lose support and tunnel)
  daxa_u32 dbg_node_overflow;      // narrow phase: # manifold-link nodes DROPPED because manifold_node_count hit BB_MAX_MANIFOLD_NODE_COUNT (0 = healthy; >0 = a body's manifold list truncated -> missed contacts)
  daxa_u32 gc_round;               // graph-coloring: current round index (incremented by owner_reset; seeds the fair-arbitration priority)
  daxa_u32 sleeping_count;         // neighborhood sleeping: # bodies currently asleep (diagnostics; recomputed per step)
  daxa_u32 avbd_color_count;       // AVBD: # body colors used this step (validator)
  daxa_u32 avbd_max_support_depth; // AVBD: max support-depth layer over dynamic bodies this step (post-BFS, clamped to SHOCK_LAYERS-1); the post-stab cascade skips layers above this
  daxa_u32 avbd_iter_tick;         // AVBD convergence early-out: main-sweep iteration counter, ticked by the dual pass (thread 0) between primal sweeps
  daxa_u32 avbd_step_res[2];       // AVBD: max primal step magnitude of main-sweep iteration (tick&1), stored as ordered float bits (asuint of a non-negative f32 compares like the float). When the PREVIOUS iteration's max step is below BB_AVBD_CONV_EPS the system converged: remaining main sweeps early-return (rest converges in a few of the 10 iterations; impacts keep the full budget - quality by construction)
  daxa_u32 avbd_violations;        // AVBD: body-coloring invariant violations (adjacent same color; must be 0)
  daxa_u32 avbd_stick_count;       // AVBD: # contacts whose sticking anchors were reused this step (diagnostics)
  daxa_u32 gc_max_degree;          // graph-coloring DIAG: max colored-degree = max popcount(body_color_mask) over bodies
  daxa_u32 gc_max_degree_body;     // graph-coloring DIAG: a body index whose mask saturated (popcount>=30)
  daxa_u32 gc_max_degree_flags;    // graph-coloring DIAG: that body's RigidBodyFlag bits as seen by the validator
  daxa_u32 gc_satbody_degree;      // graph-coloring DIAG: # manifolds referencing the saturated body (true degree)
  daxa_u32 gc_satbody_uncolored;   // graph-coloring DIAG: how many of those are uncolored
  daxa_u32 gc_satbody_pmin;        // graph-coloring DIAG: min partner body index over those manifolds
  daxa_u32 gc_satbody_pmax;        // graph-coloring DIAG: max partner body index over those manifolds
  daxa_u32 gc_sat_nanflags;        // graph-coloring DIAG: bit0 pos-NaN, bit1 rot-NaN, bit2 vel-NaN, bit3 omega-NaN, bit4 rot-denormalized
  daxa_f32 gc_sat_pos_y;           // graph-coloring DIAG: saturated body's position.y (sanity)
  // ---- velocity-explosion latch DIAG: records the FIRST body whose |v| exceeds the threshold,
  // tagged with the pipeline stage that first observed it (CAS once, never reset during a run):
  //   stage 1 = pre-solve probe (validator, after advect)  -> injected outside the solver
  //   stage 2 = integrate-positions probe (after CS sweep) -> injected by warm-start/CS
  //   stage 3 = end-of-frame probe (after CSR relax sweep) -> injected by the relax sweep
  daxa_u32 dbg_ex_stage;
  daxa_u32 dbg_ex_body;
  daxa_u32 dbg_ex_frame;
  daxa_f32 dbg_ex_vel;
  daxa_f32 dbg_ex_y;
  daxa_f32 dbg_ex_vy;
  daxa_u32 dbg_maxv;               // per-frame max |v| over dynamic bodies, integer mm/s (reset each frame)
  daxa_u32 dbg_id_sum;             // per-frame sum of body ids over all rows; constant unless the
                                   // sort/reorder permutation duplicates one row and drops another
  daxa_u32 dbg_fresh;              // per-frame contacts whose identity failed warm-start matching
                                   // (warm_start==0 = lambda reset; nonzero at rest = contact churn)
  daxa_u32 dbg_fresh_tag;          // one churning manifold: rbaIdx<<22 | rbbIdx<<12 | key<<4 | n
  daxa_u32 dbg_pen;                // per-frame deepest contact penetration in integer mm:
                                   // standing nonzero value at rest = frozen interpenetration
  daxa_u32 dbg_pad1;               // fa churn-anatomy latch (fresh|new_cc|had_old|old_cc|pair)
  // DEEP-MISS probe: a pair >=10cm interpenetrated whose previous-frame manifold cannot
  // be found is the open existence-flicker anomaly. First event per frame latches the
  // persistent body ids and a diagnostic RE-WALK of BOTH bodies' previous manifold
  // chains (reason<<30 | found_any_order<<29 | steps<<16 | other_pair_manifolds_seen).
  daxa_u32 dbg_dm_count;
  daxa_u32 dbg_np_processed;       // narrow-phase COVERAGE: threads that passed the bounds
                                   // check this frame; < broad_phase_collision_count on any
                                   // frame = indirect dispatch truncation (measured: never)
  daxa_u32 dbg_dm_mon;             // per-frame MONITOR bits for the latched dm pair:
                                   // 1 = an NP thread processed the pair this frame,
                                   // 2 = SAT produced a manifold, 4 = warm-start matched.
                                   // Anomaly frames read 0 (broad miss!) or 1 (SAT false)
  daxa_u32 dbg_poshash;            // DETERMINISM gauge: XOR of asuint(position.xyz) over all
                                   // dynamic bodies (XOR = exactly order-independent). Two
                                   // same-seed runs must produce identical values at rest;
                                   // the first diverging frame localizes residual
                                   // nondeterminism. Constant once the pile fully sleeps.
  daxa_u32 dbg_rothash;            // same, over asuint(rotation.xyzw)
  daxa_u32 dbg_cp_poshash;         // DETERMINISM CHECKPOINT (debug): same XOR hash but written at
  daxa_u32 dbg_cp_rothash;         // entry_avbd_finalize (after the main primal/dual sweeps, BEFORE
                                   // impact+post-stab). cp diverges with poshash => main solve is the
                                   // source; cp matches but poshash diverges => impact/post-stab is.
  daxa_u32 dbg_cp2_poshash;        // SECOND checkpoint at entry_avbd_prepare (post free-fall predict,
  daxa_u32 dbg_cp2_rothash;        // PRE-primal). cp2 matches but cp diverges => the primal sweeps.
  daxa_u32 dbg_vh_fin;             // velocity+omega hash at FIN end (PRE-impact): diverges => FIN/sponge
  daxa_u32 dbg_vh_imp;             // velocity+omega hash at impact-apply end (POST-impact): => impact
  daxa_u32 dbg_color_hash;         // XOR hash of body_color (entry_avbd_color_validate): diverges =>
  daxa_u32 dbg_color_pad;          // (repurposed = full contact-geometry lh hash, computed in IMPJ)
  daxa_u32 dbg_state_hash;         // XOR hash of avbd_state (pos_tilde+support_depth) — un-hashed
  daxa_u32 dbg_state_pad;          // read of the post-stab; diverges => avbd_state is process-specific
                                   // --- per-frame reset boundary (see reset_fresh array) ---
  daxa_u32 dbg_dm_ids;             // PERSISTENT: first deep-MISS pair ever ((idA<<16)|idB)
  daxa_u32 dbg_dm_walk_a;          // PERSISTENT: that event's chain-walk forensics
  daxa_u32 dbg_dm_walk_b;
  // DEEP-POCKET TRACE (entry_avbd_pocket_trace): per-frame latch of the deepest awake
  // contact AFTER the full solve, dumped per-frame to a CSV. De-confounds the oscillator's
  // lambda (captured at end-of-solve, not the fastest-body flight phase) to find the pump.
  daxa_u32 dbg_pk_pen;             // deepest awake contact depth this frame, integer mm (reset each frame)
  daxa_u32 dbg_pk_body;           // its pair ((b1<<16)|(b2&0xFFFF))
  daxa_f32 dbg_pk_lambda;         // its normal_impulse (lambda, <=0) after the last dual iteration
  daxa_f32 dbg_pk_k;              // its mass_coefficient (penalty stiffness k)
  daxa_f32 dbg_pk_vn;             // its normal relative velocity post-FIN+impact (m/s; >0 separating)
  daxa_u32 dbg_pk_stick;          // (contact_count<<1) | stick
  daxa_u32 dbg_pk_omega;          // per-frame max |omega| over contacting awake bodies, integer mrad/s
  daxa_u32 dbg_min_y;             // per-frame LOWEST dynamic-body y, encoded (y+100)*1000 (reset MAX_U32); floor top=0, cube rests at y=0.5
  daxa_u32 dbg_deep100;           // per-frame count of contacts penetrating > 100 mm
  daxa_u32 dbg_deep200;           // per-frame count of contacts penetrating > 200 mm
  daxa_u32 dbg_vox_wedge;         // per-frame count of thin-feature PASS-THROUGH WEDGES resolved:
                                  // an axis pair whose +/- manifolds BOTH ran deep (opposing forces
                                  // cancelling = stable invisible interpenetration); the deeper side
                                  // is suppressed so the shallower one extracts. scene_5 wedge fix.
  daxa_u32 dbg_vox_interior;      // per-frame MAX of a voxel manifold's interior_hits: samples fully
                                  // surrounded by the OTHER body's solid = a body is EMBEDDED inside
                                  // another. Deep voxel-voxel overlap is otherwise INVISIBLE to
                                  // dbg_pen (interior samples emit no contact; boundary samples
                                  // measure cell-local depth <= 1 cell). scene_5 investigation.
  daxa_f32 dt;
  daxa_f32 gravity;
  SimFlag flags;
  GlobalCollisionInfo g_c_info;
  // Keep the daxa_u64 LAST: a u64 in the middle forces 8-byte alignment whose
  // padding Slang's scalar layout and C++ can disagree on, shifting every field
  // after it (g_c_info.collision_count in particular). With the u64 last, all
  // count fields + g_c_info land at offsets shared by C++ and the shader.
  daxa_u64 frame_count;
  // voxel collision shape pools as raw device addresses (the buffers are written once by
  // the CPU at scene load and never touched by the GPU afterwards, so they can safely
  // bypass the task graph - the NarrowPhase head sits exactly at the 128B push limit)
  daxa_u64 voxel_shapes_addr;
  daxa_u64 voxel_occupancy_addr;
  daxa_u64 voxel_surface_addr;
  daxa_u64 voxel_sdf_addr;
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

#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
// Velocity-explosion latch probe (DIAG). Threshold 50 m/s: settled bodies are ~0 m/s and the
// initial 7 m drop peaks at ~12 m/s, so anything above is solver-injected energy.
static const daxa_f32 BB_DBG_EXPLODE_VEL2 = 2500.0f;

// Far-contact latch probe (DIAG), stage 6: fires when a contact point lies further than 100 units
// from its manifold anchor. Boxes are ~1 unit (floor 100), so a farther point means the edge-edge
// closest-point solve was ill-conditioned (nearly parallel edges -> denominator ~ sin^2(theta)).
// Such a lever arm turns a normal warm-start kick into a huge angular impulse.
static const daxa_f32 BB_DBG_FAR_CONTACT2 = 10000.0f;
void bb_dbg_far_contact_probe(SimConfig* sc, daxa_u32 obb1, daxa_u32 obb2, daxa_f32vec3 rel, daxa_f32 pen, daxa_f32 normal_y)
{
  daxa_f32 r2 = dot(rel, rel);
  if (r2 > BB_DBG_FAR_CONTACT2)
  {
    daxa_u32 prev;
    InterlockedCompareExchange(sc->dbg_ex_stage, 0u, 6u, prev);
    if (prev == 0u)
    {
      sc->dbg_ex_body = obb1;
      sc->dbg_ex_frame = daxa_u32(sc->frame_count) | (obb2 << 16);
      sc->dbg_ex_vel = sqrt(r2); // |contact - anchor|
      sc->dbg_ex_y = pen;
      sc->dbg_ex_vy = normal_y;
    }
  }
}

// Accumulated-impulse latch probe (DIAG), stage 4: fires when a warm-started/accumulated contact
// impulse exceeds the threshold (a settled contact carries ~mass*g*dt/contact ~ 0.2 Ns; thousands
// means a runaway accumulator). Reuses the same latch slots: body<-obb1, vel<-|impulse|,
// y<-penetration, vy<-normal.y of the offending contact.
static const daxa_f32 BB_DBG_EXPLODE_IMPULSE = 2000.0f;
void bb_dbg_impulse_probe(SimConfig* sc, daxa_u32 stage, daxa_u32 obb1, daxa_u32 obb2, daxa_f32 ni, daxa_f32 pen, daxa_f32 normal_y)
{
  if (abs(ni) > BB_DBG_EXPLODE_IMPULSE)
  {
    daxa_u32 prev;
    InterlockedCompareExchange(sc->dbg_ex_stage, 0u, stage, prev);
    if (prev == 0u)
    {
      sc->dbg_ex_body = obb1;
      sc->dbg_ex_frame = daxa_u32(sc->frame_count) | (obb2 << 16); // pack partner in high bits (frames < 65536 in a run)
      sc->dbg_ex_vel = ni;
      sc->dbg_ex_y = pen;
      sc->dbg_ex_vy = normal_y;
    }
  }
}
void bb_dbg_velocity_probe(SimConfig* sc, daxa_u32 stage, daxa_u32 body, daxa_f32vec3 v, daxa_f32 y)
{
  daxa_f32 v2 = dot(v, v);
  daxa_u32 prev;
  // mm/s so resting JITTER is visible (integer m/s truncated everything below 1 m/s)
  InterlockedMax(sc->dbg_maxv, daxa_u32(min(sqrt(v2) * 1000.0f, 1.0e9f)), prev);
  // lowest dynamic-body y (encoded (y+100)*1000 so InterlockedMin works on a positive uint):
  // tells us if the pile bottom sinks toward/under the floor (rest y=0.5, floor top y=0).
  daxa_u32 enc = daxa_u32(clamp((y + 100.0f) * 1000.0f, 0.0f, 4.2e9f));
  InterlockedMin(sc->dbg_min_y, enc, prev);
  // DIAG stage 9: the EXACT condition that makes miny read -100 is enc==0 (an INTEGER test, immune
  // to fast-math / no-NaN folding that kills isnan()). enc==0 means y<=-100 (real escape) OR y is
  // NaN/Inf (uint(NaN)=0). Record asuint(y)'s exponent byte to tell them apart: 255 => NaN/Inf,
  // else a finite escape. dbg_ex_vel = exponent byte; dbg_ex_vy = mantissa (nonzero => NaN, 0 => Inf).
  if (enc == 0u)
  {
    daxa_u32 prevn;
    InterlockedCompareExchange(sc->dbg_ex_stage, 0u, 9u, prevn);
    if (prevn == 0u)
    {
      daxa_u32 ybits = asuint(y);
      sc->dbg_ex_body = body;
      sc->dbg_ex_frame = daxa_u32(sc->frame_count);
      sc->dbg_ex_vel = daxa_f32((ybits >> 23u) & 0xFFu);
      sc->dbg_ex_y = y;
      sc->dbg_ex_vy = daxa_f32(ybits & 0x007FFFFFu);
    }
  }
  if (v2 > BB_DBG_EXPLODE_VEL2)
  {
    InterlockedCompareExchange(sc->dbg_ex_stage, 0u, stage, prev);
    if (prev == 0u) // we are the first observer: record the event context (single writer)
    {
      sc->dbg_ex_body = body;
      sc->dbg_ex_frame = daxa_u32(sc->frame_count);
      sc->dbg_ex_vel = sqrt(v2);
      sc->dbg_ex_y = y;
      sc->dbg_ex_vy = v.y;
    }
  }
}
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG

static const daxa_u32 RIGID_BODY_DISPATCH_COUNT_OFFSET = 0;
static const daxa_u32 ISLAND_DISPATCH_COUNT_OFFSET = 1;
static const daxa_u32 ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET = 2;
static const daxa_u32 COLLISION_DISPATCH_COUNT_OFFSET = 3;
static const daxa_u32 CONTACT_ISLAND_DISPATCH_COUNT_OFFSET = 4;
static const daxa_u32 RADIX_SORT_RIGID_BODY_DISPATCH_COUNT_OFFSET = 5;
static const daxa_u32 NARROW_PHASE_COLLISION_DISPATCH_COUNT_OFFSET = 6;
static const daxa_u32 GRAPH_COLOR_DISPATCH_COUNT_OFFSET = 7; // graph-coloring passes over manifolds (ceil(collision_count/X))
static const daxa_u32 GRAPH_COLOR_SOLVE_DISPATCH_OFFSET = 8; // per-color solve dispatch array starts here (color c at offset 8+c)
static const daxa_u32 AVBD_COLOR_SOLVE_DISPATCH_OFFSET = GRAPH_COLOR_SOLVE_DISPATCH_OFFSET + 32; // per-color AVBD PRIMAL dispatch array (body color c at offset +c); empty body colors get 0 workgroups
static const daxa_u32 AVBD_CASCADE_DISPATCH_OFFSET = AVBD_COLOR_SOLVE_DISPATCH_OFFSET + 32; // per-(layer,color) AVBD post-stab CASCADE dispatch array; entry [d*32+c]; layers d > avbd_max_support_depth get 0 workgroups

struct DispatchBuffer
{
  daxa_u32vec3 rigid_body_dispatch;
  daxa_u32vec3 island_dispatch;
  daxa_u32vec3 active_rigid_body_dispatch;
  daxa_u32vec3 collision_dispatch;
  daxa_u32vec3 contact_island_dispatch;
  daxa_u32vec3 radix_sort_rigid_body_dispatch;
  daxa_u32vec3 narrow_phase_dispatch;
  daxa_u32vec3 graph_color_dispatch; // over manifolds (collision_count) for the coloring assign/validate passes
  daxa_u32vec3 graph_color_solve_dispatch[32]; // [BB_MAX_COLORS, defined below] per-color solve dispatch: USED colors get
                                                          // ceil(coll/X), unused colors get 0 workgroups
                                                          // (a dense pile uses ~6-12 of 32 colors, so the
                                                          // rest cost nothing instead of full-count early-out)
  daxa_u32vec3 avbd_color_dispatch[32]; // per-color AVBD PRIMAL dispatch (same idea as above but per-BODY):
                                                    // used body colors (c < avbd_color_count = max(body_color)+1) get
                                                    // ceil(rigid_body_count/X), the rest get 0 (a pile uses ~7 of 32)
  daxa_u32vec3 avbd_cascade_dispatch[12 * 32]; // per-(layer,color) post-stab CASCADE dispatch [d*32+c]; 12 = BB_AVBD_SHOCK_LAYERS,
                                                          // 32 = BB_MAX_COLORS (both defined below). entry = (d <= avbd_max_support_depth
                                                          // && c < avbd_color_count) ? ceil(rigid_body_count/X) : 0 — a 2-4 layer pile
                                                          // skips layers 4..11 entirely (subsumes the per-color skip for the cascade)
};
DAXA_DECL_BUFFER_PTR(DispatchBuffer)

static const daxa_u32 RIGID_BODY_SIM_COMPUTE_X = 32;
static const daxa_u32 RADIX_SORT_WORKGROUP_SIZE = 256; // assert WORKGROUP_SIZE >= RADIX_SORT_BINS
// Define similar constants and thread group size
static const daxa_u32 NUM_BLOCKS_PER_WORKGROUP = 32; 
static const daxa_u32 RADIX_SORT_BINS = 256;
static const daxa_u32 SUBGROUP_SIZE = 32; // 32 NVIDIA; 64 AMD
static const daxa_u32 BITS = 32;          // sorting daxa_u32s
static const daxa::u32 ITERATIONS = 4;    // 4 iterations for 32-bit daxa_u32s
static const daxa::u32 BIT_SHIFT = BITS / ITERATIONS; // 8

static const daxa_u32 BB_MAX_RIGID_BODY_COUNT = 1024;
// voxel shapes (prototype scale: grids up to ~32 per axis)
static const daxa_u32 BB_MAX_VOXEL_SHAPE_COUNT = 64;
static const daxa_u32 BB_MAX_VOXEL_OCC_U32S = 16384;   // shared occupancy bit pool (u32s)
static const daxa_u32 BB_MAX_VOXEL_SURF_COUNT = 16384; // shared surface-voxel pool (packed u32)
static const daxa_u32 BB_MAX_VOXEL_SDF_F32S = 65536;   // shared NODE signed-distance pool (f32,
                                                        // (dims+1)^3 nodes per shape)
static const daxa_u32 BB_MAX_COLLISION_COUNT = BB_MAX_RIGID_BODY_COUNT * (BB_MAX_RIGID_BODY_COUNT - 1) / 2;
static const daxa_u32 BB_MAX_MANIFOLD_NODE_COUNT = BB_MAX_COLLISION_COUNT * 2;
// Graph-coloring solver: a contact gets one of BB_MAX_COLORS colors (bit per color in a u32 body mask);
// within a color no body repeats, so all that color's contacts solve in parallel. Leftovers go to an
// overflow bucket (index BB_MAX_COLORS) solved serially. 32 fits a u32 mask and a stacked box's degree.
static const daxa_u32 BB_MAX_COLORS = 32;
static const daxa_u32 BB_COLOR_OVERFLOW = BB_MAX_COLORS; // bucket index for contacts that didn't fit a color
// The per-color solver issues BB_MAX_COLORS_SOLVE dispatches (one per color). Cover ALL colors so no
// *colored* contact is ever dropped (transient high-degree bodies can briefly need >16 colors). Empty
// colors are near-free no-op indirect dispatches. Remaining residue (genuinely uncolored after the
// coloring rounds) is counted in SimConfig::graph_color_overflow as a runtime guard.
static const daxa_u32 BB_MAX_COLORS_SOLVE = BB_MAX_COLORS;
// Neighborhood sleeping: a body sleeps when it AND every contact partner stayed below both
// velocity thresholds for BB_SLEEP_STEPS consecutive sim steps (at 60 Hz, 30 steps = 0.5 s).
// Local rule instead of whole-island: measured piles keep a "simmering crust" of a few dozen
// genuinely agitated bodies (0.3-2 m/s) that would veto an island-wide minimum forever, while
// the pressed interior is perfectly quiet. Waking is automatic: a partner that speeds up vetoes
// the sleeper on the very next step (its contact still exists while it is leaving).
static const daxa_f32 BB_SLEEP_LIN_VEL2 = 0.0144f; // (0.12 m/s)^2 — generous: the soft solver leaves residual jitter at rest
static const daxa_f32 BB_SLEEP_ANG_VEL2 = 0.0225f; // (0.15 rad/s)^2
static const daxa_u32 BB_SLEEP_STEPS = 30;
static const daxa_u32 BB_SLEEP_VETO_BIT = 0x80000000u; // sleep_timer bit 31: a contact partner is not quiet
static const daxa_u32 BB_SLEEP_TIMER_MASK = 0x7FFFFFFFu;
// ============================================================================================
// AVBD SCENE_7 BAND-AID DEPENDENCY MAP (consolidated 2026-06-19)
// --------------------------------------------------------------------------------------------
// The constants below are NOT independent tunables and NOT removable cruft: each is the necessary
// adaptation to scene_7 (432 cubes raining into a deep confined pit — a deliberate torture test).
// A "clean reference AVBD" (unified alpha=0.99, no post-stab, no band-aids) handles PLACED scenes
// but BOILS scene_7's deep burials — measured: avbd-demo3d audit, the alpha=0.99 bleed injects
// ~0.15 m/s permanent velocity into deep contacts -> boil@273. Our BETA=1e5 (10x the official 1e4)
// makes that bleed WORSE, not better. So do NOT strip toward the reference; the load-bearing set:
//
//   1. SPLIT-ALPHA POST-STAB (dispatch: main sweeps alpha=1 / post-stab alpha=0). Main sweeps hide
//      C0 (delta-only) so deep pre-existing penetration never injects momentum; a separate post-
//      stab sweep (AFTER velocity reconstruction) corrects C0 positionally. This split is exactly
//      what unified alpha=0.99 collapses -> boil. ESSENTIAL.
//   2. DEEP-EXTRACT k-floor (BB_AVBD_DEEP_EXTRACT_*). Under alpha=1 a static buried contact's
//      lambda/k decay to ~0 (no delta -> no re-ramp) so it cannot dominate the 6x6 block and never
//      extracts. The post-stab floors its k. Without it the bottom stays buried (~208mm). ESSENTIAL.
//   3. SHOCK PROPAGATION (BB_AVBD_SHOCK_LAYERS). Orders the post-stab cascade by support depth so
//      lower layers settle before upper ones sample them. UNORDERED = potential-energy pump -> boil.
//   4. EXTRACTION CAP (collision_detection.slang, max(pen,-0.25)). Caps per-step depenetration;
//      uncapped post-stab corrections teleport -> avalanche the crust. ESSENTIAL.
//   5. TERMINAL SPEED CLAMP (BB_MAX_LINEAR_SPEED). Anti-punch-through; without it 28 m/s rain
//      tunnels ~470mm + trips the impulse-explosion latch (EX[s=4]). NOTE: this caps the FALL look
//      (floaty); raising it needs sub-frame substepping (falsified: cushion/cost) -- a known wall.
//   6. INELASTIC IMPACT PASS (e=0, avbd.slang IMP_J/IMP_APPLY). Removes impact rebound post-FIN.
//   7. SETTLE SPONGE (avbd.slang finalize, v<0.3 && 3+ manifolds -> *0.9). Drains residual pile
//      micro-velocity. WIDENING IT BACKFIRES (re-excitation: lighter settle -> pen~0 flicker).
//
// KNOWN IRREDUCIBLE: the resting-pile "tremble" without sleeping is SAT-axis-flap + matcher churn +
// convergence residual, all fed by micro-motion; sleeping freezes all three (industry standard).
// The solver core (LDLT, contact-C, primal H/g, dual, cone friction) is verified correct.
// See memory: beatbox-rest-tremor, beatbox-speculative-phase1-win, avbd-jitter-rootcause.
// ============================================================================================
// AVBD (Augmented Vertex Block Descent, Giles et al. SIGGRAPH 2025) — paper defaults:
// warm-start scaling lambda <- ALPHA*GAMMA*lambda, penalty k <- max(K_MIN, GAMMA*k);
// penalty growth k <- min(K_MAX, BETA*k) while a constraint stays violated.
// Scheme transcribed from the reference implementation (savant117/avbd-demo2d, solver.cpp +
// manifold.cpp) in POST-STABILIZATION mode: during the main iterations contacts solve only the
// constraint DELTA (pre-existing penetration C0 excluded -> depenetration never injects
// momentum); velocities are reconstructed BEFORE one extra stabilization sweep that corrects C0
// positionally. lambda <= 0 (force convention), persisted fully across steps; the penalty grows
// LINEARLY (k += BETA*|C|) while the contact is active and decays by GAMMA at warm-start.
// Terminal linear-speed clamp (anti-punch-through): the rain hits at ~28 m/s, tunneling ~470mm into
// the static floor in one 1/60s step and carrying KE ~ v^2 that bounces through the pile. Capping
// speed to this value drops the impact energy (12 m/s = ~18% of 28 m/s's energy) and the per-step
// displacement to <=200mm (under the 0.25m extraction cap), so cubes don't bury deep. Resting and
// settling bodies (<1 m/s) and the canonical scenes are well under the cap, so they are untouched.
static const daxa_f32 BB_MAX_LINEAR_SPEED = 12.0f;
static const daxa_f32 BB_AVBD_BETA = 100000.0f; // (iter16+2e5 convergence experiment 2026-06-16
                                                // FALSIFIED: deep contact is UNLOADED lambda~0, so
                                                // stiffer k did NOT extract it; pen stuck ~208mm,
                                                // jitter up. Reverted. Fix = LOAD deep contacts.)
static const daxa_f32 BB_AVBD_GAMMA = 0.99f;
static const daxa_f32 BB_AVBD_PENALTY_MIN = 1.0f;
static const daxa_f32 BB_AVBD_PENALTY_MAX = 1000000000.0f;
// DEEP-EXTRACTION (post-stab only): a static buried contact's lambda decays to ~0 under the
// alpha=1 main sweeps (no delta -> no re-ramp), so its k decays to ~19 and it is too weak to
// dominate the 6x6 block solve -> it is overruled by shallower contacts and never extracts
// (trace 2026-06-16: deepest contact pen 208mm, lambda~0, k~19). In the VELOCITY-FREE post-stab
// (alpha=0, runs after velocity reconstruction so it cannot inject the boil that alpha=0.99 did),
// floor k for deep contacts so they dominate and push out. Trace is the objective judge:
// global_pen should drop, maxv must NOT spike (a spike = too strong -> avalanche, lower the floor).
static const daxa_f32 BB_AVBD_DEEP_EXTRACT_THRESH = 0.08f; // contacts deeper than 80mm
static const daxa_f32 BB_AVBD_DEEP_EXTRACT_K = 8000.0f;    // k floor in post-stab for unloaded deep contacts. 2000 was too weak to push them out (lambda~0, sideways overlaps never extracted); 50000 boils the rain pile (extracts up to the 0.25m cap/frame = teleport->avalanche). 8000 is the measured window: deep200 0, no boil, canonicals untouched (floor only fires >80mm).
// TGS_SOFT (Box2D v3 / solver2d): N sub-steps per frame, each integrates positions and updates
// contact separations -> stable stacking + fast convergence. Catto's default is 4 sub-steps.
static const daxa_u32 BB_TGS_SUBSTEPS = 4u;
static const daxa_f32 BB_AVBD_MARGIN = 0.0005f;     // collision margin (avoids flickering contacts)
static const daxa_f32 BB_AVBD_STICK_THRESH = 0.01f; // max anchor drift to keep static-friction anchors
static const daxa_f32 BB_AVBD_STICK_SLOP = 0.001f;  // anchor drift deadband: below this no positional
                                                    // pull-back (keeps the post-stab sweep from micro-
                                                    // jittering resting piles, which blocks sleeping;
                                                    // measured optimum: 0 -> sleeps 870, 1mm -> 920,
                                                    // 2mm -> 820 of 1024 at rest)
static const daxa_u32 BB_AVBD_ITERATIONS = 10;      // main sweeps (+1 post-stabilization sweep)
static const daxa_f32 BB_AVBD_CONV_EPS = 5.0e-5f;   // convergence tolerance on an iteration's max
                                                    // primal step (m + rad): 0.05 mm - stops main
                                                    // sweeps only when nothing is moving anymore
static const daxa_f32 BB_AVBD_PS_SLOP = 0.005f;     // post-stab NORMAL-row slop (same family as
                                                    // BB_AVBD_STICK_SLOP below, same measured
                                                    // rationale): sub-5mm penetration is not
                                                    // positionally corrected - correcting it every
                                                    // frame is the push-out/re-settle limit cycle
                                                    // visible as rest "nervous jitter". Depth beyond
                                                    // the slop still corrects fully.
static const daxa_u32 BB_AVBD_COLOR_ROUNDS = 16;    // Jones-Plassmann body-coloring rounds
static const daxa_u32 BB_AVBD_MAX_BODY_COLORS = 32; // primal dispatches per sweep (empty = no-op)
static const daxa_u32 BB_AVBD_SHOCK_LAYERS = 12;    // shock propagation: depth buckets for the
                                                    // ORDERED post-stab cascade (and BFS relax
                                                    // pass count). Deeper bodies saturate into
                                                    // the last bucket - still a valid ordering.
                                                    // Guendelman's insight: lower layers settle
                                                    // BEFORE upper ones sample them; v1 (row
                                                    // skipping without ordering) measured as a
                                                    // potential-energy pump - order is the cure.
static const daxa_u32 BB_AVBD_POST_STAB_SWEEPS = 4; // alpha=0 positional sweeps per frame. ONE
                                                    // (the reference) extracts deep piles about a
                                                    // contact layer per frame and plateaus at
                                                    // 40-50mm standing depth under load; 4 halve
                                                    // it to ~23mm at negligible cost. MEASURED: 8
                                                    // DIVERGE (alpha=0 is full-strength positional
                                                    // Gauss-Seidel - iterating it without under-
                                                    // relaxation ratchets closed-loop piles into
                                                    // MISS storms). More sweeps need a relaxation
                                                    // factor first.
static const daxa_u32 BB_MAX_DEBUG_CONTACT_POINT_COUNT = 8192;
static const daxa_u32 BB_MAX_DEBUG_CONTACT_LINE_VERTEX_COUNT = BB_MAX_DEBUG_CONTACT_POINT_COUNT * 2;

struct ActiveRigidBody
{
  daxa_u32 rigid_body_id;
};
DAXA_DECL_BUFFER_PTR(ActiveRigidBody)

struct MortonCode
{
  daxa_u32 morton_code;
  daxa_u32 index;
#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
  __init() {
    morton_code = MAX_U32;
    index = MAX_U32;
  }

  __init(daxa_u32 mc, daxa_u32 rbi) {
    morton_code = mc;
    index = rbi;
  }
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
};
DAXA_DECL_BUFFER_PTR(MortonCode)


struct LBVHNode 
{
  Aabb aabb;
  daxa::i32 left; // left child or invalid index in case of leaf
  daxa::i32 right; // right child or invalid node in case of leaf
  // FIXME: Can we use a union here?
  daxa::u32 primitive_idx; // custom value copied from morton code or 0 in case of inner node
  daxa::u32 pad;
#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
  bool is_leaf()
  {
    return primitive_idx != MAX_U32;
  }

  daxa::u32 get_index()
  {
    return primitive_idx;
  }

  bool is_match_index(daxa::u32 idx)
  {
    return primitive_idx == idx;
  }

  bool check_index(daxa::u32 idx)
  {
    return !is_match_index(idx) && idx < primitive_idx;
  }
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
};
DAXA_DECL_BUFFER_PTR(LBVHNode)

struct LBVHConstructionInfo
{
  daxa::u32 parent; // pointer to parent node
  daxa::u32 visitation_count; // number of times the node has been visited
  // Order-preserving-mapped AABB bounds, used to merge node AABBs across workgroups
  // with portable integer atomics (InterlockedMin/Max) instead of coherent pointer
  // loads, which Slang does not support (shader-slang/slang#3870).
  daxa_u32vec3 bound_min; // avoid member names min/max (windows.h macro trap)
  daxa_u32vec3 bound_max;
};
DAXA_DECL_BUFFER_PTR(LBVHConstructionInfo)

struct BroadPhaseCollision
{
  daxa_u32 body_a_index;
  daxa_u32 body_b_index;
};
DAXA_DECL_BUFFER_PTR(BroadPhaseCollision)


struct ManifoldNode {
    daxa_u32 manifold_id;          // Id in manifold entry buffer
    daxa_i32 next;                 // Index of next node or -1 if end
};
DAXA_DECL_BUFFER_PTR(ManifoldNode)

// AVBD per-body step state: pose at step start (velocity reconstruction) + inertial target
// (free-fall pose the descent pulls toward). GPU-only buffer, accessed via raw pointers.
struct AvbdBodyState {
  daxa_f32vec3 pos_start;
  Quaternion rot_start;
  daxa_f32vec3 pos_tilde;
  Quaternion rot_tilde;
  daxa_u32 support_depth; // shock propagation: contact-graph BFS distance from static
                          // or sleeping support (0 = static/sleeping, 1 = resting on it,
                          // ...; MAX_U32 = unsupported/free-falling). Rebuilt every step.
  daxa_u32 proposed_color; // JP body-coloring: the color a round PROPOSES (committed to body_color
                           // by entry_avbd_color_commit). Keeping the proposal out of body_color
                           // makes the per-round read of a neighbor's color race-free (determinism).
};
DAXA_DECL_BUFFER_PTR(AvbdBodyState)

// Voxel collision shape (Teardown-style concave bodies): an occupancy BITMASK over a small
// grid plus the precomputed SURFACE voxel list. The body's origin is its center of mass;
// grid_origin is the world-side corner of voxel (0,0,0) in the BODY frame. Occupancy bits
// are x-major: bit index = x + y*dims.x + z*dims.x*dims.y, packed into u32s at occ_offset.
// Surface entries pack x | y<<8 | z<<16 | normal_code<<24 (normal_code 0..5 = -x,+x,-y,+y,-z,+z).
struct VoxelShape
{
  daxa_u32vec3 dims;
  daxa_f32 voxel_size;
  daxa_f32vec3 grid_origin;
  daxa_u32 occ_offset;  // first u32 of this shape's occupancy bits in the shared pool
  daxa_u32 surf_offset; // first entry of this shape's surface list in the shared pool
  daxa_u32 surf_count;
  daxa_u32 sdf_offset;  // first f32 of this shape's NODE signed-distance grid in the shared
                        // pool: (dims+1) nodes per axis at the voxel CORNERS, exact Euclidean
                        // distance to the solid surface, negative inside. Node-based (not
                        // cell-centered) so a 2-voxel-thick feature keeps its midplane "tent"
                        // (0, -vs, 0) and the trilinear gradient never flattens to zero across
                        // thin features - the failure mode of every cell-local depth heuristic.
};
DAXA_DECL_BUFFER_PTR(VoxelShape)

struct BodyLink
{
  daxa_u32 active_index; // atomic compare exchange
  daxa_u32 island_index;
};
DAXA_DECL_BUFFER_PTR(BodyLink)

struct BodyLinkIsland 
{
  daxa_u32 active_index;
};
DAXA_DECL_BUFFER_PTR(BodyLinkIsland)

struct ManifoldLinkIsland 
{
  daxa_u32 manifold_id;
  daxa_u32 body_a_index;
  daxa_u32 body_b_index;
#if DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
  daxa_u32 min_index()
  {
    return min(body_a_index, body_b_index);
  }
#endif // DAXA_SHADERLANG == DAXA_SHADERLANG_SLANG
};
DAXA_DECL_BUFFER_PTR(ManifoldLinkIsland)

// TODO: Island config struct by AABB tree (active bodies, island count, ...)

struct Island
{
  daxa_u32 start_index;
  daxa_u32 max_count; // atomic add
  daxa_u32 count;
  daxa_u32 contact_island_index;
  daxa_u32 max_manifold_count; // atomic add
};
DAXA_DECL_BUFFER_PTR(Island)

struct ContactIsland 
{
  daxa_u32 key;
  daxa_u32 body_island_index;
  daxa_u32 start_index;
  daxa_u32 max_count;
  daxa_u32 count; // atomic add
};
DAXA_DECL_BUFFER_PTR(ContactIsland)



DAXA_DECL_TASK_HEAD_BEGIN(RigidBodyDispatcherTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_DECL_TASK_HEAD_END

struct RigidBodyDispatcherPushConstants
{
  DAXA_TH_BLOB(RigidBodyDispatcherTaskHead, task_head)
};


// MORTON CODES
DAXA_DECL_TASK_HEAD_BEGIN(RigidBodyGenerateMortonCodeTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(MortonCode), morton_codes)
DAXA_DECL_TASK_HEAD_END

struct RigidBodyGenerateMortonCodePushConstants
{
  DAXA_TH_BLOB(RigidBodyGenerateMortonCodeTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(RigidBodyRadixSortHistogramTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(MortonCode), morton_codes)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(daxa_u32), global_histograms)
DAXA_DECL_TASK_HEAD_END

struct RigidBodyRadixSortHistogramPushConstants
{
  DAXA_TH_BLOB(RigidBodyRadixSortHistogramTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(RigidBodySingleRadixSortTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(MortonCode), morton_codes_in)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(MortonCode), morton_codes_out)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(daxa_u32), global_histograms)
DAXA_DECL_TASK_HEAD_END

struct RigidBodySingleRadixSortPushConstants
{
  DAXA_TH_BLOB(RigidBodySingleRadixSortTaskHead, task_head)
};

// GENERATE HIERARCHY LBVH
DAXA_DECL_TASK_HEAD_BEGIN(RigidBodyGenerateHierarchyLinearBVHTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(MortonCode), morton_codes)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(LBVHNode), lbvh_nodes)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(LBVHConstructionInfo), lbvh_construction_info)
DAXA_DECL_TASK_HEAD_END

struct RigidBodyGenerateHierarchyLinearBVHPushConstants
{
  DAXA_TH_BLOB(RigidBodyGenerateHierarchyLinearBVHTaskHead, task_head)
};

// ===================== GRAPH COLORING (parallel contact-solver coloring) =====================
// One consolidated head shared by every coloring pass (reset / owner-reset / assign-p1 / assign-p2 / validate).
// Raw u32 color buffers:
//   body_color_mask[body]                      bit c set = body already owns a contact of color c
//   manifold_color[manifold]                   assigned color (0..BB_MAX_COLORS-1), MAX_U32 = uncolored, BB_COLOR_OVERFLOW = overflow
//   body_color_owner[body*BB_MAX_COLORS + c]   per-round arbitration: lowest manifold id wanting (body,color); also reused as 'seen' by the validator
//   color_count[BB_MAX_COLORS + 1]             contacts per color (+ overflow bucket); also the validator's colors-used scratch
// All coloring passes dispatch over graph_color_dispatch = ceil(max(collision_count, rigid_body_count)/X) and guard each access by its own bound.
DAXA_DECL_TASK_HEAD_BEGIN(GraphColorTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(Manifold), collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(daxa_u32), body_color_mask)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(daxa_u32), manifold_color)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(daxa_u32), body_color_owner)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(daxa_u32), color_count)
DAXA_DECL_TASK_HEAD_END

struct GraphColorPushConstants
{
  DAXA_TH_BLOB(GraphColorTaskHead, task_head)
};

// Per-color solver: each dispatch (one per color) runs over all manifolds and processes only the
// ones whose manifold_color == push-constant color. Within a color no body repeats => no race.
DAXA_DECL_TASK_HEAD_BEGIN(GraphColorSolveTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Manifold), collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(daxa_u32), manifold_color)
DAXA_DECL_TASK_HEAD_END

struct GraphColorSolvePushConstants
{
  DAXA_TH_BLOB(GraphColorSolveTaskHead, task_head)
  daxa_u32 color; // which color this dispatch solves
  daxa_i32 tgs_phase; // 0 = normal (PGS-family) instance; 1 = TGS_SOFT sub-step-loop instance
};

// BUILD BOUNDING BOXES LBVH
DAXA_DECL_TASK_HEAD_BEGIN(RigidBodyBuildBoundingBoxesLinearBVHTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(LBVHNode), lbvh_nodes)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(LBVHConstructionInfo), lbvh_construction_info)
DAXA_DECL_TASK_HEAD_END

struct RigidBodyBuildBoundingBoxesLBVHPushConstants
{
  DAXA_TH_BLOB(RigidBodyBuildBoundingBoxesLinearBVHTaskHead, task_head)
};

// CONVERT BOUNDING BOXES LBVH: integer-mapped bounds in lbvh_construction_info ->
// float AABBs in lbvh_nodes. Separate pass because the merge accumulates bounds with
// integer atomics (portable, no coherent pointer loads needed — slang#3870).
DAXA_DECL_TASK_HEAD_BEGIN(RigidBodyConvertBoundingBoxesLinearBVHTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(LBVHNode), lbvh_nodes)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(LBVHConstructionInfo), lbvh_construction_info)
DAXA_DECL_TASK_HEAD_END

struct RigidBodyConvertBoundingBoxesLBVHPushConstants
{
  DAXA_TH_BLOB(RigidBodyConvertBoundingBoxesLinearBVHTaskHead, task_head)
};

struct RigidBodyEntry
{
  daxa_u32 index;
};
DAXA_DECL_BUFFER_PTR(RigidBodyEntry)

// Mouse pick-and-drag (review v3 feature): a tiny host-visible bridge buffer.
// The HOST writes the input half every render frame (camera ray + button flags);
// the GPU pick/spring pass writes the state half (picked body + grab anchor).
// The two halves have disjoint writers, so no copy/sync beyond submission order is needed.
#define BB_PICK_REQUEST 1u  // pick button pressed this frame -> ray-cast and grab
#define BB_PICK_DRAGGING 2u // pick button held -> keep applying the drag spring
struct PickState
{
  // host-written input (every render frame)
  daxa_f32vec3 ray_origin;
  daxa_u32 flags; // BB_PICK_* bits
  daxa_f32vec3 ray_dir;
  daxa_f32 _pad0;
  // GPU-written state (entry_pick_spring)
  daxa_f32vec3 anchor_local; // grabbed point in the body's LOCAL frame (stays glued while dragging)
  daxa_u32 picked_id;        // PERSISTENT body id (RigidBody.id), MAX_U32 = none. NOT a row index:
                             // the sim re-sorts the rigid_bodies rows every step (morton reorder),
                             // so a row index would silently point at a DIFFERENT body next step —
                             // resolve id -> current row through rigid_body_map each step instead.
  daxa_f32 grab_t;           // distance along the ray at grab time (the drag target rides the ray)
  daxa_u32 grab_count;       // diagnostic: how many grab raycasts the GPU has executed (BB_PICK_TRACE)
  daxa_f32 _pad2;
  daxa_f32 _pad3;
};
DAXA_DECL_BUFFER_PTR(PickState)

DAXA_DECL_TASK_HEAD_BEGIN(PickSpringTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(RigidBodyEntry), rigid_body_map)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(PickState), pick_state)
DAXA_DECL_TASK_HEAD_END

struct PickSpringPushConstants
{
  DAXA_TH_BLOB(PickSpringTaskHead, task_head)
};


// REORDER RIGID BODIES
DAXA_DECL_TASK_HEAD_BEGIN(RigidBodyReorderingTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBodyEntry), rigid_body_map)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(MortonCode), morton_codes)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(LBVHNode), lbvh_nodes)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_body_sorted)
DAXA_DECL_TASK_HEAD_END

struct RigidBodyReorderingPushConstants
{
  DAXA_TH_BLOB(RigidBodyReorderingTaskHead, task_head)
};


// RESET BODY LINK
DAXA_DECL_TASK_HEAD_BEGIN(ResetBodyLinkTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(RigidBodyEntry), rigid_body_map)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(RigidBodyEntry), rigid_body_map_prev)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(ActiveRigidBody), active_rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(BodyLink), scratch_body_links)
DAXA_DECL_TASK_HEAD_END

struct ResetBodyLinkPushConstants
{
  DAXA_TH_BLOB(ResetBodyLinkTaskHead, task_head)
};


// BROAD PHASE
DAXA_DECL_TASK_HEAD_BEGIN(BroadPhaseTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(ActiveRigidBody), active_rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(LBVHNode), lbvh_nodes)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(BroadPhaseCollision), broad_phase_collisions)
DAXA_DECL_TASK_HEAD_END

struct BroadPhasePushConstants
{
  DAXA_TH_BLOB(BroadPhaseTaskHead, task_head)
};

// NARROW PHASE DISPATCHER
DAXA_DECL_TASK_HEAD_BEGIN(NarrowPhaseDispatcherTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_DECL_TASK_HEAD_END

struct NarrowPhaseDispatcherPushConstants
{
  DAXA_TH_BLOB(NarrowPhaseDispatcherTaskHead, task_head)
};

struct CollisionEntry
{
  daxa_u32 index;
};
DAXA_DECL_BUFFER_PTR(CollisionEntry)

// ===================== AVBD (Augmented Vertex Block Descent) =====================
// One consolidated head shared by every AVBD pass (body coloring rounds / validator / prepare /
// per-color primal block solves / dual updates / finalize). Buffers:
//   avbd_state[body]  pose at step start + inertial target (AvbdBodyState)
//   body_color[body]  Jones-Plassmann vertex color (MAX_U32 = uncolored; statics never colored)
// Adjacency = the narrow phase's per-body manifold linked lists (manifold_nodes + RigidBody::
// manifold_node_index); manifold ids are SCRATCH ids, translated to the packed solver buffer via
// collision_map (same indirection the per-island solver used), so persisted lambda/k flow through
// the regular warm-start chain.
DAXA_DECL_TASK_HEAD_BEGIN(AvbdTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Manifold), collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(ManifoldNode), manifold_nodes)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(CollisionEntry), collision_map)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(AvbdBodyState), avbd_state)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(daxa_u32), body_color)
DAXA_DECL_TASK_HEAD_END

struct AvbdPushConstants
{
  DAXA_TH_BLOB(AvbdTaskHead, task_head)
  daxa_u32 color;       // primal: which body color to solve; coloring rounds: the round number
  daxa_f32 stab_alpha;  // 1.0 = main sweeps (constraint delta only), 0.0 = post-stabilization
  daxa_u32 ps_depth;    // shock propagation (post-stab only): solve ONLY bodies whose
                        // saturated support depth equals this layer; MAX_U32 = no filter
};


// NARROW PHASE
DAXA_DECL_TASK_HEAD_BEGIN(NarrowPhaseTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), previous_sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(BroadPhaseCollision), broad_phase_collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_RWBufferPtr(RigidBodyEntry), rigid_body_map)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(ManifoldNode), rigid_body_link_manifolds)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(CollisionEntry), collision_map)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Manifold), collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_RWBufferPtr(RigidBodyEntry), rigid_body_map_prev)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), previous_rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(ManifoldNode), previous_rigid_body_link_manifolds)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(CollisionEntry), collision_map_prev)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(Manifold), old_collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(BodyLink), scratch_body_links)
DAXA_DECL_TASK_HEAD_END

struct NarrowPhasePushConstants
{
  DAXA_TH_BLOB(NarrowPhaseTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(CollisionSolverDispatcherTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_DECL_TASK_HEAD_END

struct CollisionSolverDispatcherPushConstants
{
  DAXA_TH_BLOB(CollisionSolverDispatcherTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(RigidBodySimTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_DECL_TASK_HEAD_END


struct RigidBodySimPushConstants
{
  DAXA_TH_BLOB(RigidBodySimTaskHead, task_head)
  daxa_i32 tgs_phase; // 0 = normal advect (PGS family, once at full dt); 1 = TGS sub-step gravity
};


// NEIGHBORHOOD SLEEPING (reduce: per-body quiet timer; veto: contacts with a non-quiet partner
// set the veto bit on the body; apply: quiet + un-vetoed bodies fall asleep, vetoed/loud wake)
DAXA_DECL_TASK_HEAD_BEGIN(SleepTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(Manifold), collisions)
DAXA_DECL_TASK_HEAD_END

struct SleepPushConstants
{
  DAXA_TH_BLOB(SleepTaskHead, task_head)
};

// ISLANDS
DAXA_DECL_TASK_HEAD_BEGIN(IslandCounterTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(BodyLink), scratch_body_links)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Island), islands)
DAXA_DECL_TASK_HEAD_END

struct IslandCounterPushConstants
{
  DAXA_TH_BLOB(IslandCounterTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(IslandDispatcherTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_DECL_TASK_HEAD_END

struct IslandDispatcherPushConstants
{
  DAXA_TH_BLOB(IslandDispatcherTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(IslandBuilderTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(BodyLink), scratch_body_links)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Island), islands)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(ActiveRigidBody), active_rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(RigidBodyEntry), rigid_body_map)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_DECL_TASK_HEAD_END

struct IslandBuilderPushConstants
{
  DAXA_TH_BLOB(IslandBuilderTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(IslandPrefixSumTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Island), islands)
DAXA_DECL_TASK_HEAD_END

struct IslandPrefixSumPushConstants
{
  DAXA_TH_BLOB(IslandPrefixSumTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(IslandBuilderBodyLink2IslandTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(BodyLink), scratch_body_links)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Island), islands)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(BodyLinkIsland), body_links)
DAXA_DECL_TASK_HEAD_END

struct IslandBuilderBodyLink2IslandPushConstants
{
  DAXA_TH_BLOB(IslandBuilderBodyLink2IslandTaskHead, task_head)
};

// simple bubble sort for sorting body links in island for now
DAXA_DECL_TASK_HEAD_BEGIN(IslandBuilderSortBodyLinkInIslandTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Island), islands)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(BodyLinkIsland), body_links)
DAXA_DECL_TASK_HEAD_END

struct IslandBuilderSortBodyLinkInIslandPushConstants
{
  DAXA_TH_BLOB(IslandBuilderSortBodyLinkInIslandTaskHead, task_head)
};

// MANIFOLDS
DAXA_DECL_TASK_HEAD_BEGIN(ManifoldIslandBuilderTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(BodyLink), scratch_body_links)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(Manifold), collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Island), islands)
DAXA_DECL_TASK_HEAD_END

struct ManifoldIslandBuilderPushConstants
{
  DAXA_TH_BLOB(ManifoldIslandBuilderTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(ContactIslandGatherTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(Island), islands)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(ContactIsland), contact_islands)
DAXA_DECL_TASK_HEAD_END

struct ContactIslandGatherPushConstants
{
  DAXA_TH_BLOB(ContactIslandGatherTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(ContactIslandDispatcherTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_DECL_TASK_HEAD_END

struct ContactIslandDispatcherPushConstants
{
  DAXA_TH_BLOB(ContactIslandDispatcherTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(ManifoldIslandPrefixSumTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(ContactIsland), contact_islands)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(Island), islands)
DAXA_DECL_TASK_HEAD_END

struct ManifoldIslandPrefixSumPushConstants
{
  DAXA_TH_BLOB(ManifoldIslandPrefixSumTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(IslandBuilderManifoldLink2IslandTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(BodyLink), scratch_body_links)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(CollisionEntry), collision_map)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(Manifold), collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(Island), islands)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(ContactIsland), contact_islands)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(ManifoldLinkIsland), manifold_links)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Manifold), collision_sorted)
DAXA_DECL_TASK_HEAD_END

struct IslandBuilderManifoldLink2IslandPushConstants
{
  DAXA_TH_BLOB(IslandBuilderManifoldLink2IslandTaskHead, task_head)
};

// simple bubble sort for sorting body links in island for now
DAXA_DECL_TASK_HEAD_BEGIN(IslandBuilderSortManifoldLinkInIslandTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(ContactIsland), contact_islands)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(ManifoldLinkIsland), manifold_links)
DAXA_DECL_TASK_HEAD_END

struct IslandBuilderSortManifoldLinkInIslandPushConstants
{
  DAXA_TH_BLOB(IslandBuilderSortManifoldLinkInIslandTaskHead, task_head)
};

// SOLVER
DAXA_DECL_TASK_HEAD_BEGIN(CollisionPreSolverTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(CollisionEntry), collision_map)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Manifold), collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(ContactIsland), contact_islands)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(ManifoldLinkIsland), manifold_links)
DAXA_DECL_TASK_HEAD_END

struct CollisionPreSolverPushConstants
{
  DAXA_TH_BLOB(CollisionPreSolverTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(CollisionSolverTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(CollisionEntry), collision_map)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Manifold), collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(ContactIsland), contact_islands)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(ManifoldLinkIsland), manifold_links)
DAXA_DECL_TASK_HEAD_END

struct CollisionSolverPushConstants
{
  DAXA_TH_BLOB(CollisionSolverTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(IntegratePositionsTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_DECL_TASK_HEAD_END

struct RigidBodyIntegratePositionsPushConstants
{
  DAXA_TH_BLOB(IntegratePositionsTaskHead, task_head)
  daxa_i32 tgs_phase; // 0 = normal integrate (once at full dt); 1 = TGS sub-step integrate
};

DAXA_DECL_TASK_HEAD_BEGIN(CollisionSolverRelaxationTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(CollisionEntry), collision_map)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(Manifold), collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(ContactIsland), contact_islands)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(ManifoldLinkIsland), manifold_links)
DAXA_DECL_TASK_HEAD_END

struct CollisionSolverRelaxationPushConstants
{
  DAXA_TH_BLOB(CollisionSolverRelaxationTaskHead, task_head)
};

DAXA_DECL_TASK_HEAD_BEGIN(RigidBodyUpdateTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
// rigid_bodies is READ-ONLY here (the shader only reads it; it writes the separate
// rigid_bodies_update buffer). Declaring it READ_WRITE made the task graph think this
// pass overwrites rigid_bodies -> false WAW with the solver -> solver writes culled.
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(RigidBody), rigid_bodies)
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
DAXA_TH_BUFFER_PTR(INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(daxa_BlasInstanceData), blas_instance_data)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_RWBufferPtr(RigidBodyEntry), rigid_body_map)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(Aabb), aabbs)
DAXA_DECL_TASK_HEAD_END

struct UpdateInstancesPushConstants
{
  DAXA_TH_BLOB(UpdateInstancesTaskHead, task_head)
};


DAXA_DECL_TASK_HEAD_BEGIN(CreatePointsTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_INDIRECT_COMMAND_READ, daxa_BufferPtr(DispatchBuffer), dispatch_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(Manifold), collisions)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(daxa_u32), manifold_color)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(daxa_u32), body_color) // AVBD body coloring (graph-debug tint)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(GUIVertex), vertex_buffer)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(GUIVertexLine), line_vertex_buffer)
DAXA_DECL_TASK_HEAD_END

struct CreatePointsPushConstants
{
  DAXA_TH_BLOB(CreatePointsTaskHead, task_head)
};

// Define maximum number of bounces
static const daxa_u32 MAX_BOUNCES = 3;

// Update the HitPayload structure
struct HitPayload {
    daxa_f32vec3 position;    // Hit position
    daxa_f32vec3 normal;      // Surface normal at hit point
    // daxa_f32vec3 albedo;      // Surface albedo (color)
    // daxa_f32vec3 emission;    // Surface emission (light)
    daxa_u32 instance_index;  // Instance index
    daxa_u32 primitive_index; // Primitive index
    // daxa_b32 hit;             // Flag to indicate a hit
    // daxa_u32 seed;            // Random seed for NEE
    daxa_f32vec3 throughput; // Accumulated throughput
    daxa_f32vec3 radiance;   // Accumulated radiance
};

struct ShadowPayload
{
  daxa_b32 hit;             // Flag to indicate a hit
};

struct MyAttributes
{
  daxa_u32 instance_index;
  daxa_u32 primitive_index;
  daxa_f32 factor;
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
