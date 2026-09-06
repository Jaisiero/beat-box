#pragma once
#include "shared.inl"

static const daxa_u32 BB_FRACTURE_FLOOD_PASSES = 40u;
#if defined(__cplusplus)
static_assert(BB_MAX_VOXEL_SDF_F32S <= 65536u);
#endif

struct FractureParentContext
{
  RigidBody parent;
  VoxelShape shape;
  daxa_f32vec3 center;
  daxa_f32 radius;
  daxa_u32 site_count;
  daxa_f32vec4 sites[BB_MAX_FRACTURE_SITES];
  daxa_u32vec3 cell_dispatch;
};
struct FractureSetupPushConstants
{
  daxa_u64 bodies_addr, shapes_addr, contexts_addr, events_addr;
  daxa_u32 body_id, body_count, event_slot;
};
struct FractureGatherPushConstants
{
  daxa_u64 source_addr, target_addr, instances_addr;
  daxa_u32 source_count, target_count;
};

#if defined(__cplusplus)
// CPU command-recording bound from the AS metadata already mirrored on host.
// GPU setup independently generates the actual dispatch arguments.
inline daxa_u32 fracture_recorded_passes(VoxelShape const &shape)
{
  daxa_u32 cells=shape.dims.x*shape.dims.y*shape.dims.z;
  if (cells<=64u) return 1u;
  daxa_u32 passes=8u;
  for (;cells>1u;cells>>=1u) passes+=2u;
  return passes;
}
#endif
