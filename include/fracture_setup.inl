#pragma once
#include "shared.inl"

struct FractureParentContext
{
  RigidBody parent;
  VoxelShape shape;
  daxa_f32vec3 center;
  daxa_f32 radius;
  daxa_u32 site_count;
  daxa_f32vec4 sites[BB_MAX_FRACTURE_SITES];
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
