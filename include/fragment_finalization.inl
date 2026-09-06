#pragma once
#include "shared.inl"

// Separate from shared.inl: this contract is consumed only by fragment finalization.
struct FragmentFinalizePushConstants
{
  daxa_u64 bodies_addr;
  daxa_u64 instances_addr;
  daxa_u64 shapes_addr;
  daxa_u64 derived_addr;
  daxa_u32 body;
  daxa_f32 voxel_mass;
  daxa_f32vec3 com_old;
  daxa_f32vec3 parent_pos;
  Quaternion parent_rot;
  daxa_f32vec3 parent_vel;
  daxa_f32vec3 parent_omega;
  daxa_f32vec3 crop_off;
};
#if defined(__cplusplus)
static_assert(sizeof(FragmentFinalizePushConstants) == 120);
#endif
