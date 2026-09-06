#pragma once
#include "shared.inl"
struct FragmentLabelRemap { daxa_u32 source, target; };
struct FragmentPackingPushConstants
{
  daxa_u64 labels_addr, remap_addr, occupancy_addr;
  daxa_u32vec3 parent_dims, crop_min, crop_dims;
  daxa_u32 remap_count, target_label, occ_offset;
};

#if defined(__cplusplus)
static_assert(sizeof(FragmentLabelRemap) == 8);
static_assert(sizeof(FragmentPackingPushConstants) == 72);
#endif
