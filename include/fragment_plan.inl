#pragma once
#include "fragment_census.inl"

static const daxa_u32 FRAGMENT_PLAN_CAPACITY = BB_MAX_VOXEL_SHAPE_COUNT;
struct FragmentPlanManifest
{
  // status: 0 ready; 1 exceeds the maximum number of shape slots.
  daxa_u32 status, count, source_count, solid_count;
  FragmentComponent components[FRAGMENT_PLAN_CAPACITY];
};
struct FragmentPlanPushConstants
{
  daxa_u64 census_addr, remap_addr, manifest_addr;
  daxa_u32 cells;
  daxa_u64 context_addr; // zero for standalone verification inputs
};
