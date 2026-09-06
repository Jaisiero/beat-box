#pragma once
#include "shared.inl"

static const daxa_u32 BB_FRAGMENT_CENSUS_CAPACITY = 128;
struct FragmentComponent
{
  daxa_u32 label, count;
  daxa_u32 sum_x, sum_y, sum_z;
  daxa_u32 lo_x, lo_y, lo_z, hi_x, hi_y, hi_z;
};
struct FragmentCensusOutput
{
  daxa_u32 count;
  FragmentComponent components[BB_FRAGMENT_CENSUS_CAPACITY];
};
struct FragmentCensusPushConstants
{
  daxa_u64 labels_addr, scratch_addr, output_addr;
  daxa_u32vec3 dims;
  daxa_u64 context_addr; // zero for standalone verification inputs
};
#if defined(__cplusplus)
static_assert(sizeof(FragmentComponent) == 44);
// Coordinate sums are bounded by sum(cell_index), N*(N-1)/2. Increasing
// the shape capacity beyond this requires wider integer accumulators.
static_assert(BB_MAX_VOXEL_SDF_F32S <= 65536);
#endif
