#pragma once
#include "shared.inl"

struct VoxelPrimitiveBatchPushConstants
{
  daxa_u64 bodies_addr, shapes_addr, occupancy_addr, primitives_addr;
  daxa_u32 body_count;
};
