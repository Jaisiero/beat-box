#pragma once
#include "fracture_allocator.inl"

void copy_pool(GpuFreeList* to, GpuFreeList* from)
{
  to->capacity=from->capacity;to->high_water=from->high_water;
  to->live_units=from->live_units;to->range_count=from->range_count;
  for (uint i=0u;i<from->range_count;++i) to->ranges[i]=from->ranges[i];
}

bool release_shape(FractureAllocationState* state, VoxelShape shape, uint slot, bool reserved)
{
  if (state->private_shapes[slot]==0u) return true;
  uint cells=shape.dims.x*shape.dims.y*shape.dims.z;
  GpuFreeList* pools=reserved ? &state->reserved[0] : &state->pools[0];
  bool valid=pools[0].release(shape.occ_offset,(cells+31u)/32u);
  valid=pools[1].release(shape.sdf_offset,(shape.dims.x+1u)*(shape.dims.y+1u)*(shape.dims.z+1u)) && valid;
  valid=pools[2].release(shape.surf_offset,cells) && valid;
  valid=pools[4].release(slot,1u) && valid;
  if (valid) state->private_shapes[slot]=0u;
  return valid;
}
