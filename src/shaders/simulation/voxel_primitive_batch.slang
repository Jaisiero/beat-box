#include "voxel_primitive_batch.inl"
[[vk::push_constant]] VoxelPrimitiveBatchPushConstants P;

// One group per body, one lane per cell in a 64-cell tile. A running solid
// count replaces the per-cell scan of every preceding occupancy word. Each
// lane derives the same tile prefix, so no atomics or group barriers are needed.
// The output remains ascending occupancy-bit order, exactly as the old builder.
[shader("compute")]
[numthreads(64,1,1)]
void entry_voxel_primitive_batch(uint3 group : SV_GroupID, uint lane : SV_GroupIndex)
{
  uint id=group.x;
  if (id>=P.body_count) return;
  RigidBody* body=&((RigidBody*)P.bodies_addr)[id];
  if (body->shape_index==0u) return;
  VoxelShape shape=((VoxelShape*)P.shapes_addr)[body->shape_index-1u];
  uint cells=shape.dims.x*shape.dims.y*shape.dims.z;
  uint words=(cells+31u)/32u;
  uint* occ=(uint*)P.occupancy_addr;
  Aabb* primitives=(Aabb*)P.primitives_addr;
  uint prefix=0u;
  for (uint tile=0u;tile<cells;tile+=64u)
  {
    uint wi=tile/32u;
    uint lo=occ[shape.occ_offset+wi];
    uint hi=wi+1u<words ? occ[shape.occ_offset+wi+1u] : 0u;
    uint word=lane<32u ? lo : hi;
    uint bit=lane%32u;
    uint cell=tile+lane;
    if (cell<cells && ((word >> bit)&1u)!=0u)
    {
      uint rank=prefix+(lane<32u ? 0u : countbits(lo));
      rank+=countbits(word & ((1u << bit)-1u));
      uint x=cell%shape.dims.x;
      uint y=(cell/shape.dims.x)%shape.dims.y;
      uint z=cell/(shape.dims.x*shape.dims.y);
      float3 center=shape.grid_origin+float3(float(x)+0.5f,float(y)+0.5f,float(z)+0.5f)*shape.voxel_size;
      primitives[body->primitive_offset+rank].minimum=center-float3(0.5f*shape.voxel_size);
      primitives[body->primitive_offset+rank].maximum=center+float3(0.5f*shape.voxel_size);
    }
    prefix+=countbits(lo)+countbits(hi);
  }
}
