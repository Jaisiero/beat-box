#include "fracture_allocator.inl"
[[vk::push_constant]] FractureBatchPackingPushConstants P;
[shader("compute")]
[numthreads(64,1,1)]
void entry_fragment_batch_pack(uint3 tid : SV_DispatchThreadID)
{
  FractureAllocationManifest* allocations=(FractureAllocationManifest*)P.allocation_addr;
  if (tid.y>=allocations->count) return;
  FragmentComponent c=((FragmentPlanManifest*)P.plan_addr)->components[tid.y];
  uint3 dims=uint3(c.hi_x-c.lo_x+1u,c.hi_y-c.lo_y+1u,c.hi_z-c.lo_z+1u);
  uint cells=dims.x*dims.y*dims.z,word=tid.x;
  if (word>=(cells+31u)/32u) return;
  uint3 parent=((FractureParentContext*)P.contexts_addr)[P.parent_id].shape.dims;
  uint parent_cells=parent.x*parent.y*parent.z;
  uint bits=0u;
  for (uint bit=0u;bit<32u && word*32u+bit<cells;++bit)
  {
    uint cell=word*32u+bit;
    uint3 position=uint3(cell%dims.x,(cell/dims.x)%dims.y,cell/(dims.x*dims.y))+uint3(c.lo_x,c.lo_y,c.lo_z);
    uint label=((uint*)P.labels_addr)[position.x+position.y*parent.x+position.z*parent.x*parent.y];
    if (label<parent_cells && ((uint*)P.remap_addr)[label]==c.label) bits|=1u<<bit;
  }
  uint offset=allocations->children[tid.y].offsets[0];
  ((uint*)P.occupancy_addr)[offset+word]=bits;
}
