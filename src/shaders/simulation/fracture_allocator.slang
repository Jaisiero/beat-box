#include "fracture_allocator_common.slang"
[[vk::push_constant]] FractureAllocatorPushConstants P;

[shader("compute")]
[numthreads(1,1,1)]
void entry_fracture_allocate(uint3 tid : SV_DispatchThreadID)
{
  FractureAllocationState* state=(FractureAllocationState*)P.state_addr;
  VoxelShape* shapes=(VoxelShape*)P.shapes_addr;
  FractureBatchManifest* batch=(FractureBatchManifest*)P.batch_addr;
  if (P.operation==2u) { batch->status=0u;batch->count=0u;batch->refused=0u;return; }
  FragmentPlanManifest* plan=(FragmentPlanManifest*)P.plan_addr;
  FractureAllocationManifest* out=(FractureAllocationManifest*)P.allocation_addr;
  FractureParentContext* context=&((FractureParentContext*)P.contexts_addr)[P.parent_id];
  if (P.operation==1u)
  {
    if (state->active==0u) {
      if (P.batch_addr!=0u && out->status!=0u) {
        ++batch->refused;
        if (out->status>=3u) batch->status=out->status;
      }
      return;
    }
    // Packing has consumed every source cell. No pending reader can still use
    // its private voxel slices; pinned scene templates remain resident.
    if (!release_shape(state,context->shape,context->parent.shape_index-1u,true)) ++state->failures;
    for (uint i=0u;i<GPU_POOL_COUNT;++i) copy_pool(&state->pools[i],&state->reserved[i]);
    state->active=0u;
    if (P.batch_addr!=0u) {
      for (uint child=0u;child<out->count;++child) {
        uint index=batch->count++;
        batch->children[index].parent_id=P.parent_id;
        batch->children[index].component=plan->components[child];
        batch->children[index].allocation=out->children[child];
      }
    }
    return;
  }
  out->status=plan->status;out->count=0u;
  out->dispatch_x=0u;out->dispatch_y=0u;out->dispatch_z=1u;
  if (plan->status!=0u || plan->count<=1u) return;
  if (P.batch_addr!=0u && plan->count>FRAGMENT_PLAN_CAPACITY-batch->count) { out->status=2u;return; }
  if (plan->solid_count!=context->parent.primitive_count) { out->status=4u;return; }
  if (state->active!=0u || state->failures!=0u) { out->status=3u;return; }
  for (uint i=0u;i<GPU_POOL_COUNT;++i) copy_pool(&state->reserved[i],&state->pools[i]);
  uint max_words=0u;
  for (uint child=0u;child<plan->count;++child)
  {
    FragmentComponent c=plan->components[child];
    uint3 dims=uint3(c.hi_x-c.lo_x+1u,c.hi_y-c.lo_y+1u,c.hi_z-c.lo_z+1u);
    uint cells=dims.x*dims.y*dims.z;
    uint units[GPU_POOL_COUNT];
    units[0]=(cells+31u)/32u;units[1]=(dims.x+1u)*(dims.y+1u)*(dims.z+1u);
    units[2]=cells;units[3]=0u;units[4]=1u;units[5]=child==0u ? 0u : 1u;
    for (uint pool=0u;pool<GPU_POOL_COUNT;++pool)
    {
      uint offset=state->reserved[pool].allocate(units[pool]);
      out->children[child].offsets[pool]=offset;
      if (offset==GPU_POOL_INVALID) { out->status=2u;return; }
    }
    if (child==0u) out->children[child].offsets[5]=P.parent_id;
    max_words=max(max_words,units[0]);
  }
  // Only after every reservation succeeded may descriptors become visible.
  for (uint child=0u;child<plan->count;++child)
  {
    FragmentComponent c=plan->components[child];
    FractureChildAllocation allocation=out->children[child];
    VoxelShape shape=context->shape;
    shape.dims=uint3(c.hi_x-c.lo_x+1u,c.hi_y-c.lo_y+1u,c.hi_z-c.lo_z+1u);
    shape.occ_offset=allocation.offsets[0];shape.sdf_offset=allocation.offsets[1];shape.surf_offset=allocation.offsets[2];
    shape.surf_count=0u;
    shapes[allocation.offsets[4]]=shape;
    state->private_shapes[allocation.offsets[4]]=1u;
    FractureBodyEdit* edit=&state->body_edits[allocation.offsets[5]];
    edit->kind=1u;edit->parent_id=P.parent_id;edit->shape_id=allocation.offsets[4]+1u;
    edit->primitive_count=c.count;edit->crop_off=float3(c.lo_x,c.lo_y,c.lo_z)*shape.voxel_size;
  }
  state->active=1u;out->count=plan->count;
  out->dispatch_x=(max_words+63u)/64u;out->dispatch_y=plan->count;
}
