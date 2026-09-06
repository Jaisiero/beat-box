#include "fracture_allocator_common.slang"
[[vk::push_constant]] FractureSceneEditPushConstants P;

uint spawn_random(inout uint state)
{
  state=state*747796405u+2891336453u;
  uint word=((state >> ((state >> 28u)+4u)) ^ state)*277803737u;
  return (word >> 22u)^word;
}
float spawn_unit(inout uint state) { return float(spawn_random(state) >> 8u)*(1.0f/16777216.0f); }

[shader("compute")]
[numthreads(1,1,1)]
void entry_fracture_scene_edit(uint3 tid : SV_DispatchThreadID)
{
  FractureAllocationState* state=(FractureAllocationState*)P.allocator_addr;
  FractureSceneEditManifest* out=(FractureSceneEditManifest*)P.output_addr;
  out->status=0u;out->retired_count=0u;out->spawn_template=0u;out->spawn_id=MAX_U32;
  if (state->active!=0u || state->failures!=0u) { out->status=3u;return; }
  if ((P.operation & FRACTURE_EDIT_RETIRE)!=0u)
  {
    RigidBody* bodies=(RigidBody*)P.bodies_addr;
    VoxelShape* shapes=(VoxelShape*)P.shapes_addr;
    for (uint i=0u;i<P.body_count;++i)
    {
      RigidBody live=bodies[i];
      if (!live.has_flag(RigidBodyFlag::DYNAMIC) || !(live.position.y<P.kill_y)) continue;
      bool valid=live.id < BB_MAX_RIGID_BODY_COUNT;
      if (live.shape_index!=0u)
        valid=release_shape(state,shapes[live.shape_index-1u],live.shape_index-1u,false) && valid;
      valid=state->pools[5].release(live.id,1u) && valid;
      if (!valid) { ++state->failures;out->status=3u;return; }
      state->body_edits[live.id].kind=2u;
      out->ids[out->retired_count++]=live.id;
    }
  }
  if ((P.operation & FRACTURE_EDIT_SPAWN)==0u || P.template_count==0u) return;
  uint slot=state->pools[5].allocate(1u);
  if (slot==GPU_POOL_INVALID) { out->status=2u;return; }
  uint rng=state->spawn_seed;
  uint source=spawn_random(rng)%P.template_count;
  uint anvil=spawn_random(rng)%4u;
  RigidBody body=((RigidBody*)P.templates_addr)[source];
  body.id=slot;
  float x=-9.0f+float(anvil)*6.0f+(spawn_unit(rng)*2.0f-1.0f)*0.4f;
  float y=11.0f+(spawn_unit(rng)*2.0f-1.0f)*0.4f;
  float z=14.0f+(spawn_unit(rng)*2.0f-1.0f)*0.4f;
  body.position=float3(x,y,z);
  float wx=spawn_unit(rng)*2.0f-1.0f;
  float wy=spawn_unit(rng)*2.0f-1.0f;
  float wz=spawn_unit(rng)*2.0f-1.0f;
  body.omega=float3(wx,wy,wz);
  body.prev_omega=body.omega;body.prev_velocity=body.velocity;
  body.island_index=MAX_U32;body.manifold_node_index=MAX_U32;body.active_index=MAX_U32;
  body.sleep_timer=0u;
  ((RigidBody*)P.spawn_addr)[0]=body;
  state->body_edits[slot].kind=3u;
  state->spawn_seed=rng;
  out->spawn_template=source;out->spawn_id=slot;
}
