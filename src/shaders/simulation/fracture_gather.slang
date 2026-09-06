#include "fracture_setup.inl"
[[vk::push_constant]] FractureGatherPushConstants P;

[shader("compute")]
[numthreads(64,1,1)]
void entry_fracture_gather(uint3 tid : SV_DispatchThreadID)
{
  if (tid.x >= P.source_count) return;
  RigidBody live = ((RigidBody*)P.source_addr)[tid.x];
  if (live.id >= P.target_count) return;
  RigidBody* target = &((RigidBody*)P.target_addr)[live.id];
  *target = live;
  target->island_index = MAX_U32;
  target->manifold_node_index = MAX_U32;
  target->active_index = MAX_U32;
  ((daxa_BlasInstanceData*)P.instances_addr)[live.id].transform = target->get_instance_transform();
}
