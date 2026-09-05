#include "rigid_body_math.slang"
#include "LBVH.slang"

[[vk::push_constant]] BroadPhasePushConstants BP;

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_broad_phase(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  var sim_config =
      (SimConfig*)(BP.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  if (index >= sim_config->rigid_body_count)
  {
    return;
  }

  // FIXME: split between static and dynamic objects
  // Pointer access (NOT value-copy): a whole-struct load through a daxa buffer
  // pointer pads vec3->16 in Daxa 3.6, corrupting the OBB and thus the query AABB.
  RigidBody* rigid_bodies = (RigidBody*)(BP.task_head.rigid_bodies);
  RigidBody* rigid_body = &rigid_bodies[index];
  let lbvh_nodes = (LBVHNode*)(BP.task_head.lbvh_nodes);
  var bp_collisions =
      (BroadPhaseCollision*)(BP.task_head.broad_phase_collisions);

  Aabb aabb = rigid_body->get_AABB_from_OBB();

  LBVH::broad_phase(sim_config, index, aabb, lbvh_nodes, bp_collisions);
}
