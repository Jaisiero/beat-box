#include "collision_detection.slang"

[[vk::push_constant]] NarrowPhasePushConstants NP;

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_narrow_phase(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  var sim_config =
      (SimConfig*)(NP.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  daxa_u32 collision_count = min(sim_config->broad_phase_collision_count, BB_MAX_BROAD_PAIR_COUNT);
  if (index >= collision_count)
  {
    return;
  }
  { // narrow-phase COVERAGE probe: must equal the broad count every frame (truncation check)
    daxa_u32 _np_prev;
    InterlockedAdd(sim_config->dbg_np_processed, 1u, _np_prev);
  }

  let previous_sim_config =
      *NP.task_head.previous_sim_config;

  var rigid_body_map =
      (RigidBodyEntry*)(NP.task_head.rigid_body_map);

  var rigid_bodies =
      (RigidBody*)(NP.task_head.rigid_bodies);

  var previous_rigid_body_map =
      (RigidBodyEntry*)(NP.task_head.rigid_body_map_prev);

  var previous_rigid_bodies =
      (RigidBody*)(NP.task_head.previous_rigid_bodies);

  var rigid_body_link_manifolds =
      (ManifoldNode*)(NP.task_head.rigid_body_link_manifolds);

  var collision_map =
      (CollisionEntry*)(NP.task_head.collision_map);

  var collisions =
      (Manifold*)(NP.task_head.collisions);

  var previous_rigid_body_link_manifolds =
      (ManifoldNode*)(NP.task_head.previous_rigid_body_link_manifolds);

  var collision_map_prev =
      (CollisionEntry*)(NP.task_head.collision_map_prev);

  var old_collisions =
      (Manifold*)(NP.task_head.old_collisions);

  var body_links =
      (BodyLink*)(NP.task_head.scratch_body_links);

  var bp_collision =
      (BroadPhaseCollision)(NP.task_head.broad_phase_collisions[index]);

  narrow_phase(bp_collision, rigid_body_map, rigid_bodies, sim_config, previous_sim_config, rigid_body_link_manifolds, collision_map, collisions, previous_rigid_body_map, previous_rigid_bodies, previous_rigid_body_link_manifolds, collision_map_prev, old_collisions, body_links, index);
}
