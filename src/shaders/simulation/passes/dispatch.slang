#include "shared.inl"

[[vk::push_constant]] RigidBodyDispatcherPushConstants RBD;
[[vk::push_constant]] NarrowPhaseDispatcherPushConstants NPD;
[[vk::push_constant]] NarrowPhasePushConstants NP;
[[vk::push_constant]] CollisionSolverDispatcherPushConstants CSD;
[[vk::push_constant]] IslandDispatcherPushConstants ID;
[[vk::push_constant]] ContactIslandDispatcherPushConstants CID;

[shader("compute")]
    [numthreads(1, 1, 1)] void
    entry_rigid_body_dispatcher(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  if (compute_i.x >= 1u)
  {
    return;
  }

  SimConfig* sim_config =
      (SimConfig*)(RBD.task_head.sim_config);

  DispatchBuffer* dispatch_buffer =
      (DispatchBuffer*)(RBD.task_head.dispatch_buffer);

  let rb_count = (sim_config->rigid_body_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
  dispatch_buffer->rigid_body_dispatch = daxa_u32vec3(rb_count, 1, 1);

  let arb_count = (sim_config->active_rigid_body_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
  dispatch_buffer->active_rigid_body_dispatch = daxa_u32vec3(arb_count, 1, 1);

  let rs_rb_count = (sim_config->rigid_body_count + RADIX_SORT_WORKGROUP_SIZE - 1) / RADIX_SORT_WORKGROUP_SIZE;
  dispatch_buffer->radix_sort_rigid_body_dispatch = daxa_u32vec3(rs_rb_count, 1, 1);

  // printf("radix_sort_rigid_body_dispatch %d\n", rs_rb_count);
}

[shader("compute")]
    [numthreads(1, 1, 1)] void
    entry_narrow_phase_dispatcher(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  if (compute_i.x >= 1u)
  {
    return;
  }

  SimConfig* sim_config =
      (SimConfig*)(NPD.task_head.sim_config);

  // per-frame reset of narrow-phase-written diagnostics, here (single-thread, BEFORE the NP that
  // writes them) instead of in coloring (which runs AFTER NP and would wipe them every frame).
  sim_config->dbg_pen = 0u;
  sim_config->dbg_deep100 = 0u;
  sim_config->dbg_deep200 = 0u;
  sim_config->dbg_contact_overflow = 0u; // saturation counters: reset here (single-thread, BEFORE the NP that increments them)
  sim_config->dbg_node_overflow = 0u;
  sim_config->dbg_vox_interior = 0u; // voxel embedding latch (see shared.inl)
  sim_config->dbg_vox_wedge = 0u;    // thin-feature pass-through wedge latch (see shared.inl)
  sim_config->dbg_miss_present = 0u; // warm-start MISS anatomy histogram (see shared.inl)
  sim_config->dbg_miss_absent = 0u;
  sim_config->dbg_miss_corrupt = 0u;
  sim_config->dbg_miss_emptyhead = 0u;

  DispatchBuffer* dispatch_buffer =
      (DispatchBuffer*)(NPD.task_head.dispatch_buffer);

  let broad_phase_collision_count = min(sim_config->broad_phase_collision_count, BB_MAX_BROAD_PAIR_COUNT);
  let np_count = (broad_phase_collision_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
  dispatch_buffer->narrow_phase_dispatch = daxa_u32vec3(np_count, 1, 1);
}

[shader("compute")]
    [numthreads(1, 1, 1)] void
    entry_island_dispatcher(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      if (compute_i.x >= 1u)
      {
        return;
      }

      SimConfig* sim_config =
          (SimConfig*)(ID.task_head.sim_config);

      DispatchBuffer* dispatch_buffer =
          (DispatchBuffer*)(ID.task_head.dispatch_buffer);

      let i_count = (sim_config->island_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
      dispatch_buffer->island_dispatch = daxa_u32vec3(i_count, 1, 1);
    }

[shader("compute")]
    [numthreads(1, 1, 1)] void entry_collision_solver_dispatcher(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{

  if (compute_i.x >= 1u)
  {
    return;
  }

  SimConfig* sim_config =
      (SimConfig*)(CSD.task_head.sim_config);

  DispatchBuffer* dispatch_buffer =
      (DispatchBuffer*)(CSD.task_head.dispatch_buffer);

  var collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  var collision_c = (collision_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
  dispatch_buffer->collision_dispatch = daxa_u32vec3(collision_c, 1, 1);
}

[shader("compute")]
    [numthreads(1, 1, 1)] void
    entry_contact_island_dispatcher(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      if (compute_i.x >= 1u)
      {
        return;
      }

      SimConfig* sim_config =
          (SimConfig*)(CID.task_head.sim_config);

      DispatchBuffer* dispatch_buffer =
          (DispatchBuffer*)(CID.task_head.dispatch_buffer);

      var contact_island_count = min(sim_config->contact_island_count, BB_MAX_RIGID_BODY_COUNT);
      var ci_count = (contact_island_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
      dispatch_buffer->contact_island_dispatch = daxa_u32vec3(ci_count, 1, 1);
    }
