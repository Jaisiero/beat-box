#include "body_activity.slang"
#include "rigid_body_math.slang"
#include "rigid_body_integration.slang"
#include "state_copy.slang"

const static daxa_u32 AXES_POINT_COUNT = 6;

[[vk::push_constant]] RigidBodySimPushConstants RB_sim;
[[vk::push_constant]] CollisionSolverPushConstants CS;
[[vk::push_constant]] RigidBodyIntegratePositionsPushConstants RB_integrate;
[[vk::push_constant]] CollisionSolverRelaxationPushConstants CSR;
[[vk::push_constant]] RigidBodyUpdatePushConstants RB_update;



[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_rigid_body_sim(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      SimConfig* sim_config =
          (SimConfig*)(RB_sim.task_head.sim_config);

      daxa_u32 index = compute_i.x;
      if (index >= sim_config->rigid_body_count)
      {
        return;
      }

      RigidBody* rigid_bodies = (RigidBody*)(RB_sim.task_head.rigid_bodies);
      RigidBody* r = &rigid_bodies[index];

      // Static bodies never move: skip them so solver-injected velocity (e.g. concurrent impulse
      // writes on a shared static body) can never accumulate or drift their state.
      // Sleeping bodies skip too: their island is at rest (velocities were zeroed at sleep onset).
      if (!rb_is_awake_dynamic(r))
      {
        return;
      }

      // AVBD folds gravity into its inertial target (entry_avbd_prepare) and reconstructs
      // velocities itself; this advect pass belongs to the PGS family only.
      if (sim_config->solver_type == SimSolverType::AVBD)
      {
        return;
      }
      // TGS sub-step gating: tgs_phase==1 instances run ONLY for TGS (per sub-step in the loop);
      // tgs_phase==0 instances (the once-per-frame advect) skip TGS so gravity isn't double-applied.
      if (RB_sim.tgs_phase == 1)
      {
        if (sim_config->solver_type != SimSolverType::TGS_SOFT) return;
      }
      else if (sim_config->solver_type == SimSolverType::TGS_SOFT)
      {
        return;
      }

      if (sim_config->has_flag(SimFlag::ADVECTION))
      {
        r->advect(sim_config);
      }

      // p_r->manifold_node_index = MAX_U32;
    }

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_integrate_positions_rigid_bodies(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      SimConfig* sim_config =
          (SimConfig*)(RB_integrate.task_head.sim_config);

      if (compute_i.x >= sim_config->rigid_body_count)
      {
        return;
      }

      daxa_u32 index = compute_i.x;

      RigidBody* rigid_bodies = (RigidBody*)(RB_integrate.task_head.rigid_bodies);
      RigidBody* r = &rigid_bodies[index];

      // Static bodies never integrate (they'd drift from solver-raced velocity writes);
      // sleeping bodies hold their pose until their island wakes.
      if (!rb_is_awake_dynamic(r))
      {
        return;
      }

      // AVBD writes positions directly during the primal descent; entry_avbd_finalize
      // reconstructs velocities. This velocity-level integration is PGS-family only.
      if (sim_config->solver_type == SimSolverType::AVBD)
      {
        return;
      }
      // TGS sub-step gating (mirror of advect): tgs_phase==1 = the per-sub-step integrate for TGS;
      // tgs_phase==0 = the once-per-frame integrate for the PGS family (skips TGS).
      if (RB_integrate.tgs_phase == 1)
      {
        if (sim_config->solver_type != SimSolverType::TGS_SOFT) return;
      }
      else if (sim_config->solver_type == SimSolverType::TGS_SOFT)
      {
        return;
      }

      // DIAG probe stage 2 (after the CS sweep, before positions move): huge |v| first seen here
      // means warm-start/CS injected it this frame.
      bb_dbg_velocity_probe(sim_config, 2u, index, r->velocity, r->position.y);

      // Integrate positions (TGS uses the sub-step h = dt/N; others the full dt)
      if (sim_config->has_flag(SimFlag::ADVECTION))
      {
        daxa_f32 h = (sim_config->solver_type == SimSolverType::TGS_SOFT)
                     ? sim_config->dt / daxa_f32(sim_config->tgs_substeps) : sim_config->dt;
        r->integrate_positions(h);
      }
    }

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_update_rigid_bodies(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(RB_update.task_head.sim_config);

  if (compute_i.x >= sim_config->rigid_body_count)
  {
    return;
  }

  daxa_u32 index = compute_i.x;

  // FIXME: split between static and dynamic objects?
  RigidBody* rigid_bodies = (RigidBody*)(RB_update.task_head.rigid_bodies);
  RigidBody* rigid_bodies_update = (RigidBody*)(RB_update.task_head.rigid_bodies_update);
  RigidBody* r = &rigid_bodies[index];
  RigidBody* r_copy = &rigid_bodies_update[index];

  // DIAG: row-permutation checksum. Sum of ids over all rows is constant (n*(n-1)/2) unless the
  // sort/reorder duplicated one body row and dropped another (destroying a body).
  daxa_u32 _prev;
  InterlockedAdd(sim_config->dbg_id_sum, r->id, _prev);
  // DIAG probe stage 3 (end of frame, after the CSR relax sweep).
  if (r->has_flag(RigidBodyFlag::DYNAMIC))
  {
    bb_dbg_velocity_probe(sim_config, 3u, index, r->velocity, r->position.y);
    // DETERMINISM gauges: XOR of the raw pose bits of every dynamic body. XOR is exactly
    // commutative/associative, so the hash is independent of thread order - two same-seed
    // runs must print identical values (constant once the pile fully sleeps); the first
    // diverging sample localizes any residual nondeterminism.
    if (sim_config->has_flag(SimFlag::DET_HASHES))
    {
      daxa_u32 ph = asuint(r->position.x) ^ (asuint(r->position.y) * 0x9e3779b9u) ^
                    (asuint(r->position.z) * 0x85ebca6bu) ^ (r->id * 0xc2b2ae35u);
      daxa_u32 rh = asuint(r->rotation.v.x) ^ (asuint(r->rotation.v.y) * 0x9e3779b9u) ^
                    (asuint(r->rotation.v.z) * 0x85ebca6bu) ^ (asuint(r->rotation.w) * 0x27d4eb2fu) ^
                    (r->id * 0xc2b2ae35u);
      InterlockedXor(sim_config->dbg_poshash, ph, _prev);
      InterlockedXor(sim_config->dbg_rothash, rh, _prev);
    }
  }

  copy_rigid_body_fields(r_copy, r);
  r_copy->island_index = MAX_U32;
  r_copy->active_index = MAX_U32;
  r_copy->manifold_node_index = MAX_U32;

  if (sim_config->has_flag(SimFlag::DEBUG_INFO))
  {
    daxa::f32vec3 _min = r->minimum;
    daxa::f32vec3 _max = r->maximum;
    daxa::f32vec3 center = (_min + _max) * 0.5f;
    var axes_vertex_buffer = (GUIVertexLine*)(RB_update.task_head.axes_vertex_buffer);
    let x_axis = r->rotate_vector(daxa::f32vec3(1.0f, 0.0f, 0.0f));
    let y_axis = r->rotate_vector(daxa::f32vec3(0.0f, 1.0f, 0.0f));
    let z_axis = r->rotate_vector(daxa::f32vec3(0.0f, 0.0f, 1.0f));

    center = r->object_to_world(center);

    axes_vertex_buffer[index * AXES_POINT_COUNT].position = center;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 1].position = center + x_axis;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 2].position = center;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 3].position = center + y_axis;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 4].position = center;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 5].position = center + z_axis;
  }

#if defined(BB_DEBUG)
  // printf("Updating rigid body %d sim frame: %lu c frame: %lu\n", index, r.frame_count, r_copy.frame_count);
#endif // BB_DEBUG
}
