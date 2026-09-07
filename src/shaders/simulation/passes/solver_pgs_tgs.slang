#include "body_activity.slang"
#include "rigid_body_math.slang"
#include "solvers.slang"
#include "color_solve_dispatch.slang"

[[vk::push_constant]] CollisionPreSolverPushConstants CPS;
[[vk::push_constant]] CollisionSolverPushConstants CS;
[[vk::push_constant]] CollisionSolverRelaxationPushConstants CSR;
[[vk::push_constant]] GraphColorSolvePushConstants GCS;



bool gc_overflowed(daxa_u32 c)
{
  return c == MAX_U32 || c >= BB_MAX_COLORS_SOLVE;
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_collision_pre_solver(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      SimConfig* sim_config =
          (SimConfig*)(CPS.task_head.sim_config);

      daxa_u32 index = compute_i.x;
#if defined(BB_SIM_DEBUG)
      if (index >= 1)
#else
  daxa_u32 contact_island_count = min(sim_config->contact_island_count, BB_MAX_RIGID_BODY_COUNT);
  if (index >= contact_island_count)
#endif // BB_SIM_DEBUG
      {
        return;
      }

#if defined(BB_SIM_DEBUG)
      daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
      for (daxa_u32 manifold_index = 0; manifold_index < collision_count; ++manifold_index)
      {
#else
  ContactIsland* islands =
      (ContactIsland*)(CPS.task_head.contact_islands);

  ManifoldLinkIsland* manifold_links =
      (ManifoldLinkIsland*)(CPS.task_head.manifold_links);

  let collision_map =
      (CollisionEntry*)(CPS.task_head.collision_map);

  daxa_u32 start_index = islands[index].start_index;
  daxa_u32 max_count = islands[index].count;

  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (start_index + max_count > collision_count || max_count == 0)
  {
#if defined(BB_DEBUG)
    printf("Collision pre solver failed index %d, start_index %d, max_count %d\n", index, start_index, max_count);
#endif // BB_DEBUG
    return;
  }

  for (daxa_u32 manifold_index = start_index; manifold_index < start_index + max_count; ++manifold_index)
  {
#endif // BB_SIM_DEBUG

        Manifold* manifolds = (Manifold*)(CPS.task_head.collisions);
        Manifold* manifold = &manifolds[manifold_index];

        if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index)
        {
          continue;
        }

        RigidBody* rigid_bodies = (RigidBody*)(CPS.task_head.rigid_bodies);
        RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
        RigidBody* r2 = &rigid_bodies[manifold->obb2_index];

        if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2))
        {
          continue; // sleeping/static pair: at rest
        }

        daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
        daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
        daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
        daxa::f32 hertz = min(MIN_CONTACT_HERTZ, inv_dt * 0.3333333333333333f);

        if (sim_config->solver_type == SimSolverType::PGS_SOFT)
        {
          PGS_SOFT.pre_step(manifold, inv_dt, r1, r2, accum_impulse, friction, sim_config->dt, hertz);
        }
        else if (sim_config->solver_type == SimSolverType::PGS)
        {
          PGS.pre_step(manifold, inv_dt, r1, r2, accum_impulse, friction);
        }

        // manifold + r1/r2 are pointers into the buffers; pre_step wrote contacts/velocity directly.
      }
    }

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_collision_solver(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(CS.task_head.sim_config);

  daxa_u32 index = compute_i.x;
#if defined(BB_SIM_DEBUG)
  if (index >= 1)
#else
  daxa_u32 contact_island_count = min(sim_config->contact_island_count, BB_MAX_RIGID_BODY_COUNT);
  if (index >= contact_island_count)
#endif // BB_SIM_DEBUG
  {
    return;
  }

#if defined(BB_SIM_DEBUG)
  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  for (daxa_u32 manifold_index = 0; manifold_index < collision_count; ++manifold_index)
  {
#else
  ContactIsland* islands =
      (ContactIsland*)(CS.task_head.contact_islands);

  ManifoldLinkIsland* manifold_links =
      (ManifoldLinkIsland*)(CS.task_head.manifold_links);

  let collision_map =
      (CollisionEntry*)(CS.task_head.collision_map);

  daxa_u32 start_index = islands[index].start_index;
  daxa_u32 max_count = islands[index].count;

  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (start_index + max_count > collision_count || max_count == 0)
  {
#if defined(BB_DEBUG)
    printf("Collision solver failed index %d, start_index %d, max_count %d\n", index, start_index, max_count);
#endif // BB_DEBUG
    return;
  }

  for (daxa_u32 manifold_index = start_index; manifold_index < start_index + max_count; ++manifold_index)
  {
#endif // BB_SIM_DEBUG

    Manifold* manifolds = (Manifold*)(CS.task_head.collisions);
    Manifold* manifold = &manifolds[manifold_index];

    if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index)
    {
      continue;
    }

    // Raw `RigidBody*` (scalar layout) instead of value-copy through the daxa BufferPtr.
    RigidBody* rigid_bodies = (RigidBody*)(CS.task_head.rigid_bodies);
    RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
    RigidBody* r2 = &rigid_bodies[manifold->obb2_index];

    if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2))
    {
      continue; // sleeping/static pair: at rest
    }

    daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
    daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
    daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;

    // Solve the collision
    if (sim_config->solver_type == SimSolverType::PGS_SOFT)
    {
      PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, true, inv_dt);
    }
    else if (sim_config->solver_type == SimSolverType::PGS)
    {
      PGS.solve(manifold, r1, r2, accum_impulse, friction);
    }

    // manifold + r1/r2 are pointers into the buffers; solve wrote contacts/velocity directly.
  }
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_collision_solver_relaxation(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(CSR.task_head.sim_config);

  daxa_u32 index = compute_i.x;

#if defined(BB_SIM_DEBUG)
  if (index >= 1)
#else
  daxa_u32 contact_island_count = min(sim_config->contact_island_count, BB_MAX_RIGID_BODY_COUNT);
  if (index >= contact_island_count)
#endif // BB_SIM_DEBUG
  {
    return;
  }

#if defined(BB_SIM_DEBUG)
  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  for (daxa_u32 manifold_index = 0; manifold_index < collision_count; ++manifold_index)
  {
#else
  ContactIsland* islands =
      (ContactIsland*)(CSR.task_head.contact_islands);

  ManifoldLinkIsland* manifold_links =
      (ManifoldLinkIsland*)(CSR.task_head.manifold_links);

  let collision_map =
      (CollisionEntry*)(CSR.task_head.collision_map);

  daxa_u32 start_index = islands[index].start_index;
  daxa_u32 max_count = islands[index].count;

  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (start_index + max_count > collision_count || max_count == 0)
  {
#if defined(BB_DEBUG)
    printf("Collision solver relaxation failed index %d, start_index %d, max_count %d\n", index, start_index, max_count);
#endif // BB_DEBUG
    return;
  }

  for (daxa_u32 manifold_index = start_index; manifold_index < start_index + max_count; ++manifold_index)
  {
#endif // BB_SIM_DEBUG

    Manifold* manifolds = (Manifold*)(CSR.task_head.collisions);
    Manifold* manifold = &manifolds[manifold_index];

    if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index)
    {
      continue;
    }

    // Raw `RigidBody*` (scalar layout) instead of value-copy through the daxa BufferPtr.
    RigidBody* rigid_bodies = (RigidBody*)(CSR.task_head.rigid_bodies);
    RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
    RigidBody* r2 = &rigid_bodies[manifold->obb2_index];

    if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2))
    {
      continue; // sleeping/static pair: at rest
    }

    daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
    daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
    daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;

    // Solve the collision with relaxation
    if (sim_config->solver_type == SimSolverType::PGS_SOFT)
    {
      PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, false, inv_dt);
    }

    // manifold + r1/r2 are pointers into the buffers; solve wrote contacts/velocity directly.
  }
}

[shader("compute")]
[numthreads(COLOR_SOLVE_COMPUTE_X, 1, 1)] void entry_collision_pre_solver_color(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(GCS.task_head.sim_config);
  daxa_u32 m = compute_i.x;
  let coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (m >= coll) return;
  daxa_u32* manifold_color = (daxa_u32*)(GCS.task_head.manifold_color);
  if (manifold_color[m] != GCS.color) return;
  Manifold* manifold = &((Manifold*)(GCS.task_head.collisions))[m];
  if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index) return;
  RigidBody* rigid_bodies = (RigidBody*)(GCS.task_head.rigid_bodies);
  RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
  RigidBody* r2 = &rigid_bodies[manifold->obb2_index];
  if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2)) return; // sleeping/static pair: at rest
  daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
  daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
  daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
  daxa::f32 hertz = min(MIN_CONTACT_HERTZ, inv_dt * 0.3333333333333333f);
  // DIAG stage 4: runaway accumulator INHERITED from last frame (warm-start bomb already armed)
  // DIAG stage 6: contact point absurdly far from the body (ill-conditioned closest-point solve)
  for (daxa_i32 ci = 0; ci < manifold->contact_count; ++ci)
  {
    bb_dbg_impulse_probe(sim_config, 4u, manifold->obb1_index, manifold->obb2_index,
                         manifold->contacts[ci].normal_impulse, manifold->contacts[ci].penetration, manifold->normal.y);
    bb_dbg_far_contact_probe(sim_config, manifold->obb1_index, manifold->obb2_index,
                             manifold->contacts[ci].position - manifold->anchor_a,
                             manifold->contacts[ci].penetration, manifold->normal.y);
  }
  // TGS_SOFT sub-step gating: tgs_phase==1 prepares the TGS contacts (soft coeffs with sub-step h
  // + local anchors); tgs_phase==0 (the normal PGS-family pre instance) skips TGS.
  if (GCS.tgs_phase == 1)
  {
    if (sim_config->solver_type != SimSolverType::TGS_SOFT) return;
    daxa::f32 h = sim_config->dt / daxa_f32(sim_config->tgs_substeps);
    daxa::f32 inv_h = h > 0.0f ? 1.0f / h : 0.0f;
    daxa::f32 tgs_hertz = min(TGS_CONTACT_HERTZ, inv_h * 0.3333333333333333f);
    TGS_SOFT.pre_step(manifold, r1, r2, h, tgs_hertz, sim_config, m);
    return;
  }
  if (GCS.tgs_phase == 2) // Box2D v3's per-sub-step warm start stage
  {
    if (sim_config->solver_type != SimSolverType::TGS_SOFT) return;
    TGS_SOFT.warm_start(manifold, r1, r2, accum_impulse, friction);
    return;
  }
  if (sim_config->solver_type == SimSolverType::TGS_SOFT) return; // TGS runs its own sub-step loop
  if (sim_config->solver_type == SimSolverType::PGS_SOFT)
    PGS_SOFT.pre_step(manifold, inv_dt, r1, r2, accum_impulse, friction, sim_config->dt, hertz);
}

[shader("compute")]
[numthreads(COLOR_SOLVE_COMPUTE_X, 1, 1)] void entry_collision_solver_color(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(GCS.task_head.sim_config);
  daxa_u32 m = compute_i.x;
  let coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (m >= coll) return;
  daxa_u32* manifold_color = (daxa_u32*)(GCS.task_head.manifold_color);
  if (manifold_color[m] != GCS.color) return;
  Manifold* manifold = &((Manifold*)(GCS.task_head.collisions))[m];
  if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index) return;
  RigidBody* rigid_bodies = (RigidBody*)(GCS.task_head.rigid_bodies);
  RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
  RigidBody* r2 = &rigid_bodies[manifold->obb2_index];
  if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2)) return; // sleeping/static pair: at rest
  daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
  daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
  daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
  if (GCS.tgs_phase == 1)
  {
    if (sim_config->solver_type != SimSolverType::TGS_SOFT) return;
    daxa::f32 inv_h = daxa_f32(sim_config->tgs_substeps) * inv_dt; // 1 / (dt/N)
    TGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, true, inv_h);
    return;
  }
  if (sim_config->solver_type == SimSolverType::TGS_SOFT) return;
  if (sim_config->solver_type == SimSolverType::PGS_SOFT)
    PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, true, inv_dt);
}

[shader("compute")]
[numthreads(COLOR_SOLVE_COMPUTE_X, 1, 1)] void entry_collision_solver_relax_color(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(GCS.task_head.sim_config);
  daxa_u32 m = compute_i.x;
  let coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (m >= coll) return;
  daxa_u32* manifold_color = (daxa_u32*)(GCS.task_head.manifold_color);
  if (manifold_color[m] != GCS.color) return;
  Manifold* manifold = &((Manifold*)(GCS.task_head.collisions))[m];
  if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index) return;
  RigidBody* rigid_bodies = (RigidBody*)(GCS.task_head.rigid_bodies);
  RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
  RigidBody* r2 = &rigid_bodies[manifold->obb2_index];
  if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2)) return; // sleeping/static pair: at rest
  daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
  daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
  daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
  // DIAG stage 5: runaway accumulator BUILT this frame by the CS sweep (seen entering relax)
  for (daxa_i32 ci = 0; ci < manifold->contact_count; ++ci)
    bb_dbg_impulse_probe(sim_config, 5u, manifold->obb1_index, manifold->obb2_index,
                         manifold->contacts[ci].normal_impulse, manifold->contacts[ci].penetration, manifold->normal.y);
  if (GCS.tgs_phase == 1)
  {
    if (sim_config->solver_type != SimSolverType::TGS_SOFT) return;
    daxa::f32 inv_h = daxa_f32(sim_config->tgs_substeps) * inv_dt;
    TGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, false, inv_h); // relax: no bias
    return;
  }
  if (sim_config->solver_type == SimSolverType::TGS_SOFT) return;
  if (sim_config->solver_type == SimSolverType::PGS_SOFT)
    PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, false, inv_dt);
}

[shader("compute")]
[numthreads(1, 1, 1)] void entry_collision_pre_solver_overflow(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  if (compute_i.x >= 1u) return;
  SimConfig* sim_config = (SimConfig*)(GCS.task_head.sim_config);
  // Diagnostic reference: preserve color order, serialize every manifold within each color.
  bool serial_all = GCS.color == MAX_U32;
  if (!serial_all && sim_config->graph_color_overflow == 0u) return;
  let coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  daxa_u32* manifold_color = (daxa_u32*)(GCS.task_head.manifold_color);
  RigidBody* rigid_bodies = (RigidBody*)(GCS.task_head.rigid_bodies);
  daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
  daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
  daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
  daxa::f32 hertz = min(MIN_CONTACT_HERTZ, inv_dt * 0.3333333333333333f);
  for (daxa_u32 pass = 0u; pass < (serial_all ? BB_MAX_COLORS_SOLVE + 1u : 1u); ++pass)
  for (daxa_u32 m = 0u; m < coll; ++m)
  {
    if (serial_all && pass < BB_MAX_COLORS_SOLVE)
    {
      if (manifold_color[m] != pass) continue;
    }
    else if (!gc_overflowed(manifold_color[m])) continue;
    Manifold* manifold = &((Manifold*)(GCS.task_head.collisions))[m];
    if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index) continue;
    RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
    RigidBody* r2 = &rigid_bodies[manifold->obb2_index];
    if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2)) continue; // sleeping/static pair: at rest
    if (sim_config->solver_type == SimSolverType::TGS_SOFT)
    {
      // TGS overflow: mirror the per-color path (phase 1 = prepare, phase 2 = warm start).
      // Without this the manifolds the colored dispatches skip are never solved under TGS.
      if (GCS.tgs_phase == 1)
      {
        daxa::f32 h = sim_config->dt / daxa_f32(sim_config->tgs_substeps);
        daxa::f32 inv_h_sub = h > 0.0f ? 1.0f / h : 0.0f;
        daxa::f32 tgs_hertz = min(TGS_CONTACT_HERTZ, inv_h_sub * 0.3333333333333333f);
        TGS_SOFT.pre_step(manifold, r1, r2, h, tgs_hertz, sim_config, m);
      }
      else if (GCS.tgs_phase == 2)
        TGS_SOFT.warm_start(manifold, r1, r2, accum_impulse, friction);
      continue;
    }
    if (sim_config->solver_type == SimSolverType::PGS_SOFT)
      PGS_SOFT.pre_step(manifold, inv_dt, r1, r2, accum_impulse, friction, sim_config->dt, hertz);
  }
}

[shader("compute")]
[numthreads(1, 1, 1)] void entry_collision_solver_overflow(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  if (compute_i.x >= 1u) return;
  SimConfig* sim_config = (SimConfig*)(GCS.task_head.sim_config);
  // Diagnostic reference: preserve color order, serialize every manifold within each color.
  bool serial_all = GCS.color == MAX_U32;
  if (!serial_all && sim_config->graph_color_overflow == 0u) return;
  let coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  daxa_u32* manifold_color = (daxa_u32*)(GCS.task_head.manifold_color);
  RigidBody* rigid_bodies = (RigidBody*)(GCS.task_head.rigid_bodies);
  daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
  daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
  daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
  for (daxa_u32 pass = 0u; pass < (serial_all ? BB_MAX_COLORS_SOLVE + 1u : 1u); ++pass)
  for (daxa_u32 m = 0u; m < coll; ++m)
  {
    if (serial_all && pass < BB_MAX_COLORS_SOLVE)
    {
      if (manifold_color[m] != pass) continue;
    }
    else if (!gc_overflowed(manifold_color[m])) continue;
    Manifold* manifold = &((Manifold*)(GCS.task_head.collisions))[m];
    if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index) continue;
    RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
    RigidBody* r2 = &rigid_bodies[manifold->obb2_index];
    if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2)) continue; // sleeping/static pair: at rest
    if (sim_config->solver_type == SimSolverType::TGS_SOFT)
    {
      TGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, true, daxa_f32(sim_config->tgs_substeps) * inv_dt);
      continue;
    }
    if (sim_config->solver_type == SimSolverType::PGS_SOFT)
      PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, true, inv_dt);
  }
}

[shader("compute")]
[numthreads(1, 1, 1)] void entry_collision_solver_relax_overflow(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  if (compute_i.x >= 1u) return;
  SimConfig* sim_config = (SimConfig*)(GCS.task_head.sim_config);
  // Diagnostic reference: preserve color order, serialize every manifold within each color.
  bool serial_all = GCS.color == MAX_U32;
  if (!serial_all && sim_config->graph_color_overflow == 0u) return;
  let coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  daxa_u32* manifold_color = (daxa_u32*)(GCS.task_head.manifold_color);
  RigidBody* rigid_bodies = (RigidBody*)(GCS.task_head.rigid_bodies);
  daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
  daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
  daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
  for (daxa_u32 pass = 0u; pass < (serial_all ? BB_MAX_COLORS_SOLVE + 1u : 1u); ++pass)
  for (daxa_u32 m = 0u; m < coll; ++m)
  {
    if (serial_all && pass < BB_MAX_COLORS_SOLVE)
    {
      if (manifold_color[m] != pass) continue;
    }
    else if (!gc_overflowed(manifold_color[m])) continue;
    Manifold* manifold = &((Manifold*)(GCS.task_head.collisions))[m];
    if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index) continue;
    RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
    RigidBody* r2 = &rigid_bodies[manifold->obb2_index];
    if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2)) continue; // sleeping/static pair: at rest
    if (sim_config->solver_type == SimSolverType::TGS_SOFT)
    {
      TGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, false, daxa_f32(sim_config->tgs_substeps) * inv_dt);
      continue;
    }
    if (sim_config->solver_type == SimSolverType::PGS_SOFT)
      PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, false, inv_dt);
  }
}
