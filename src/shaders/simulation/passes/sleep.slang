#include "shared.inl"

[[vk::push_constant]] SleepPushConstants SLP;

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_sleep_reduce(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(SLP.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  if (index == 0u)
  {
    sim_config->sleeping_count = 0u; // recomputed by entry_sleep_apply (separate dispatch)
  }
  if (index >= sim_config->rigid_body_count)
  {
    return;
  }

  RigidBody* rigid_bodies = (RigidBody*)(SLP.task_head.rigid_bodies);
  RigidBody* r = &rigid_bodies[index];
  if (!r->has_flag(RigidBodyFlag::DYNAMIC))
  {
    return;
  }

  if (!sim_config->has_flag(SimFlag::SLEEPING_ENABLED))
  {
    // feature disabled: everything stays awake
    r->sleep_timer = 0u;
    r->flags = RigidBodyFlag(daxa_u32(r->flags) & ~daxa_u32(RigidBodyFlag::SLEEPING));
    return;
  }

  // SUPPORT-LOSS WAKE (user-found "floating sleeper"): a sleeping body's supporters can
  // slide out from under it during an avalanche WITHOUT touching it - the neighborhood
  // veto only propagates through EXISTING manifolds, so the frozen body never re-evaluates
  // its support and floats mid-air forever. An EMPTY manifold chain this frame means
  // contactless (the chain head is reset to MAX_U32 every frame in the end-of-frame copy
  // and re-linked by the narrow phase, which runs before this pass). Bleed the timer hard
  // instead of zeroing: settling piles open 1-frame mm-gaps under sleepers when neighbors
  // extract, and waking on a single gap frame is a wake-storm pump - a REAL floater stays
  // contactless and wakes after ~3 consecutive frames (timer < threshold -> apply() wakes,
  // gravity resumes), which is still visually instant.
  if (r->manifold_node_index == MAX_U32 &&
      (r->flags & RigidBodyFlag::SLEEPING) != RigidBodyFlag::NONE)
  {
    daxa_u32 t = r->sleep_timer & BB_SLEEP_TIMER_MASK;
    r->sleep_timer = t >= 10u ? t - 10u : 0u;
    return;
  }

  daxa_f32 lin2 = dot(r->velocity, r->velocity);
  daxa_f32 ang2 = dot(r->omega, r->omega);
  daxa_u32 timer = r->sleep_timer & BB_SLEEP_TIMER_MASK; // also clears last step's veto bit
  // DECAY instead of hard reset: a settling crust body spikes above the velocity
  // threshold for a frame or two and used to pay 30 fresh quiet steps each time —
  // that re-arming is most of the full-sleep tail. Capping the timer at the threshold
  // and bleeding 4/loud-frame makes recovery proportional to the disturbance (~5x its
  // duration, max 30): a 1-frame spike costs 4 quiet steps, a real kick re-arms fully.
  // Genuine oscillators that are quiet >4/5 of the time get ratcheted to sleep — that
  // FREEZES the limit cycle (velocity zeroed), and the depth/partner vetoes still hold.
  bool calm = lin2 < BB_SLEEP_LIN_VEL2 && ang2 < BB_SLEEP_ANG_VEL2;
  r->sleep_timer = calm ? min(timer + 1u, BB_SLEEP_STEPS) : (timer >= 4u ? timer - 4u : 0u);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_sleep_veto(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(SLP.task_head.sim_config);
  if (!sim_config->has_flag(SimFlag::SLEEPING_ENABLED))
  {
    return;
  }

  daxa_u32 m = compute_i.x;
  daxa_u32 coll_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (m >= coll_count)
  {
    return;
  }

  Manifold* manifolds = (Manifold*)(SLP.task_head.collisions);
  daxa_u32 a = manifolds[m].obb1_index;
  daxa_u32 b = manifolds[m].obb2_index;
  if (a >= sim_config->rigid_body_count || b >= sim_config->rigid_body_count || a == b)
  {
    return;
  }

  RigidBody* rigid_bodies = (RigidBody*)(SLP.task_head.rigid_bodies);
  RigidBody* ra = &rigid_bodies[a];
  RigidBody* rb = &rigid_bodies[b];
  bool a_dyn = ra->has_flag(RigidBodyFlag::DYNAMIC);
  bool b_dyn = rb->has_flag(RigidBodyFlag::DYNAMIC);
  // statics are always "quiet"; only a non-quiet DYNAMIC partner vetoes
  // Other invocations atomically set the veto bit in this same word.
  daxa_u32 timer_a, timer_b;
  InterlockedOr(ra->sleep_timer, 0u, timer_a);
  InterlockedOr(rb->sleep_timer, 0u, timer_b);
  bool a_loud = a_dyn && ((timer_a & BB_SLEEP_TIMER_MASK) < BB_SLEEP_STEPS);
  bool b_loud = b_dyn && ((timer_b & BB_SLEEP_TIMER_MASK) < BB_SLEEP_STEPS);
  // OBB contacts must satisfy the penetration tolerance before sleeping. A flat
  // depth trend is not evidence that an overlapping pair is resolved: freezing it
  // would preserve the overlap permanently. Voxel contacts retain their existing
  // discrete-surface policy, independently of the unit-box convergence criterion.
  bool voxel_pair = ra->shape_index != 0u || rb->shape_index != 0u;
  bool stalled = manifolds[m].error != 0x7FFFFFFF && abs(manifolds[m].error) < 10;
  daxa_f32 depth_limit = voxel_pair ? 0.05f : PENETRATION_FACTOR;
  bool deep = false;
  if (!voxel_pair || !stalled)
  {
    for (daxa_i32 ci = 0; ci < manifolds[m].contact_count; ++ci)
    {
      if (manifolds[m].contacts[ci].penetration < -depth_limit) { deep = true; break; }
    }
  }
  daxa_u32 prev;
  if (deep && !voxel_pair)
  {
    // All connected bodies must be free to resolve a deep overlap. Waking only
    // the pair can trap it between neighbors already treated as immovable.
    Island* islands = (Island*)(SLP.task_head.islands);
    daxa_u32 island_count = min(sim_config->island_count, BB_MAX_RIGID_BODY_COUNT);
    daxa_u32 island_a = rigid_bodies[a].island_index;
    daxa_u32 island_b = rigid_bodies[b].island_index;
    if (a_dyn && island_a < island_count)
      InterlockedOr(islands[island_a].sleep_veto, 1u, prev);
    if (b_dyn && island_b < island_count)
      InterlockedOr(islands[island_b].sleep_veto, 1u, prev);
  }
  if ((a_loud || deep) && b_dyn)
  {
    InterlockedOr(rb->sleep_timer, BB_SLEEP_VETO_BIT, prev);
  }
  if ((b_loud || deep) && a_dyn)
  {
    InterlockedOr(ra->sleep_timer, BB_SLEEP_VETO_BIT, prev);
  }
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_sleep_apply(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(SLP.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  if (index >= sim_config->rigid_body_count)
  {
    return;
  }
  if (!sim_config->has_flag(SimFlag::SLEEPING_ENABLED))
  {
    return; // reduce already cleared the flags
  }

  RigidBody* rigid_bodies = (RigidBody*)(SLP.task_head.rigid_bodies);
  RigidBody* r = &rigid_bodies[index];
  if (!r->has_flag(RigidBodyFlag::DYNAMIC))
  {
    return;
  }

  bool vetoed = (r->sleep_timer & BB_SLEEP_VETO_BIT) != 0u;
  // The preceding veto dispatch publishes this field through the islands RW
  // attachment; this dispatch only reads it, so no atomic load is needed here.
  Island* islands = (Island*)(SLP.task_head.islands);
  daxa_u32 island_count = min(sim_config->island_count, BB_MAX_RIGID_BODY_COUNT);
  daxa_u32 body_island = rigid_bodies[index].island_index;
  if (body_island < island_count)
    vetoed = vetoed || islands[body_island].sleep_veto != 0u;
  bool quiet = (r->sleep_timer & BB_SLEEP_TIMER_MASK) >= BB_SLEEP_STEPS;
  bool sleep = quiet && !vetoed;
  bool was_sleeping = r->has_flag(RigidBodyFlag::SLEEPING);
  if (sleep)
  {
    if (!was_sleeping)
    {
      // falling asleep: freeze residual motion so the body holds rock-solid and wakes cleanly
      r->velocity = daxa_f32vec3(0.0f, 0.0f, 0.0f);
      r->omega = daxa_f32vec3(0.0f, 0.0f, 0.0f);
      r->flags = RigidBodyFlag(daxa_u32(r->flags) | daxa_u32(RigidBodyFlag::SLEEPING));
    }
    daxa_u32 prev;
    InterlockedAdd(sim_config->sleeping_count, 1u, prev);
  }
  else if (was_sleeping)
  {
    r->flags = RigidBodyFlag(daxa_u32(r->flags) & ~daxa_u32(RigidBodyFlag::SLEEPING));
  }
}
