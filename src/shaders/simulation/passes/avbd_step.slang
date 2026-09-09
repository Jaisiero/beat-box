#include "rigid_body_rotation.slang"
#include "avbd_common.slang"

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_prepare(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(AVBD_PC.task_head.sim_config);
  if (sim_config->solver_type != SimSolverType::AVBD)
  {
    return;
  }
  daxa_u32 i = compute_i.x;
  if (i == 0u)
  {
    sim_config->avbd_stick_count = 0u; // re-accumulated by the warm-start dispatch
    // AVBD runs a SEPARATE task graph that skips entry_graph_color_reset (where the PGS/TGS path
    // resets these per-frame diagnostics). Without this, dbg_min_y stays at its init value 0 and
    // miny decodes to a PHANTOM -100 every AVBD frame (not a real floor ejection); dbg_maxv would
    // become a monotonic run-max instead of per-frame. Reset them here, symmetric to coloring.slang.
    sim_config->dbg_maxv = 0u;
    sim_config->dbg_min_y = MAX_U32;
    // deep-pocket trace latches: same phantom as dbg_min_y above - their reset lives in
    // entry_graph_color_reset, which the AVBD graph SKIPS, so under AVBD the trace froze
    // the run-max (a frame-2 rain transient read as a "stuck 250mm pair" for 12000 frames)
    sim_config->dbg_pk_pen = 0u;
    sim_config->dbg_pk_body = 0u;
    sim_config->dbg_pk_lambda = 0.0f;
    sim_config->dbg_pk_k = 0.0f;
    sim_config->dbg_pk_vn = 0.0f;
    sim_config->dbg_pk_stick = 0u;
    sim_config->dbg_pk_omega = 0u;
    // convergence early-out state: fresh iteration counter + residual slots per step
    // (iteration 0 InterlockedMax-es into res[0]; stale values would poison the max)
    sim_config->avbd_iter_tick = 0u;
    sim_config->avbd_step_res[0] = 0u;
    sim_config->avbd_step_res[1] = 0u;
  }
  if (i >= sim_config->rigid_body_count)
  {
    return;
  }

  RigidBody* rigid_bodies = (RigidBody*)(AVBD_PC.task_head.rigid_bodies);
  RigidBody* r = &rigid_bodies[i];
  if (!avbd_is_dynamic(r) || (r->flags & RigidBodyFlag::SLEEPING) != RigidBodyFlag::NONE)
  {
    return; // statics anchor; sleeping bodies hold their pose
  }

  daxa_f32 dt = sim_config->dt;
  AvbdBodyState* states = (AvbdBodyState*)(AVBD_PC.task_head.avbd_state);
  AvbdBodyState* s = &states[i];

  // Each node belongs to one body's chain, so these writes are disjoint.
  // Preserve its canonical order while resolving the scratch-to-packed map once.
  ManifoldNode* nodes = (ManifoldNode*)AVBD_PC.task_head.manifold_nodes;
  CollisionEntry* map = (CollisionEntry*)AVBD_PC.task_head.collision_map;
  daxa_u32 node_count = min(sim_config->manifold_node_count, BB_MAX_MANIFOLD_NODE_COUNT);
  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  daxa_u32vec2* links = avbd_resolved_links(states);
  daxa_u32 node = r->manifold_node_index;
  for (daxa_u32 visited = 0u; node < node_count && visited < node_count; ++visited)
  {
    daxa_u32 packed = avbd_node_to_packed(node, node_count, collision_count, nodes, map);
    daxa_i32 next = nodes[node].next;
    daxa_u32 successor = next < 0 ? MAX_U32 : daxa_u32(next);
    links[node] = daxa_u32vec2(packed, successor);
    node = successor;
  }

  // anti-spin clamp (reference solver.cpp)
  daxa_f32 om_len = length(r->omega);
  if (om_len > 50.0f)
  {
    r->omega *= 50.0f / om_len;
  }

  // terminal linear-speed clamp (anti-punch-through): cap impact energy so fast rain doesn't tunnel
  // deep into the floor. Applied here so the inertial target + warm-start jump below use the clamped
  // velocity. Resting/settling bodies are far under the cap, so they are untouched.
  daxa_f32 v_len = length(r->velocity);
  if (v_len > BB_MAX_LINEAR_SPEED)
  {
    r->velocity *= BB_MAX_LINEAR_SPEED / v_len;
  }

  daxa_f32 grav = ((r->flags & RigidBodyFlag::GRAVITY) != RigidBodyFlag::NONE) ? sim_config->gravity : 0.0f;

  // inertial target (Eq 2): full gravity
  s->pos_start = r->position;
  s->rot_start.v = r->rotation.v;
  s->rot_start.w = r->rotation.w;
  s->pos_tilde = r->position + r->velocity * dt + daxa_f32vec3(0.0f, grav, 0.0f) * (dt * dt);
  Quaternion q_tilde = r->apply_angular_velocity(r->rotation, r->omega, dt);
  s->rot_tilde.v = q_tilde.v;
  s->rot_tilde.w = q_tilde.w;

  // adaptive warm start (original VBD): start the primal state at pos + v*dt plus only the
  // fraction of gravity that was actually realized last step (0 when resting on support)
  daxa_f32 accel_w = 0.0f;
  if (abs(grav) > 1e-6f)
  {
    daxa_f32 accel_y = (r->velocity.y - r->prev_velocity.y) / dt;
    accel_w = clamp(accel_y * sign(grav) / abs(grav), 0.0f, 1.0f);
  }
  r->position = r->position + r->velocity * dt + daxa_f32vec3(0.0f, grav, 0.0f) * (accel_w * dt * dt);
  r->rotation.v = q_tilde.v;
  r->rotation.w = q_tilde.w;

  // DETERMINISM CHECKPOINT 2 (debug): hash the post-PRE (predicted) pose, BEFORE the primal sweeps.
  // cp2 matches but cp (FIN) diverges => the primal sweeps are the source. Same gate as the FIN hash.
  if (sim_config->has_flag(SimFlag::DET_HASHES) && avbd_is_dynamic(r) && (r->flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE)
  {
    daxa_u32 c2ph = asuint(r->position.x) ^ (asuint(r->position.y) * 0x9e3779b9u) ^
                    (asuint(r->position.z) * 0x85ebca6bu) ^ (r->id * 0xc2b2ae35u);
    daxa_u32 c2rh = asuint(r->rotation.v.x) ^ (asuint(r->rotation.v.y) * 0x9e3779b9u) ^
                    (asuint(r->rotation.v.z) * 0x85ebca6bu) ^ (asuint(r->rotation.w) * 0x27d4eb2fu) ^
                    (r->id * 0xc2b2ae35u);
    daxa_u32 _c2prev;
    InterlockedXor(sim_config->dbg_cp2_poshash, c2ph, _c2prev);
    InterlockedXor(sim_config->dbg_cp2_rothash, c2rh, _c2prev);
  }
}

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_finalize(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(AVBD_PC.task_head.sim_config);
  if (sim_config->solver_type != SimSolverType::AVBD)
  {
    return;
  }
  daxa_u32 i = compute_i.x;
  if (i >= sim_config->rigid_body_count)
  {
    return;
  }

  RigidBody* rigid_bodies = (RigidBody*)(AVBD_PC.task_head.rigid_bodies);
  RigidBody* r = &rigid_bodies[i];
  if (!avbd_is_dynamic(r) || (r->flags & RigidBodyFlag::SLEEPING) != RigidBodyFlag::NONE)
  {
    return;
  }

  daxa_f32 dt = sim_config->dt;
  daxa_f32 inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;
  AvbdBodyState* states = (AvbdBodyState*)(AVBD_PC.task_head.avbd_state);
  AvbdBodyState* s = &states[i];

  r->prev_velocity = r->velocity; // for next step's adaptive warm start
  r->prev_omega = r->omega;       // E2: capture step-entry omega BEFORE it is overwritten below, so next
                                  // step's impact classifier can build the exact approach velocity
  r->velocity = (r->position - s->pos_start) * inv_dt;

  // dq = q_cur * conjugate(q_start); small-angle: omega = 2*dq.v/dt (sign-fixed)
  Quaternion qs = Quaternion(s->rot_start.v, s->rot_start.w);
  Quaternion qc = Quaternion(r->rotation.v, r->rotation.w);
  Quaternion qs_conj = Quaternion(-qs.v, qs.w);
  Quaternion dq = qc * qs_conj;
  daxa_f32 sign = dq.w < 0.0f ? -1.0f : 1.0f;
  r->omega = dq.v * (2.0f * sign * inv_dt);

  // DETERMINISM CHECKPOINT (debug): hash the post-main-solve pose here (impact + post-stab have
  // not moved it yet), mirroring the end-of-step gauge in entry_rigid_body_update. Comparing cp vs
  // poshash between two runs localizes whether the divergence enters the main solve or after it.
  // Gated: only runs while measuring determinism (the chain-walk hash below is not free).
  if (sim_config->has_flag(SimFlag::DET_HASHES))
  {
    daxa_u32 cph = asuint(r->position.x) ^ (asuint(r->position.y) * 0x9e3779b9u) ^
                   (asuint(r->position.z) * 0x85ebca6bu) ^ (r->id * 0xc2b2ae35u);
    daxa_u32 crh = asuint(r->rotation.v.x) ^ (asuint(r->rotation.v.y) * 0x9e3779b9u) ^
                   (asuint(r->rotation.v.z) * 0x85ebca6bu) ^ (asuint(r->rotation.w) * 0x27d4eb2fu) ^
                   (r->id * 0xc2b2ae35u);
    daxa_u32 _cprev;
    InterlockedXor(sim_config->dbg_cp_poshash, cph, _cprev);
    InterlockedXor(sim_config->dbg_cp_rothash, crh, _cprev);
    // avbd_state hash (pos_tilde + support_depth) — un-hashed inputs the post-stab reads regardless
    // of the layer filter. Diverges cross-process => an avbd_state field is process-specific.
    daxa_u32 sh = asuint(s->pos_tilde.x) ^ (asuint(s->pos_tilde.y) * 0x9e3779b9u) ^
                  (asuint(s->pos_tilde.z) * 0x85ebca6bu) ^ (s->support_depth * 0x27d4eb2fu) ^ (r->id * 0xc2b2ae35u);
    InterlockedXor(sim_config->dbg_state_hash, sh, _cprev);
    // CHAIN-WALK hash: replicate the primal's chain walk and roll a hash over the RESOLVED partner
    // ids + walk order. The post-stab primal processes contacts via this walk; if collision_map
    // resolution (allocation-order dependent) MISSES a contact in one launch but not the other, the
    // processed SET differs while lh (per-manifold direct) still matches. Diverges => deep-miss is it.
    {
      ManifoldNode* mnodes = (ManifoldNode*)(AVBD_PC.task_head.manifold_nodes);
      CollisionEntry* cmap = (CollisionEntry*)(AVBD_PC.task_head.collision_map);
      Manifold* colls = (Manifold*)(AVBD_PC.task_head.collisions);
      daxa_u32 mnc = min(sim_config->manifold_node_count, BB_MAX_MANIFOLD_NODE_COUNT);
      daxa_u32 ccnt = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
      daxa_u32 wh = r->id + 1u;
      daxa_u32 wnode = r->manifold_node_index;
      daxa_u32 wsteps = 0u;
      while (wnode != MAX_U32 && wsteps < mnc)
      {
        daxa_u32 wpacked = avbd_node_to_packed(wnode, mnc, ccnt, mnodes, cmap);
        daxa_u32 token = (wpacked != MAX_U32) ? (avbd_partner(&colls[wpacked], i) + 1u) : 0xDEADu;
        wh = wh * 0x9e3779b9u + token; // order-sensitive
        daxa_i32 wnx = mnodes[wnode].next;
        wnode = wnx < 0 ? MAX_U32 : daxa_u32(wnx);
        ++wsteps;
      }
      InterlockedXor(sim_config->dbg_state_pad, wh, _cprev);
    }
  }

  // SETTLE ASSIST: a body that is clearly part of a settled pile (3+ manifolds) and
  // already slow gets its residual micro-velocity sponged exponentially, so the crust
  // crosses the sleep threshold (0.12 m/s for 30 steps) in a fraction of a second
  // instead of simmering just above it for minutes while the neighborhood veto holds
  // whole bubbles awake. The 3-manifold gate protects legitimate physics: a ramp
  // slider carries 1-2 manifolds and must keep decelerating by FRICTION alone
  // (scene_4 canonical outcomes are the regression baseline).
  // NOTE: a wider/stronger sponge (0.5 m/s band, x0.85) was measured NET NEGATIVE:
  // draining harder settles bodies "lighter", leaving more contacts hovering at
  // pen ~ 0 whose manifolds flicker and re-excite the crust (full-sleep regressed
  // from sample ~201 to stalls at 419-431/432). These gentler values are the optimum.
  daxa_f32 v2 = dot(r->velocity, r->velocity);
  daxa_f32 w2 = dot(r->omega, r->omega);
  if (v2 < 0.09f && w2 < 0.16f) // (0.3 m/s)^2, (0.4 rad/s)^2
  {
    ManifoldNode* manifold_nodes = (ManifoldNode*)(AVBD_PC.task_head.manifold_nodes);
    daxa_u32 manifold_node_count = min(sim_config->manifold_node_count, BB_MAX_MANIFOLD_NODE_COUNT);
    daxa_u32 node = r->manifold_node_index;
    daxa_u32 cnt = 0u;
    daxa_u32 steps = 0u;
    while (node != MAX_U32 && node < manifold_node_count && cnt < 3u && steps < manifold_node_count)
    {
      ++cnt;
      daxa_i32 nxt = manifold_nodes[node].next;
      node = nxt < 0 ? MAX_U32 : daxa_u32(nxt);
      ++steps;
    }
    if (cnt >= 3u)
    {
      r->velocity *= 0.9f;
      r->omega *= 0.9f;
    }
  }

  // DETERMINISM velocity checkpoint (debug): post-FIN velocity+omega (PRE-impact).
  if (sim_config->has_flag(SimFlag::DET_HASHES))
  {
    daxa_u32 vh = asuint(r->velocity.x) ^ (asuint(r->velocity.y) * 0x9e3779b9u) ^ (asuint(r->velocity.z) * 0x85ebca6bu)
                ^ (asuint(r->omega.x) * 0x27d4eb2fu) ^ (asuint(r->omega.y) * 0x165667b1u) ^ (asuint(r->omega.z) * 0xc2b2ae35u)
                ^ (r->id * 0x9e3779b9u);
    daxa_u32 _vp; InterlockedXor(sim_config->dbg_vh_fin, vh, _vp);
  }
}
