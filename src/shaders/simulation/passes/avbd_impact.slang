#include "avbd_common.slang"
#include "fracture.slang"

bool avbd_impact_pair_gates(SimConfig* sim_config, Manifold* m, out daxa_u32 b1, out daxa_u32 b2)
{
  b1 = m->obb1_index;
  b2 = m->obb2_index;
  return !(b1 >= sim_config->rigid_body_count || b2 >= sim_config->rigid_body_count || b1 == b2);
}

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_impact_j(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(AVBD_PC.task_head.sim_config);
  if (sim_config->solver_type != SimSolverType::AVBD)
  {
    return;
  }
  daxa_u32 m_idx = compute_i.x;
  daxa_u32 coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (m_idx >= coll)
  {
    return;
  }
  Manifold* m = &((Manifold*)(AVBD_PC.task_head.collisions))[m_idx];

  // DETERMINISM (debug): hash this manifold's final main-solve contact lambda/k (post-dual, pre
  // post-stab). Keyed by body indices => order-independent + deterministic. If dbg_color_pad
  // diverges while cph (pose) matches, the per-contact LAMBDA/K is the non-deterministic carrier
  // the post-stab amplifies (the pose converges; lambda need not).
  if (sim_config->has_flag(SimFlag::DET_HASHES))
  {
    daxa_u32 hcc = min((daxa_u32)max(m->contact_count, 0), MAX_CONTACT_POINT_COUNT);
    daxa_u32 lh = (m->obb1_index * 0x9e3779b9u) ^ (m->obb2_index * 0x85ebca6bu);
    lh ^= asuint(m->normal.x) ^ (asuint(m->normal.y) * 0x27d4eb2fu) ^ (asuint(m->normal.z) * 0x165667b1u);
    lh ^= asuint(m->tangent_vectors[0].x) ^ (asuint(m->tangent_vectors[0].y) * 0x9e3779b9u) ^ (asuint(m->tangent_vectors[1].z) * 0x85ebca6bu);
    for (daxa_u32 hj = 0u; hj < hcc; ++hj)
    {
      lh ^= asuint(m->contacts[hj].normal_impulse) * 0x27d4eb2fu;
      lh ^= asuint(m->contacts[hj].mass_coefficient) * 0x165667b1u;
      lh ^= asuint(m->contacts[hj].position.x) ^ (asuint(m->contacts[hj].position.y) * 0x9e3779b9u) ^ (asuint(m->contacts[hj].position.z) * 0x85ebca6bu);
      lh ^= asuint(m->contacts[hj].penetration) * 0xc2b2ae35u;
      lh ^= asuint(m->contacts[hj].anchor_a.x) ^ (asuint(m->contacts[hj].anchor_a.y) * 0x27d4eb2fu) ^ (asuint(m->contacts[hj].anchor_a.z) * 0x165667b1u);
      lh ^= asuint(m->contacts[hj].anchor_b.x) ^ (asuint(m->contacts[hj].anchor_b.y) * 0x9e3779b9u) ^ (asuint(m->contacts[hj].anchor_b.z) * 0x85ebca6bu);
      lh ^= m->contacts[hj].stick * 0x27d4eb2fu;
      lh ^= asuint(m->contacts[hj].tangent_impulse[0]) ^ (asuint(m->contacts[hj].tangent_impulse[1]) * 0xc2b2ae35u);
      // the post-stab primal reads tangent_mass (friction rows) — NOT hashed before; prime uninit suspect
      lh ^= asuint(m->contacts[hj].tangent_mass[0]) * 0x9e3779b9u ^ (asuint(m->contacts[hj].tangent_mass[1]) * 0x85ebca6bu);
      lh ^= asuint(m->contacts[hj].normal_mass) * 0x27d4eb2fu;
      lh ^= asuint(m->contacts[hj].bias_factor) ^ (asuint(m->contacts[hj].impulse_coefficient) * 0x165667b1u);
    }
    daxa_u32 _lp;
    InterlockedXor(sim_config->dbg_color_pad, lh, _lp);
  }

  daxa_u32 b1, b2;
  if (!avbd_impact_pair_gates(sim_config, m, b1, b2))
  {
    return;
  }
  RigidBody* rigid_bodies = (RigidBody*)(AVBD_PC.task_head.rigid_bodies);
  RigidBody* r1 = &rigid_bodies[b1];
  RigidBody* r2 = &rigid_bodies[b2];
  bool dyn1 = avbd_is_dynamic(r1) && (r1->flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE;
  bool dyn2 = avbd_is_dynamic(r2) && (r2->flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE;
  daxa_u32 cc = min((daxa_u32)max(m->contact_count, 0), MAX_CONTACT_POINT_COUNT);
  if (!dyn1 && !dyn2)
  {
    for (daxa_u32 j = 0u; j < cc; ++j) { m->contacts[j].normal_mass = 0.0f; }
    return;
  }

  daxa_f32vec3 n = m->normal; // points from body1 toward body2 (SAT convention)
  daxa_f32mat3x3 inv_i1 = r1->get_transformed_inertia();
  daxa_f32mat3x3 inv_i2 = r2->get_transformed_inertia();
  daxa_f32 inv_m1 = dyn1 ? r1->inv_mass : 0.0f; // sleeping/static side acts as an anchor
  daxa_f32 inv_m2 = dyn2 ? r2->inv_mass : 0.0f;

  // SEQUENTIAL row relaxation (this kernel is single-thread per manifold): each row's
  // impulse is computed against velocities that already include the previous rows'
  // impulses. Computing every row from the SAME pre-apply velocities over-corrects by up
  // to the row count when contacts coincide (a corner catch emits 2-4 near-coincident
  // rows), which REFLECTED the full approach speed (~11 m/s straight back up, e=0) once
  // the approach side started firing. IMP_APPLY still applies the stored impulses to the
  // real bodies; these locals only order the solve.
  daxa_f32vec3 lv1 = r1->velocity, lw1 = r1->omega;
  daxa_f32vec3 lv2 = r2->velocity, lw2 = r2->omega;
  // FRACTURE classifier: strongest fired-row impulse + its contact point, per manifold
  daxa_f32 frac_imp = 0.0f;
  daxa_f32vec3 frac_p = daxa_f32vec3(0.0f);

  for (daxa_u32 j = 0u; j < cc; ++j)
  {
    m->contacts[j].normal_mass = 0.0f; // scratch must be defined for IMP_APPLY either way
    daxa_f32vec3 p = m->contacts[j].position;
    daxa_f32vec3 ra = p - r1->position;
    daxa_f32vec3 rb = p - r2->position;

    // entry (pre-step) approach velocity: prev_velocity + prev_omega both hold the step-entry values
    // (saved by FIN before they were overwritten), so vrel_pre is exact even for a spinning body
    // landing on a corner (where omega x r dominates the contact velocity) — E2.
    daxa_f32vec3 vrel_pre = (r2->prev_velocity + cross(r2->prev_omega, rb)) - (r1->prev_velocity + cross(r1->prev_omega, ra));
    daxa_f32 vn_pre = dot(n, vrel_pre);
    // approach threshold = ~one step's worth of gravity-induced normal velocity. Use the actual sim
    // gravity (sim_config->gravity, negative) instead of a hardcoded -9.8 so non-default-gravity
    // scenes classify resting/sliding vs real approaching contacts correctly. (2*gravity*dt is
    // negative, matching the original -2*9.8*dt; identical at the default g up to 9.8-vs-9.81.)
    if (vn_pre > 2.0f * sim_config->gravity * sim_config->dt) // not a real approach: resting/sliding contact
    {
      continue;
    }
    // post-step (reconstructed) relative normal velocity, INCLUDING prior rows' impulses
    daxa_f32vec3 vrel = (lv2 + cross(lw2, rb)) - (lv1 + cross(lw1, ra));
    daxa_f32 vn = dot(n, vrel);
    // E1: restitution. The target post-impact SEPARATING velocity is e*approach = -e*vn_pre
    // (vn_pre < 0 for a real approach, gated above), so e=0 is fully inelastic (target 0). The
    // per-pair e is max(r1,r2), clamped to [0,1] (>1 would inject energy). The impulse drives vn to
    // the target: it TRIMS an over-shoot (dvn>0) and, for e>0, INJECTS the bounce when the solver
    // under-produced it (dvn<0). For e==0 this reduces EXACTLY to the old code (target 0 -> the gate
    // becomes `vn <= 1e-4` and the impulse -vn/K), so every restitution=0 body/scene is byte-identical.
    daxa_f32 e = clamp(max(r1->restitution, r2->restitution), 0.0f, 1.0f);
    daxa_f32 target = -e * vn_pre; // >= 0
    daxa_f32 dvn = vn - target;
    // effective mass along n (hoisted above the fire gate: the FRACTURE classifier needs
    // it for every approaching row, fired or not)
    daxa_f32vec3 t1 = dyn1 ? cross(mul(inv_i1, cross(ra, n)), ra) : daxa_f32vec3(0.0f);
    daxa_f32vec3 t2 = dyn2 ? cross(mul(inv_i2, cross(rb, n)), rb) : daxa_f32vec3(0.0f);
    daxa_f32 K = inv_m1 + inv_m2 + dot(n, t1 + t2);
    if (K <= 1.0e-9f)
    {
      continue;
    }
    // FRACTURE classifier: the impulse a full inelastic stop of the ENTRY approach would
    // take (-vn_pre/K). NOT the fired trim impulse: for voxel bodies the positional solve
    // (deep-extract ramp) often absorbs the stop, leaving the velocity trim ~nothing to
    // remove (measured: a 12 m/s slam read 0.005 through the trim) - the material doesn't
    // care WHO absorbed the momentum.
    daxa_f32 stop_imp = -vn_pre / K; // > 0 (vn_pre < 0 past the approach gate)
    if (stop_imp > frac_imp) { frac_imp = stop_imp; frac_p = p; }
    // fire on BOTH sides of the target (hard-impact "teleport" fix). The old e=0 rule only
    // trimmed leftover SEPARATION (dvn > 0) - but on the impact frame itself the alpha-blind
    // main solve barely resolves the overlap, so vn is still APPROACHING and the row never
    // fired: the un-consumed momentum then lived on as the dual's lambda spike (stopping
    // ~10 m/s builds lambda ~ -150k in one frame), which the post-stab's force law
    // min(k*C+lam, 0) discharges POSITIONALLY to its equilibrium C = -lam/k ~ +0.9 m -> the
    // one-frame floor-impact blast. Removing the remaining approach velocity here makes the
    // touchdown perfectly inelastic AT THE CONTACT INSTANT (a corner catch stops dead, then
    // topples naturally); resting/sliding rows never reach this code (approach gate ~2*g*dt).
    bool fire = abs(dvn) > 1.0e-4f;
    if (!fire) // already at the target post-impact velocity
    {
      continue;
    }
    daxa_f32 p_imp = -dvn / K; // the impulse that drives vn -> target
    m->contacts[j].normal_mass = p_imp; // trims over-rebound / leftover approach, injects e>0 bounce
    // relax the local velocities so the NEXT row sees this row's impulse (see header note)
    daxa_f32vec3 imp = p_imp * n;
    if (dyn1) { lv1 -= inv_m1 * imp; lw1 -= mul(inv_i1, cross(ra, imp)); }
    if (dyn2) { lv2 += inv_m2 * imp; lw2 += mul(inv_i2, cross(rb, imp)); }
    // and RESET this row's dual: a fired row is an IMPACT, not a load - its lambda is the
    // one-step stopping force (a momentum spike), not the steady contact force the warm
    // start should carry. Kept, it re-discharges as the positional blast above (or, gated,
    // as a next-frame velocity bounce through the main sweeps - both falsified live). The
    // ramp rebuilds within a frame exactly as for any fresh contact; loads never fire.
    m->contacts[j].normal_impulse = 0.0f;
  }

  // Share material/event handling with TGS; AVBD retains its pre-impact classifier.
  emit_fracture_pair(sim_config, r1, r2, frac_imp, frac_p, n);
}

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_impact_apply(daxa_u32vec3 compute_i : SV_DispatchThreadID)
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
  ManifoldNode* nodes = (ManifoldNode*)(AVBD_PC.task_head.manifold_nodes);
  CollisionEntry* cmap = (CollisionEntry*)(AVBD_PC.task_head.collision_map);
  Manifold* colls = (Manifold*)(AVBD_PC.task_head.collisions);
  daxa_u32 node_count = min(sim_config->manifold_node_count, BB_MAX_MANIFOLD_NODE_COUNT);
  daxa_u32 coll_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);

  daxa_f32mat3x3 inv_i = r->get_transformed_inertia();
  daxa_f32 inv_m = r->inv_mass;
  daxa_f32vec3 dv = daxa_f32vec3(0.0f);
  daxa_f32vec3 dw = daxa_f32vec3(0.0f);

  daxa_u32 current = r->manifold_node_index;
  daxa_u32 steps = 0u;
  while (current != MAX_U32 && current < node_count && steps < node_count)
  {
    daxa_u32 packed = avbd_node_to_packed(current, node_count, coll_count, nodes, cmap);
    if (packed != MAX_U32)
    {
      Manifold* m = &colls[packed];
      daxa_u32 b1, b2;
      if (avbd_impact_pair_gates(sim_config, m, b1, b2) && (b1 == i || b2 == i))
      {
        daxa_f32 side = (b2 == i) ? 1.0f : -1.0f; // impulse +p*n on body2, -p*n on body1
        daxa_f32vec3 n = m->normal;
        daxa_u32 cc = min((daxa_u32)max(m->contact_count, 0), MAX_CONTACT_POINT_COUNT);
        for (daxa_u32 j = 0u; j < cc; ++j)
        {
          daxa_f32 p_imp = m->contacts[j].normal_mass;
          if (p_imp == 0.0f)
          {
            continue;
          }
          daxa_f32vec3 arm = m->contacts[j].position - r->position;
          dv += n * (side * p_imp * inv_m);
          dw += mul(inv_i, cross(arm, n)) * (side * p_imp);
        }
      }
    }
    daxa_i32 nx = nodes[current].next;
    current = nx < 0 ? MAX_U32 : daxa_u32(nx);
    ++steps;
  }
  r->velocity += dv;
  r->omega += dw;
  // DETERMINISM velocity checkpoint (debug): post-impact velocity+omega.
  if (sim_config->has_flag(SimFlag::DET_HASHES))
  {
    daxa_u32 vh = asuint(r->velocity.x) ^ (asuint(r->velocity.y) * 0x9e3779b9u) ^ (asuint(r->velocity.z) * 0x85ebca6bu)
                ^ (asuint(r->omega.x) * 0x27d4eb2fu) ^ (asuint(r->omega.y) * 0x165667b1u) ^ (asuint(r->omega.z) * 0xc2b2ae35u)
                ^ (r->id * 0x9e3779b9u);
    daxa_u32 _vp; InterlockedXor(sim_config->dbg_vh_imp, vh, _vp);
  }
}
