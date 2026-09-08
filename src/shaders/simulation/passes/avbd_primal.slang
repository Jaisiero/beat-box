#include "color_solve_dispatch.slang"
#include "avbd_constraints.slang"

daxa_f32mat3x3 avbd_inverse3(daxa_f32mat3x3 a)
{
  daxa_f32vec3 c0 = daxa_f32vec3(a[0][0], a[1][0], a[2][0]);
  daxa_f32vec3 c1 = daxa_f32vec3(a[0][1], a[1][1], a[2][1]);
  daxa_f32vec3 c2 = daxa_f32vec3(a[0][2], a[1][2], a[2][2]);
  daxa_f32vec3 r0 = cross(c1, c2);
  daxa_f32vec3 r1 = cross(c2, c0);
  daxa_f32vec3 r2 = cross(c0, c1);
  daxa_f32 det = dot(c0, r0);
  daxa_f32 inv_det = abs(det) > 1e-12f ? 1.0f / det : 0.0f;
  daxa_f32mat3x3 inv;
  inv[0][0] = r0.x * inv_det; inv[0][1] = r0.y * inv_det; inv[0][2] = r0.z * inv_det;
  inv[1][0] = r1.x * inv_det; inv[1][1] = r1.y * inv_det; inv[1][2] = r1.z * inv_det;
  inv[2][0] = r2.x * inv_det; inv[2][1] = r2.y * inv_det; inv[2][2] = r2.z * inv_det;
  return inv;
}

void avbd_solve6(inout daxa_f32 H[6][6], inout daxa_f32 b[6], out daxa_f32 x[6])
{
  // LDL^T factorization
  [unroll]
  for (daxa_u32 jj = 0u; jj < 6u; ++jj)
  {
    daxa_f32 d = H[jj][jj];
    [unroll]
    for (daxa_u32 kk = 0u; kk < 6u; ++kk)
    {
      if (kk < jj) { d -= H[jj][kk] * H[jj][kk] * H[kk][kk]; }
    }
    d = max(d, 1e-8f); // regularize
    H[jj][jj] = d;
    [unroll]
    for (daxa_u32 ii = 0u; ii < 6u; ++ii)
    {
      if (ii > jj)
      {
        daxa_f32 s = H[ii][jj];
        [unroll]
        for (daxa_u32 kk = 0u; kk < 6u; ++kk)
        {
          if (kk < jj) { s -= H[ii][kk] * H[jj][kk] * H[kk][kk]; }
        }
        H[ii][jj] = s / d;
      }
    }
  }
  // forward: L y = b
  [unroll]
  for (daxa_u32 ii = 0u; ii < 6u; ++ii)
  {
    daxa_f32 s = b[ii];
    [unroll]
    for (daxa_u32 kk = 0u; kk < 6u; ++kk)
    {
      if (kk < ii) { s -= H[ii][kk] * b[kk]; }
    }
    b[ii] = s;
  }
  // diagonal + backward: D z = y ; L^T x = z
  [unroll]
  for (daxa_u32 ii = 0u; ii < 6u; ++ii)
  {
    b[ii] /= H[ii][ii];
  }
  [unroll]
  for (daxa_i32 ii = 5; ii >= 0; --ii)
  {
    daxa_f32 s = b[ii];
    [unroll]
    for (daxa_u32 kk = 0u; kk < 6u; ++kk)
    {
      if (kk > daxa_u32(ii)) { s -= H[kk][ii] * x[kk]; }
    }
    x[ii] = s;
  }
}

// Call with the two supported modes as literals so Slang can remove main-only
// and post-only constraint work. The host emits alpha=1 for main sweeps and
// alpha=0 for post-stabilization; iteration counts and arithmetic order stay fixed.
void avbd_primal_specialized(daxa_u32vec3 compute_i, daxa_f32 stab_alpha)
{
  SimConfig* sim_config = (SimConfig*)(AVBD_PC.task_head.sim_config);
  if (sim_config->solver_type != SimSolverType::AVBD)
  {
    return;
  }
  // DETERMINISM mode: skip the non-convergent alpha=0 post-stab (the driver's per-launch SPIR-V->ISA
  // codegen perturbs its FP sub-ULP, and with no unique fixed point it can't converge that out). ph
  // collapses to cph (the CONVERGED main solve), which IS bitwise-deterministic. Main sweeps untouched.
  if (stab_alpha < 0.5f && sim_config->has_flag(SimFlag::DETERMINISTIC))
  {
    return;
  }
  daxa_u32 i = compute_i.x;
  if (i >= sim_config->rigid_body_count)
  {
    return;
  }
  daxa_u32* body_color = (daxa_u32*)(AVBD_PC.task_head.body_color);
  if (body_color[i] != AVBD_PC.color)
  {
    return;
  }

  RigidBody* rigid_bodies = (RigidBody*)(AVBD_PC.task_head.rigid_bodies);
  RigidBody* r = &rigid_bodies[i];
  if (!avbd_is_dynamic(r) || (r->flags & RigidBodyFlag::SLEEPING) != RigidBodyFlag::NONE)
  {
    return;
  }

  AvbdBodyState* states = (AvbdBodyState*)(AVBD_PC.task_head.avbd_state);
  AvbdBodyState* s = &states[i];

  // shock propagation (post-stab cascade): each dispatch solves ONE support-depth layer,
  // in ascending order, so this body's supports already settled within this sweep
  if (AVBD_PC.ps_depth != MAX_U32)
  {
    daxa_u32 my_layer = min(s->support_depth, BB_AVBD_SHOCK_LAYERS - 1u);
    if (my_layer != AVBD_PC.ps_depth)
    {
      return;
    }
  }

  // convergence early-out (main sweeps only): when the PREVIOUS iteration's max primal
  // step over ALL bodies fell below the tolerance, the system is at its fixed point and
  // the remaining sweeps would only re-derive it - skip them. Rest converges in a few
  // iterations (the bulk of AVBD's cost at rest); impacts keep the full budget because
  // their steps stay large. asuint on non-negative floats preserves ordering.
  bool main_sweep = stab_alpha > 0.5f && AVBD_PC.ps_depth == MAX_U32;
  daxa_u32 tick = sim_config->avbd_iter_tick;
  if (main_sweep && tick > 0u &&
      sim_config->avbd_step_res[(tick + 1u) & 1u] < asuint(BB_AVBD_CONV_EPS))
  {
    return;
  }

  daxa_f32 dt = sim_config->dt;
  daxa_f32 inv_dt2 = dt > 0.0f ? 1.0f / (dt * dt) : 0.0f;

  // inertia terms: linear m/dt^2, angular I_world/dt^2
  daxa_f32 m_lin = r->mass * inv_dt2;
  daxa_f32mat3x3 inv_inertia_w = r->get_transformed_inertia();
  daxa_f32mat3x3 inertia_w = avbd_inverse3(inv_inertia_w);

  daxa_f32 H[6][6];
  daxa_f32 g[6];
  [unroll]
  for (daxa_u32 a = 0u; a < 6u; ++a)
  {
    g[a] = 0.0f;
    [unroll]
    for (daxa_u32 b = 0u; b < 6u; ++b) { H[a][b] = 0.0f; }
  }
  H[0][0] = m_lin; H[1][1] = m_lin; H[2][2] = m_lin;
  [unroll]
  for (daxa_u32 a = 0u; a < 3u; ++a)
  {
    [unroll]
    for (daxa_u32 b = 0u; b < 3u; ++b) { H[3u + a][3u + b] = inertia_w[a][b] * inv_dt2; }
  }

  // inertia gradient: M/dt^2 * (x - x_tilde)
  daxa_f32vec3 dpos = r->position - s->pos_tilde;
  daxa_f32vec3 dth_t = avbd_small_angle_delta(r->rotation, Quaternion(s->rot_tilde.v, s->rot_tilde.w));
  g[0] = m_lin * dpos.x; g[1] = m_lin * dpos.y; g[2] = m_lin * dpos.z;
  daxa_f32vec3 g_ang = mul(inertia_w, dth_t) * inv_dt2;
  g[3] = g_ang.x; g[4] = g_ang.y; g[5] = g_ang.z;

  // contact energies over this body's manifold list
  ManifoldNode* manifold_nodes = (ManifoldNode*)(AVBD_PC.task_head.manifold_nodes);
  CollisionEntry* collision_map = (CollisionEntry*)(AVBD_PC.task_head.collision_map);
  Manifold* collisions = (Manifold*)(AVBD_PC.task_head.collisions);
  daxa_u32 manifold_node_count = min(sim_config->manifold_node_count, BB_MAX_MANIFOLD_NODE_COUNT);
  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);

  daxa_u32 node = r->manifold_node_index;
  daxa_u32 steps = 0u;
  while (node != MAX_U32 && steps < manifold_node_count)
  {
    daxa_u32 packed = avbd_node_to_packed(node, manifold_node_count, collision_count, manifold_nodes, collision_map);
    if (packed != MAX_U32)
    {
      Manifold* m = &collisions[packed];
      daxa_u32 b1 = m->obb1_index;
      daxa_u32 b2 = m->obb2_index;
      if (b1 < sim_config->rigid_body_count && b2 < sim_config->rigid_body_count && b1 != b2)
      {
        bool i_is_1 = (b1 == i);
        RigidBody* r1 = &rigid_bodies[b1];
        RigidBody* r2 = &rigid_bodies[b2];
        bool dyn1 = avbd_is_dynamic(r1) && (r1->flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE;
        bool dyn2 = avbd_is_dynamic(r2) && (r2->flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE;
        AvbdBodyState* s1 = &states[b1];
        AvbdBodyState* s2 = &states[b2];

        AvbdMotion motion1 = avbd_motion(r1, s1, dyn1);
        AvbdMotion motion2 = avbd_motion(r2, s2, dyn2);

        // NOTE on shock propagation: the cascade ORDERS the layer solves (dispatch
        // order) but every row stays - including partners above. Skipping upper rows
        // was tried TWICE (unordered v1, ordered v2) and both boiled identically: in a
        // penalty solver the post-stab must see the same LOADS as the main sweeps, or
        // it rebuilds an unloaded pile each frame and fights the loaded equilibrium.
        daxa_u32 cc = min((daxa_u32)max(m->contact_count, 0), MAX_CONTACT_POINT_COUNT);
        daxa_f32 mu = sqrt(max(r1->friction, 0.0f) * max(r2->friction, 0.0f));
        for (daxa_u32 j = 0u; j < cc; ++j)
        {
          daxa_f32vec3 rr1, rr2;
          daxa_f32 C = avbd_contact_C(m, j, r1, r2, s1, s2, dyn1, dyn2, stab_alpha, motion1, motion2, rr1, rr2);
          daxa_f32 lam = m->contacts[j].normal_impulse; // <= 0 (force convention)
          daxa_f32 k = m->contacts[j].mass_coefficient;
          // Compare the geometric separation, before margin/slop, with the sleep depth
          // limit. C already has those offsets applied; comparing it directly silently
          // delays extraction by another 5.5 mm and prevents the island from sleeping.
          daxa_f32 initial_C = m->contacts[j].penetration + BB_AVBD_MARGIN;
          daxa_f32 removed_slop = min(max(-initial_C, 0.0f), BB_AVBD_PS_SLOP);
          daxa_f32 geometric_C = C - (BB_AVBD_MARGIN + removed_slop) * (1.0f - stab_alpha);
          if (stab_alpha < 0.5f && geometric_C < -BB_AVBD_DEEP_EXTRACT_THRESH)
          {
            k = max(k, BB_AVBD_DEEP_EXTRACT_K);
          }
          // clamped force magnitude (reference Sec 3.2): contacts only push -> f in (-inf, 0]
          daxa_f32 f = min(k * C + lam, 0.0f);
          // my jacobian (J1 = [-n, -(r1 x n)], J2 = [+n, +(r2 x n)])
          daxa_f32vec3 n = m->normal;
          daxa_f32vec3 Jl = i_is_1 ? -n : n;
          daxa_f32vec3 Ja = i_is_1 ? -cross(rr1, n) : cross(rr2, n);
          // gradient += J * f (Eq. 13); hessian += k * J J^T (Eq. 17, contacts have no
          // second-order term: truncated Taylor, reference Sec 4)
          g[0] += f * Jl.x; g[1] += f * Jl.y; g[2] += f * Jl.z;
          g[3] += f * Ja.x; g[4] += f * Ja.y; g[5] += f * Ja.z;
          daxa_f32 J[6] = { Jl.x, Jl.y, Jl.z, Ja.x, Ja.y, Ja.z };
          [unroll]
          for (daxa_u32 a = 0u; a < 6u; ++a)
          {
            [unroll]
            for (daxa_u32 b = 0u; b < 6u; ++b) { H[a][b] += k * J[a] * J[b]; }
          }
          // friction rows: CONE-bounded tangent forces (reference avbd-demo3d manifold.cpp):
          // clamp the JOINT 2D magnitude of (ft0,ft1) to mu*|lambda_n|, not each tangent to a
          // box. Box clamping lets the friction force wander the square's corners as the two
          // tangent directions saturate independently -> the contact torque jitters -> the
          // user's "nervous rotational jitter". The cone keeps the friction force on a disc
          // of radius mu*|F_n| -> rotationally stable, matching the official solver.
          // friction bound uses the ACTUAL normal force f = min(k*C + lam, 0), not just the
          // accumulated impulse lam. At a converged rest contact f == lam (identical), but a
          // body held up mostly by the penalty term (k*C) while its warm-started lam has decayed
          // to ~0 - e.g. a light fracture fragment whose floor contact re-keys as it slides -
          // otherwise gets bound = mu*0 = 0 and slides frictionlessly forever. Bounding by |f|
          // ties friction to the real support force, killing that frictionless drift.
          daxa_f32 bound = mu * abs(f);
          daxa_f32vec3 td0 = m->tangent_vectors[0];
          daxa_f32vec3 td1 = m->tangent_vectors[1];
          daxa_f32vec3 ar1_0, ar2_0, ar1_1, ar2_1;
          daxa_f32 C0t0, C0t1;
          daxa_f32 Ct0 = avbd_dir_C(td0, m, j, r1, r2, s1, s2, dyn1, dyn2, stab_alpha, motion1, motion2, ar1_0, ar2_0, C0t0);
          // The second tangent uses exactly the same step-start anchor arms.
          ar1_1 = ar1_0; ar2_1 = ar2_0;
          daxa_f32 Ct1 = avbd_dir_C_from_arms(td1,
              dyn1 ? s1->pos_start : r1->position, dyn2 ? s2->pos_start : r2->position,
              ar1_1, ar2_1, dyn1, dyn2, stab_alpha, motion1, motion2, C0t1);
          daxa_f32 ft0 = m->contacts[j].tangent_mass[0] * Ct0 + m->contacts[j].tangent_impulse[0];
          daxa_f32 ft1 = m->contacts[j].tangent_mass[1] * Ct1 + m->contacts[j].tangent_impulse[1];
          daxa_f32 fmag = sqrt(ft0 * ft0 + ft1 * ft1);
          if (fmag > bound && fmag > 1.0e-9f) { daxa_f32 sc = bound / fmag; ft0 *= sc; ft1 *= sc; }
          [unroll]
          for (daxa_u32 t = 0u; t < 2u; ++t)
          {
            daxa_f32vec3 td = t == 0u ? td0 : td1;
            daxa_f32vec3 ar1 = t == 0u ? ar1_0 : ar1_1;
            daxa_f32vec3 ar2 = t == 0u ? ar2_0 : ar2_1;
            daxa_f32 ft = t == 0u ? ft0 : ft1;
            daxa_f32 kt = m->contacts[j].tangent_mass[t];
            daxa_f32vec3 Jlt = i_is_1 ? -td : td;
            daxa_f32vec3 Jat = i_is_1 ? -cross(ar1, td) : cross(ar2, td);
            g[0] += ft * Jlt.x; g[1] += ft * Jlt.y; g[2] += ft * Jlt.z;
            g[3] += ft * Jat.x; g[4] += ft * Jat.y; g[5] += ft * Jat.z;
            daxa_f32 Jt[6] = { Jlt.x, Jlt.y, Jlt.z, Jat.x, Jat.y, Jat.z };
            [unroll]
            for (daxa_u32 a = 0u; a < 6u; ++a)
            {
              [unroll]
              for (daxa_u32 b = 0u; b < 6u; ++b) { H[a][b] += kt * Jt[a] * Jt[b]; }
            }
          }
        }
      }
    }
    daxa_i32 next = manifold_nodes[node].next;
    node = next < 0 ? MAX_U32 : daxa_u32(next);
    ++steps;
  }

  // Newton step: H dx = -g
  daxa_f32 rhs[6];
  [unroll]
  for (daxa_u32 a = 0u; a < 6u; ++a) { rhs[a] = -g[a]; }
  daxa_f32 dx[6];
  avbd_solve6(H, rhs, dx);

  // no step limiting: the reference scheme relies on the inertia term + penalty curvature.
  // relax < 1 damps the EXTRA post-stab sweeps (full-strength alpha=0 GS diverges past 4).
  daxa_f32vec3 dlin = daxa_f32vec3(dx[0], dx[1], dx[2]) * AVBD_PC.relax;
  daxa_f32vec3 dang = daxa_f32vec3(dx[3], dx[4], dx[5]) * AVBD_PC.relax;

  r->position += dlin;
  daxa_f32vec3 half = dang * 0.5f;
  Quaternion dq = Quaternion(half, 1.0f);
  Quaternion qn = (dq * r->rotation).normalize();
  r->rotation.v = qn.v;
  r->rotation.w = qn.w;

  // convergence residual: this iteration's max step magnitude (linear + angular)
  if (main_sweep)
  {
    daxa_f32 mag = length(dlin) + length(dang);
    daxa_u32 prev;
    InterlockedMax(sim_config->avbd_step_res[tick & 1u], asuint(mag), prev);
  }
}

[shader("compute")]
[numthreads(COLOR_SOLVE_COMPUTE_X, 1, 1)] void entry_avbd_primal(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  if (AVBD_PC.stab_alpha > 0.5f) avbd_primal_specialized(compute_i, 1.0f);
  else avbd_primal_specialized(compute_i, 0.0f);
}
