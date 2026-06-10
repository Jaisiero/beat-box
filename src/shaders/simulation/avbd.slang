// AVBD (Augmented Vertex Block Descent, Giles/Diaz/Yuksel SIGGRAPH 2025) passes.
//
// Phase F1 (this file so far): Jones-Plassmann vertex coloring of the BODY graph
// (nodes = dynamic bodies, edges = contacts). Unlike the contact (edge) coloring used by the
// PGS per-color solver — which needs >= max-degree colors and degenerates on hub bodies — the
// chromatic number here is bounded by the local geometry (~10-20 for piles), independent of how
// many contacts a single body has: the hub occupies one color alone and solves all its contacts
// inside its own block update.
//
// Adjacency: the narrow phase links every manifold into both bodies' linked lists
// (RigidBody::manifold_node_index -> manifold_nodes). Manifold ids in those lists are SCRATCH
// ids; collision_map translates them to the packed solver buffer (collisions).

#include "extensions.slang"

[[vk::push_constant]] AvbdPushConstants AVBD_PC;

static const daxa_u32 AVBD_UNCOLORED = MAX_U32;

// ----- adjacency walk helpers -----

// Translate a body's manifold-list node into the packed manifold, or nullptr-like sentinel.
// Returns MAX_U32 when the node/manifold is out of bounds.
daxa_u32 avbd_node_to_packed(daxa_u32 node, daxa_u32 manifold_node_count, daxa_u32 collision_count,
                             ManifoldNode* manifold_nodes, CollisionEntry* collision_map)
{
  if (node >= manifold_node_count)
  {
    return MAX_U32;
  }
  daxa_u32 scratch_id = manifold_nodes[node].manifold_id;
  if (scratch_id >= collision_count)
  {
    return MAX_U32;
  }
  daxa_u32 packed = collision_map[scratch_id].index;
  return (packed < collision_count) ? packed : MAX_U32;
}

daxa_u32 avbd_partner(Manifold* m, daxa_u32 me)
{
  return (m->obb1_index == me) ? m->obb2_index : m->obb1_index;
}

bool avbd_is_dynamic(RigidBody* r)
{
  return (r->flags & RigidBodyFlag::DYNAMIC) != RigidBodyFlag::NONE;
}

// unique per-round priority: rotating hash in the high bits, body index in the low bits (no ties)
daxa_u32 avbd_priority(daxa_u32 body, daxa_u32 round)
{
  daxa_u32 x = body * 0x9e3779b9u + round * 0x85ebca6bu;
  x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16;
  return ((x & 0xFFFu) << 20) | (body & 0xFFFFFu);
}

// ----- step setup / teardown (F2) -----

// Saves the step-start pose, computes the inertial (free-fall) target, and INITIALIZES the
// primal state at that target: with zero primal iterations bodies free-fall exactly, and the
// descent pulls them back from x_tilde toward constraint satisfaction.
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
    // TEMP DIAG (zeroed here, accumulated by primal/dual dispatches later this step)
    sim_config->avbd_dbg_max_dx = 0u;
    sim_config->avbd_dbg_max_lam = 0u;
    sim_config->avbd_dbg_active = 0u;
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

  s->pos_start = r->position;
  s->rot_start.v = r->rotation.v;
  s->rot_start.w = r->rotation.w;

  daxa_f32vec3 v = r->velocity;
  if ((r->flags & RigidBodyFlag::GRAVITY) != RigidBodyFlag::NONE)
  {
    v.y += sim_config->gravity * dt; // gravity folded into the inertial velocity
  }
  s->pos_tilde = r->position + v * dt;
  Quaternion q_tilde = r->apply_angular_velocity(r->rotation, r->omega, dt);
  s->rot_tilde.v = q_tilde.v;
  s->rot_tilde.w = q_tilde.w;

  // start the primal state at the inertial target
  r->position = s->pos_tilde;
  r->rotation.v = q_tilde.v;
  r->rotation.w = q_tilde.w;
  r->velocity = v; // keep the gravity-updated velocity for diagnostics/sleep thresholds mid-step
}

// Reconstructs velocities from the pose delta (BDF1): v = (x - x_start)/dt,
// omega from the small-angle quaternion delta.
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

  r->velocity = (r->position - s->pos_start) * inv_dt;

  // dq = q_cur * conjugate(q_start); small-angle: omega = 2*dq.v/dt (sign-fixed)
  Quaternion qs = Quaternion(s->rot_start.v, s->rot_start.w);
  Quaternion qc = Quaternion(r->rotation.v, r->rotation.w);
  Quaternion qs_conj = Quaternion(-qs.v, qs.w);
  Quaternion dq = qc * qs_conj;
  daxa_f32 sign = dq.w < 0.0f ? -1.0f : 1.0f;
  r->omega = dq.v * (2.0f * sign * inv_dt);
}

// ----- F3: contact constraints (normal only), primal 6x6 block solve + dual updates -----
//
// Constraint per contact j (bodies 1=obb1, 2=obb2, normal n points 1->2, penetration<0=overlap):
//   C(x) = pen_start + n.((dp2 + dth2 x r2) - (dp1 + dth1 x r1))   [>= 0 required]
// with dp/dth the pose deltas since step start and r_i the world contact arms at step start.
// Augmented Lagrangian energy: E = -lambda*C + (k/2)*min(C,0)^2, lambda >= 0.
// Storage reuse: Contact.normal_impulse = lambda_n, Contact.mass_coefficient = penalty k_n.

daxa_f32vec3 avbd_small_angle_delta(Quaternion q_now, Quaternion q_start)
{
  Quaternion qs_conj = Quaternion(-q_start.v, q_start.w);
  Quaternion dq = q_now * qs_conj;
  daxa_f32 sign = dq.w < 0.0f ? -1.0f : 1.0f;
  return dq.v * (2.0f * sign);
}

// current constraint value of contact j of manifold m (also outputs the world arms)
daxa_f32 avbd_contact_C(Manifold* m, daxa_u32 j, RigidBody* r1, RigidBody* r2,
                        AvbdBodyState* s1, AvbdBodyState* s2,
                        bool dyn1, bool dyn2,
                        out daxa_f32vec3 rr1, out daxa_f32vec3 rr2)
{
  daxa_f32vec3 n = m->normal;
  daxa_f32vec3 p = m->contacts[j].position;
  // world arms relative to the step-start centers (statics: current center == start center)
  daxa_f32vec3 c1 = dyn1 ? s1->pos_start : r1->position;
  daxa_f32vec3 c2 = dyn2 ? s2->pos_start : r2->position;
  rr1 = p - c1;
  rr2 = p - c2;

  daxa_f32 C = m->contacts[j].penetration;
  if (dyn1)
  {
    daxa_f32vec3 dp1 = r1->position - s1->pos_start;
    daxa_f32vec3 dth1 = avbd_small_angle_delta(r1->rotation, s1->rot_start);
    C -= dot(n, dp1 + cross(dth1, rr1));
  }
  if (dyn2)
  {
    daxa_f32vec3 dp2 = r2->position - s2->pos_start;
    daxa_f32vec3 dth2 = avbd_small_angle_delta(r2->rotation, s2->rot_start);
    C += dot(n, dp2 + cross(dth2, rr2));
  }
  return C;
}

// 3x3 inverse (closed form, for the world inertia from the stored world inv-inertia)
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

// solve H * x = b for symmetric positive-definite 6x6 (in-place LDLT, diagonal regularized)
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

// per-manifold warm start: scale persisted lambda/k (paper: alpha*gamma / gamma) or reset fresh
[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_warmstart(daxa_u32vec3 compute_i : SV_DispatchThreadID)
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
  daxa_u32 cc = min((daxa_u32)max(m->contact_count, 0), MAX_CONTACT_POINT_COUNT);
  for (daxa_u32 j = 0u; j < cc; ++j)
  {
    bool matched = m->contacts[j].warm_start != 0u && sim_config->has_flag(SimFlag::WARM_STARTING);
    daxa_f32 lam = matched ? m->contacts[j].normal_impulse : 0.0f;
    daxa_f32 k = matched ? m->contacts[j].mass_coefficient : 0.0f;
    m->contacts[j].normal_impulse = max(lam, 0.0f) * BB_AVBD_ALPHA * BB_AVBD_GAMMA;
    m->contacts[j].mass_coefficient = clamp(k * BB_AVBD_GAMMA, BB_AVBD_K_MIN, BB_AVBD_K_MAX);
    // Position-correction limiting, FIXED once per step (paper's momentum-spike mitigation):
    // shift the constraint so this step only asks to correct BB_AVBD_MAX_DEPEN of the
    // step-start penetration. Re-evaluating the limit per iteration would move the goalpost
    // every primal sweep and make lambda climb without bound. bias_factor is a PGS-only
    // field, reused as the per-step shift under AVBD.
    daxa_f32 depth0 = max(0.0f, -m->contacts[j].penetration);
    m->contacts[j].bias_factor = max(0.0f, depth0 - BB_AVBD_MAX_DEPEN);
  }
}

// primal: one thread per body of the push-constant color; minimize inertia + contact energies
// over the body's own 6 DoF (Gauss-Newton step on a 6x6 system; neighbors of other colors are
// frozen this dispatch, which is what the body coloring guarantees)
[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_primal(daxa_u32vec3 compute_i : SV_DispatchThreadID)
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

        daxa_u32 cc = min((daxa_u32)max(m->contact_count, 0), MAX_CONTACT_POINT_COUNT);
        for (daxa_u32 j = 0u; j < cc; ++j)
        {
          daxa_f32vec3 rr1, rr2;
          daxa_f32 C = avbd_contact_C(m, j, r1, r2, s1, s2, dyn1, dyn2, rr1, rr2)
                       + m->contacts[j].bias_factor; // per-step correction limit (see warmstart)
          daxa_f32 lam = m->contacts[j].normal_impulse;
          daxa_f32 k = m->contacts[j].mass_coefficient;
          // Rockafellar augmented Lagrangian for the inequality C >= 0:
          // F = max(0, lambda - k*C). Crucially F cuts to ZERO once separated past C > lambda/k —
          // a plain "-lambda" force would keep ATTRACTING separated bodies (glue) and pump energy.
          daxa_f32 F = max(0.0f, lam - k * C);
          if (F <= 0.0f)
          {
            continue; // inactive
          }
          // my jacobian (J1 = [-n, -(r1 x n)], J2 = [+n, +(r2 x n)])
          daxa_f32vec3 n = m->normal;
          daxa_f32vec3 Jl = i_is_1 ? -n : n;
          daxa_f32vec3 Ja = i_is_1 ? -cross(rr1, n) : cross(rr2, n);
          daxa_f32 force = -F; // dE/dC
          // gradient += force * J
          g[0] += force * Jl.x; g[1] += force * Jl.y; g[2] += force * Jl.z;
          g[3] += force * Ja.x; g[4] += force * Ja.y; g[5] += force * Ja.z;
          // hessian += k * J J^T (Gauss-Newton penalty curvature, active region only)
          daxa_f32 J[6] = { Jl.x, Jl.y, Jl.z, Ja.x, Ja.y, Ja.z };
          [unroll]
          for (daxa_u32 a = 0u; a < 6u; ++a)
          {
            [unroll]
            for (daxa_u32 b = 0u; b < 6u; ++b) { H[a][b] += k * J[a] * J[b]; }
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

  // step limiting (prototype anti-explosion guard)
  daxa_f32vec3 dlin = daxa_f32vec3(dx[0], dx[1], dx[2]);
  daxa_f32vec3 dang = daxa_f32vec3(dx[3], dx[4], dx[5]);
  daxa_f32 lin_len = length(dlin);
  if (lin_len > 0.5f) { dlin *= 0.5f / lin_len; }
  daxa_f32 ang_len = length(dang);
  if (ang_len > 0.5f) { dang *= 0.5f / ang_len; }
  { // TEMP DIAG
    daxa_u32 prevd;
    InterlockedMax(sim_config->avbd_dbg_max_dx, asuint(min(lin_len, 0.5f)), prevd);
  }

  r->position += dlin;
  daxa_f32vec3 half = dang * 0.5f;
  Quaternion dq = Quaternion(half, 1.0f);
  Quaternion qn = (dq * r->rotation).normalize();
  r->rotation.v = qn.v;
  r->rotation.w = qn.w;
}

// dual: one thread per manifold; ascend lambda, grow the penalty while still violated
[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_dual(daxa_u32vec3 compute_i : SV_DispatchThreadID)
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
  daxa_u32 b1 = m->obb1_index;
  daxa_u32 b2 = m->obb2_index;
  if (b1 >= sim_config->rigid_body_count || b2 >= sim_config->rigid_body_count || b1 == b2)
  {
    return;
  }

  RigidBody* rigid_bodies = (RigidBody*)(AVBD_PC.task_head.rigid_bodies);
  AvbdBodyState* states = (AvbdBodyState*)(AVBD_PC.task_head.avbd_state);
  RigidBody* r1 = &rigid_bodies[b1];
  RigidBody* r2 = &rigid_bodies[b2];
  bool dyn1 = avbd_is_dynamic(r1) && (r1->flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE;
  bool dyn2 = avbd_is_dynamic(r2) && (r2->flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE;
  if (!dyn1 && !dyn2)
  {
    return; // fully at rest / static pair
  }

  daxa_u32 cc = min((daxa_u32)max(m->contact_count, 0), MAX_CONTACT_POINT_COUNT);
  for (daxa_u32 j = 0u; j < cc; ++j)
  {
    daxa_f32vec3 rr1, rr2;
    daxa_f32 C = avbd_contact_C(m, j, r1, r2, &states[b1], &states[b2], dyn1, dyn2, rr1, rr2)
                 + m->contacts[j].bias_factor; // same shifted constraint the primal minimizes
    daxa_f32 lam = m->contacts[j].normal_impulse;
    daxa_f32 k = m->contacts[j].mass_coefficient;
    daxa_f32 new_lam = max(0.0f, lam - k * C);
    m->contacts[j].normal_impulse = new_lam;
    if (C < 0.0f)
    {
      m->contacts[j].mass_coefficient = min(BB_AVBD_K_MAX, k * BB_AVBD_BETA);
    }
    { // TEMP DIAG
      daxa_u32 prevd;
      InterlockedMax(sim_config->avbd_dbg_max_lam, asuint(new_lam), prevd);
      if (new_lam > 0.0f)
      {
        InterlockedAdd(sim_config->avbd_dbg_active, 1u, prevd);
      }
    }
  }
}

// ----- coloring passes -----

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_color_reset(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(AVBD_PC.task_head.sim_config);
  daxa_u32 i = compute_i.x;
  if (i == 0u)
  {
    sim_config->avbd_color_count = 0u;
    sim_config->avbd_violations = 0u;
  }
  if (i >= sim_config->rigid_body_count)
  {
    return;
  }
  daxa_u32* body_color = (daxa_u32*)(AVBD_PC.task_head.body_color);
  body_color[i] = AVBD_UNCOLORED;
}

// One Jones-Plassmann round (push constant = round number): an uncolored body colors itself when
// it holds the highest priority among its UNCOLORED dynamic neighbors; it picks the smallest
// color unused by its COLORED neighbors. Two adjacent bodies can never both be local maxima, so
// no two neighbors commit the same color in the same round without arbitration buffers.
[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_color_round(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(AVBD_PC.task_head.sim_config);
  daxa_u32 i = compute_i.x;
  if (i >= sim_config->rigid_body_count)
  {
    return;
  }

  daxa_u32* body_color = (daxa_u32*)(AVBD_PC.task_head.body_color);
  if (body_color[i] != AVBD_UNCOLORED)
  {
    return;
  }

  RigidBody* rigid_bodies = (RigidBody*)(AVBD_PC.task_head.rigid_bodies);
  RigidBody* r = &rigid_bodies[i];
  if (!avbd_is_dynamic(r))
  {
    return; // statics are never colored (they take no primal block update)
  }

  ManifoldNode* manifold_nodes = (ManifoldNode*)(AVBD_PC.task_head.manifold_nodes);
  CollisionEntry* collision_map = (CollisionEntry*)(AVBD_PC.task_head.collision_map);
  Manifold* collisions = (Manifold*)(AVBD_PC.task_head.collisions);
  daxa_u32 manifold_node_count = min(sim_config->manifold_node_count, BB_MAX_MANIFOLD_NODE_COUNT);
  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);

  daxa_u32 my_prio = avbd_priority(i, AVBD_PC.color);

  // pass 1: local-max test among uncolored dynamic neighbors + collect colored neighbor mask
  daxa_u32 used_mask = 0u;
  daxa_u32 node = r->manifold_node_index;
  daxa_u32 steps = 0u;
  while (node != MAX_U32 && steps < manifold_node_count)
  {
    daxa_u32 packed = avbd_node_to_packed(node, manifold_node_count, collision_count, manifold_nodes, collision_map);
    if (packed != MAX_U32)
    {
      daxa_u32 p = avbd_partner(&collisions[packed], i);
      if (p < sim_config->rigid_body_count && p != i)
      {
        RigidBody* rp = &rigid_bodies[p];
        if (avbd_is_dynamic(rp))
        {
          daxa_u32 pc = body_color[p];
          if (pc == AVBD_UNCOLORED)
          {
            if (avbd_priority(p, AVBD_PC.color) > my_prio)
            {
              return; // a higher-priority uncolored neighbor goes first this round
            }
          }
          else if (pc < BB_AVBD_MAX_BODY_COLORS)
          {
            used_mask |= (1u << pc);
          }
        }
      }
    }
    daxa_i32 next = manifold_nodes[node].next;
    node = next < 0 ? MAX_U32 : daxa_u32(next);
    ++steps;
  }

  daxa_u32 avail = ~used_mask;
  if (avail == 0u)
  {
    return; // >32 distinct neighbor colors (extreme hub): stays uncolored, validator reports it
  }
  daxa_u32 c = firstbitlow(avail);
  if (c >= BB_AVBD_MAX_BODY_COLORS)
  {
    return;
  }
  body_color[i] = c;
}

// validator: no two adjacent dynamic bodies share a color; uncolored dynamic bodies WITH
// contacts also count as violations (their block never gets solved).
[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_color_validate(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(AVBD_PC.task_head.sim_config);
  daxa_u32 i = compute_i.x;
  if (i >= sim_config->rigid_body_count)
  {
    return;
  }

  RigidBody* rigid_bodies = (RigidBody*)(AVBD_PC.task_head.rigid_bodies);
  RigidBody* r = &rigid_bodies[i];
  if (!avbd_is_dynamic(r))
  {
    return;
  }

  daxa_u32* body_color = (daxa_u32*)(AVBD_PC.task_head.body_color);
  daxa_u32 my_color = body_color[i];

  ManifoldNode* manifold_nodes = (ManifoldNode*)(AVBD_PC.task_head.manifold_nodes);
  CollisionEntry* collision_map = (CollisionEntry*)(AVBD_PC.task_head.collision_map);
  Manifold* collisions = (Manifold*)(AVBD_PC.task_head.collisions);
  daxa_u32 manifold_node_count = min(sim_config->manifold_node_count, BB_MAX_MANIFOLD_NODE_COUNT);
  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);

  bool has_contacts = false;
  daxa_u32 node = r->manifold_node_index;
  daxa_u32 steps = 0u;
  while (node != MAX_U32 && steps < manifold_node_count)
  {
    daxa_u32 packed = avbd_node_to_packed(node, manifold_node_count, collision_count, manifold_nodes, collision_map);
    if (packed != MAX_U32)
    {
      daxa_u32 p = avbd_partner(&collisions[packed], i);
      if (p < sim_config->rigid_body_count && p != i)
      {
        RigidBody* rp = &rigid_bodies[p];
        if (avbd_is_dynamic(rp))
        {
          has_contacts = true;
          if (my_color != AVBD_UNCOLORED && body_color[p] == my_color && p > i)
          {
            daxa_u32 v;
            InterlockedAdd(sim_config->avbd_violations, 1u, v); // adjacent same color (count once per edge)
          }
        }
      }
    }
    daxa_i32 next = manifold_nodes[node].next;
    node = next < 0 ? MAX_U32 : daxa_u32(next);
    ++steps;
  }

  if (my_color == AVBD_UNCOLORED)
  {
    if (has_contacts)
    {
      daxa_u32 v;
      InterlockedAdd(sim_config->avbd_violations, 1u, v); // contacted body left uncolored
    }
    return;
  }

  daxa_u32 prevc;
  InterlockedMax(sim_config->avbd_color_count, my_color + 1u, prevc);
}
