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
// NOTE: `body` must be the PERSISTENT body id, not the sorted index. The sorted index
// reshuffles with the radix sort whenever anything moves, which re-colored the graph
// every step -> the Gauss-Seidel solve order (color = phase) changed per step -> the
// residual redistributed differently each step ("order noise"), a micro-jitter floor
// that kept settling crusts hovering above the sleep threshold (user-diagnosed).
// Hashing the persistent id makes the coloring a pure function of the contact graph:
// identical every step at rest, deterministic always.
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
    sim_config->avbd_stick_count = 0u; // re-accumulated by the warm-start dispatch
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

  // anti-spin clamp (reference solver.cpp)
  daxa_f32 om_len = length(r->omega);
  if (om_len > 50.0f)
  {
    r->omega *= 50.0f / om_len;
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

  r->prev_velocity = r->velocity; // for next step's adaptive warm start
  r->velocity = (r->position - s->pos_start) * inv_dt;

  // dq = q_cur * conjugate(q_start); small-angle: omega = 2*dq.v/dt (sign-fixed)
  Quaternion qs = Quaternion(s->rot_start.v, s->rot_start.w);
  Quaternion qc = Quaternion(r->rotation.v, r->rotation.w);
  Quaternion qs_conj = Quaternion(-qs.v, qs.w);
  Quaternion dq = qc * qs_conj;
  daxa_f32 sign = dq.w < 0.0f ? -1.0f : 1.0f;
  r->omega = dq.v * (2.0f * sign * inv_dt);

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
}

// ----- F3: contact constraints (normal only), primal 6x6 block solve + dual updates -----
//
// Direct transcription of the reference AVBD discrete scheme (Giles et al. 2025,
// savant117/avbd-demo2d, postStabilize mode):
//   C(x) = C0*(1 - stab_alpha) + J . dpose       (truncated Taylor at step start, Sec 4)
//   force f = clamp(k*C + lambda, -inf, 0)        (contacts only push; lambda <= 0)
//   primal: g += J*f, H += k*J J^T  -> x += solve(H, -g)  (no geometric stiffness, Sec 3.5)
//   dual (main sweeps only): lambda = f; if lambda < 0: k += beta*|C| (linear ramp, Eq 16)
//   post-stab sweep: stab_alpha=0 AFTER velocity reconstruction (depenetration w/o momentum)
// Storage reuse: Contact.normal_impulse = lambda (<= 0), Contact.mass_coefficient = penalty k.

daxa_f32vec3 avbd_small_angle_delta(Quaternion q_now, Quaternion q_start)
{
  Quaternion qs_conj = Quaternion(-q_start.v, q_start.w);
  Quaternion dq = q_now * qs_conj;
  daxa_f32 sign = dq.w < 0.0f ? -1.0f : 1.0f;
  return dq.v * (2.0f * sign);
}

// Current constraint value of contact j of manifold m (also outputs the world arms).
// Truncated Taylor series around the step-start pose (reference manifold.cpp):
//   C(x) = C0*(1 - stab_alpha) + J . dpose
// During the main sweeps stab_alpha=1: only the constraint DELTA is solved (pre-existing
// penetration C0 is invisible to the dynamics, so depenetration never injects momentum).
// The post-stabilization sweep passes stab_alpha=0 to correct C0 positionally AFTER the
// velocities were already reconstructed.
daxa_f32 avbd_contact_C(Manifold* m, daxa_u32 j, RigidBody* r1, RigidBody* r2,
                        AvbdBodyState* s1, AvbdBodyState* s2,
                        bool dyn1, bool dyn2, daxa_f32 stab_alpha,
                        out daxa_f32vec3 rr1, out daxa_f32vec3 rr2)
{
  daxa_f32vec3 n = m->normal;
  daxa_f32vec3 p = m->contacts[j].position;
  // world arms relative to the step-start centers (statics: current center == start center)
  daxa_f32vec3 c1 = dyn1 ? s1->pos_start : r1->position;
  daxa_f32vec3 c2 = dyn2 ? s2->pos_start : r2->position;
  rr1 = p - c1;
  rr2 = p - c2;

  daxa_f32 C0 = m->contacts[j].penetration + BB_AVBD_MARGIN;
  daxa_f32 C = C0 * (1.0f - stab_alpha);
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

// rotate a vector by a quaternion: v' = u + 2*qv x (qv x u + w*u)
daxa_f32vec3 avbd_q_rotate(Quaternion q, daxa_f32vec3 u)
{
  return u + 2.0f * cross(q.v, cross(q.v, u) + q.w * u);
}

// F4 friction: tangential constraint row along direction d (reference manifold.cpp rows
// i*2+1) with sticking anchors. The arms come from the per-contact anchors (Contact::
// anchor_a/anchor_b, local space): freshly re-anchored every step while slipping (then
// C0_t = 0 and the row measures only this step's slide), but REUSED while the contact
// sticks — then C0_t is the total tangential drift since anchoring and the row becomes a
// positional spring back to the anchor, up to the Coulomb bound (true static friction).
// C0_t enters with (1 - stab_alpha) like the normal row: invisible to the dynamics in the
// main sweeps, corrected positionally by the post-stabilization sweep (no momentum).
daxa_f32 avbd_dir_C(daxa_f32vec3 d, Manifold* m, daxa_u32 j,
                    RigidBody* r1, RigidBody* r2,
                    AvbdBodyState* s1, AvbdBodyState* s2,
                    bool dyn1, bool dyn2, daxa_f32 stab_alpha,
                    out daxa_f32vec3 ar1, out daxa_f32vec3 ar2, out daxa_f32 C0t)
{
  // step-start poses (statics: current pose == start pose)
  daxa_f32vec3 c1 = dyn1 ? s1->pos_start : r1->position;
  daxa_f32vec3 c2 = dyn2 ? s2->pos_start : r2->position;
  Quaternion q1 = dyn1 ? Quaternion(s1->rot_start.v, s1->rot_start.w) : Quaternion(r1->rotation.v, r1->rotation.w);
  Quaternion q2 = dyn2 ? Quaternion(s2->rot_start.v, s2->rot_start.w) : Quaternion(r2->rotation.v, r2->rotation.w);
  // world arms of the anchored contact points at step start
  ar1 = avbd_q_rotate(q1, m->contacts[j].anchor_a);
  ar2 = avbd_q_rotate(q2, m->contacts[j].anchor_b);
  // pre-existing tangential drift between the anchors (C grows when body2 leads along +d).
  // The positional pull-back only acts on the drift beyond a small slop: a deadband that
  // keeps the post-stabilization sweep from perpetually micro-correcting compressed pile
  // contacts (sub-mm jitter that vetoes sleeping). Drift stays bounded by the slop.
  C0t = dot(d, (c2 + ar2) - (c1 + ar1));
  daxa_f32 c0_eff = sign(C0t) * max(0.0f, abs(C0t) - BB_AVBD_STICK_SLOP);
  daxa_f32 C = c0_eff * (1.0f - stab_alpha);
  if (dyn1)
  {
    daxa_f32vec3 dp1 = r1->position - s1->pos_start;
    daxa_f32vec3 dth1 = avbd_small_angle_delta(r1->rotation, s1->rot_start);
    C -= dot(d, dp1 + cross(dth1, ar1));
  }
  if (dyn2)
  {
    daxa_f32vec3 dp2 = r2->position - s2->pos_start;
    daxa_f32vec3 dth2 = avbd_small_angle_delta(r2->rotation, s2->rot_start);
    C += dot(d, dp2 + cross(dth2, ar2));
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
  // step-start poses for the anchor local frames (prepare already jumped the bodies to the
  // warmstarted pose, so dynamics read pos/rot_start from avbd_state; statics never move)
  daxa_f32vec3 c1 = dyn1 ? states[b1].pos_start : r1->position;
  daxa_f32vec3 c2 = dyn2 ? states[b2].pos_start : r2->position;
  Quaternion q1c = dyn1 ? Quaternion(-states[b1].rot_start.v, states[b1].rot_start.w) : Quaternion(-r1->rotation.v, r1->rotation.w);
  Quaternion q2c = dyn2 ? Quaternion(-states[b2].rot_start.v, states[b2].rot_start.w) : Quaternion(-r2->rotation.v, r2->rotation.w);

  daxa_u32 cc = min((daxa_u32)max(m->contact_count, 0), MAX_CONTACT_POINT_COUNT);
  for (daxa_u32 j = 0u; j < cc; ++j)
  {
    bool matched = m->contacts[j].warm_start != 0u && sim_config->has_flag(SimFlag::WARM_STARTING);
    if (!matched)
    {
      // fresh contact: zero force estimates, minimum penalties
      m->contacts[j].normal_impulse = 0.0f;
      m->contacts[j].mass_coefficient = BB_AVBD_PENALTY_MIN;
      m->contacts[j].tangent_impulse[0] = 0.0f;
      m->contacts[j].tangent_impulse[1] = 0.0f;
      m->contacts[j].tangent_mass[0] = BB_AVBD_PENALTY_MIN;
      m->contacts[j].tangent_mass[1] = BB_AVBD_PENALTY_MIN;
      m->contacts[j].stick = 0u;
    }
    else if (!dyn1 && !dyn2)
    {
      // SLEEPING STASIS (user-found chain-reaction wakes: "a small perturbation turns
      // whole red regions blue in a propagating wave"): the gamma decay below used to
      // run on SLEEPING pairs too, so a contact asleep N frames had its stiffness
      // collapse k *= 0.99^N toward 1 while its lambda stayed loaded. Waking the region
      // resumed the solver on force-without-stiffness contacts: the area SANK under its
      // own weight while the ramp regrew k from 1, the motion churned the clips (fresh,
      // blue) and woke the neighbors - whose k had also collapsed - a propagating
      // softness wave. A sleeping contact is in STASIS: keep k and lambda bit-exact.
      m->contacts[j].normal_impulse = min(m->contacts[j].normal_impulse, 0.0f);
    }
    else
    {
      // post-stabilization mode (reference solver.cpp): keep the FULL lambda from last step
      // (it never saw C0, so it is the steady contact force, not an impact spike) and only
      // decay the penalty so it can shrink when contacts calm down.
      daxa_f32 k = m->contacts[j].mass_coefficient;
      m->contacts[j].mass_coefficient = clamp(k * BB_AVBD_GAMMA, BB_AVBD_PENALTY_MIN, BB_AVBD_PENALTY_MAX);
      // lambda (normal_impulse) kept as is; force convention lambda <= 0
      m->contacts[j].normal_impulse = min(m->contacts[j].normal_impulse, 0.0f);
      // friction rows: same treatment (lambda kept, penalty decayed); tangent lambda is
      // free-signed inside its box bounds
      [unroll]
      for (daxa_u32 t = 0u; t < 2u; ++t)
      {
        daxa_f32 kt = m->contacts[j].tangent_mass[t];
        m->contacts[j].tangent_mass[t] = clamp(kt * BB_AVBD_GAMMA, BB_AVBD_PENALTY_MIN, BB_AVBD_PENALTY_MAX);
      }
    }
    // anchor maintenance (reference initialize()): sticking contacts arrived here with their
    // old anchors (update_from_old reused them); everything else re-anchors at the fresh
    // clipped point, expressed in each body's step-start local frame
    bool sticking = matched && m->contacts[j].stick != 0u;
    if (!sticking)
    {
      daxa_f32vec3 p = m->contacts[j].position;
      m->contacts[j].anchor_a = avbd_q_rotate(q1c, p - c1);
      m->contacts[j].anchor_b = avbd_q_rotate(q2c, p - c2);
    }
    if (sticking)
    {
      daxa_u32 prevd;
      InterlockedAdd(sim_config->avbd_stick_count, 1u, prevd);
    }
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
          daxa_f32 C = avbd_contact_C(m, j, r1, r2, s1, s2, dyn1, dyn2, AVBD_PC.stab_alpha, rr1, rr2);
          daxa_f32 lam = m->contacts[j].normal_impulse; // <= 0 (force convention)
          daxa_f32 k = m->contacts[j].mass_coefficient;
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
          // friction rows: box-bounded tangent forces, bounds refreshed from the LATEST
          // normal lambda (reference updates fmin/fmax in every computeConstraint call);
          // arms come from the sticking anchors (== the clip point while slipping)
          daxa_f32 bound = mu * abs(lam);
          [unroll]
          for (daxa_u32 t = 0u; t < 2u; ++t)
          {
            daxa_f32vec3 td = m->tangent_vectors[t];
            daxa_f32vec3 ar1, ar2;
            daxa_f32 C0t;
            daxa_f32 Ct = avbd_dir_C(td, m, j, r1, r2, s1, s2, dyn1, dyn2, AVBD_PC.stab_alpha, ar1, ar2, C0t);
            daxa_f32 lam_t = m->contacts[j].tangent_impulse[t];
            daxa_f32 kt = m->contacts[j].tangent_mass[t];
            daxa_f32 ft = clamp(kt * Ct + lam_t, -bound, bound);
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

  // no step limiting: the reference scheme relies on the inertia term + penalty curvature
  daxa_f32vec3 dlin = daxa_f32vec3(dx[0], dx[1], dx[2]);
  daxa_f32vec3 dang = daxa_f32vec3(dx[3], dx[4], dx[5]);

  r->position += dlin;
  daxa_f32vec3 half = dang * 0.5f;
  Quaternion dq = Quaternion(half, 1.0f);
  Quaternion qn = (dq * r->rotation).normalize();
  r->rotation.v = qn.v;
  r->rotation.w = qn.w;
}

// ----- shock propagation: support-depth BFS over the contact graph -----
//
// depth 0 = static or sleeping bodies (they ARE ground for whatever rests on them);
// each relax pass lowers a body's depth to min(depth, touching partner's depth + 1).
// BB_AVBD_SHOCK_LAYERS passes recognize piles that many layers tall; taller stacks
// saturate into the last layer (still a consistent ordering for the cascade).

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_depth_reset(daxa_u32vec3 compute_i : SV_DispatchThreadID)
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
  AvbdBodyState* states = (AvbdBodyState*)(AVBD_PC.task_head.avbd_state);
  RigidBody* r = &rigid_bodies[i];
  bool grounded = !avbd_is_dynamic(r) || (r->flags & RigidBodyFlag::SLEEPING) != RigidBodyFlag::NONE;
  states[i].support_depth = grounded ? 0u : MAX_U32;
}

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_depth_relax(daxa_u32vec3 compute_i : SV_DispatchThreadID)
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
  if (m->contact_count <= 0)
  {
    return;
  }
  AvbdBodyState* states = (AvbdBodyState*)(AVBD_PC.task_head.avbd_state);
  daxa_u32 d1 = states[b1].support_depth;
  daxa_u32 d2 = states[b2].support_depth;
  daxa_u32 prev;
  if (d2 != MAX_U32 && d2 + 1u < d1)
  {
    InterlockedMin(states[b1].support_depth, d2 + 1u, prev);
  }
  if (d1 != MAX_U32 && d1 + 1u < d2)
  {
    InterlockedMin(states[b2].support_depth, d1 + 1u, prev);
  }
}

// dual: one thread per manifold; assign lambda = clamped force, grow the penalty linearly
// while the force is strictly inside its bounds (main sweeps only; skipped in post-stab)
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
  daxa_f32 mu = sqrt(max(r1->friction, 0.0f) * max(r2->friction, 0.0f));
  for (daxa_u32 j = 0u; j < cc; ++j)
  {
    daxa_f32vec3 rr1, rr2;
    // dual updates only run during the main sweeps -> stab_alpha = 1 (delta-only constraint)
    daxa_f32 C = avbd_contact_C(m, j, r1, r2, &states[b1], &states[b2], dyn1, dyn2, 1.0f, rr1, rr2);
    daxa_f32 lam = m->contacts[j].normal_impulse;
    daxa_f32 k = m->contacts[j].mass_coefficient;
    // lambda update (reference Eq 11): ASSIGN the clamped force estimate
    daxa_f32 new_lam = min(k * C + lam, 0.0f);
    m->contacts[j].normal_impulse = new_lam;
    // penalty ramp (Eq 16): LINEAR growth, only while the force is not at its clamp bound
    if (new_lam < 0.0f)
    {
      m->contacts[j].mass_coefficient = min(k + BB_AVBD_BETA * abs(C), BB_AVBD_PENALTY_MAX);
    }
    // friction rows: bounds from the ENTRY normal lambda (the reference recomputes
    // fmin/fmax in computeConstraint BEFORE this dual pass touches the rows)
    daxa_f32 bound = mu * abs(lam);
    bool stick_ok = true;
    daxa_f32 drift2 = 0.0f;
    [unroll]
    for (daxa_u32 t = 0u; t < 2u; ++t)
    {
      daxa_f32vec3 td = m->tangent_vectors[t];
      daxa_f32vec3 ar1, ar2;
      daxa_f32 C0t;
      daxa_f32 Ct = avbd_dir_C(td, m, j, r1, r2, &states[b1], &states[b2], dyn1, dyn2, 1.0f, ar1, ar2, C0t);
      daxa_f32 lam_t = m->contacts[j].tangent_impulse[t];
      daxa_f32 kt = m->contacts[j].tangent_mass[t];
      daxa_f32 new_lam_t = clamp(kt * Ct + lam_t, -bound, bound);
      m->contacts[j].tangent_impulse[t] = new_lam_t;
      if (new_lam_t > -bound && new_lam_t < bound)
      {
        m->contacts[j].tangent_mass[t] = min(kt + BB_AVBD_BETA * abs(Ct), BB_AVBD_PENALTY_MAX);
      }
      else
      {
        stick_ok = false; // friction saturated -> kinetic regime
      }
      drift2 += C0t * C0t;
    }
    // stick determination (reference computeConstraint, re-evaluated every dual iteration;
    // the last one decides whether the next step reuses the anchors)
    m->contacts[j].stick = (stick_ok && drift2 < BB_AVBD_STICK_THRESH * BB_AVBD_STICK_THRESH) ? 1u : 0u;
  }
}

// ----- inelastic impact treatment (e = 0), post-FIN velocity passes -----
//
// The penalty ramp couples contact stiffness with impact ELASTICITY: a rain cube punching
// deep stores beta-scaled spring energy within the step and fires it back as separation
// velocity (user-judged trade-off: beta=1e4 settles 10x calmer but interpenetrates too
// visibly; beta=1e5 is stiff but bouncy). This pass keeps the stiff ramp and removes the
// bounce AT THE SOURCE: any contact whose bodies arrived APPROACHING faster than
// ~2*g*dt and ended the step SEPARATING gets the separation velocity removed (perfectly
// inelastic impact). Resting contacts never pass the approach gate, so stacks, sliders
// and sleeping are untouched.
//
// Two races-free passes: IMP_J (per manifold, READS only) computes the per-contact
// impulse scalar into Contact.normal_mass (a PGS-only scratch field, dead under AVBD);
// IMP_APPLY (per awake dynamic body) walks the body's own chain and applies its own
// dv/dw share - both bodies compute the same j from the same inputs, no atomics needed.

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

  for (daxa_u32 j = 0u; j < cc; ++j)
  {
    m->contacts[j].normal_mass = 0.0f; // scratch must be defined for IMP_APPLY either way
    daxa_f32vec3 p = m->contacts[j].position;
    daxa_f32vec3 ra = p - r1->position;
    daxa_f32vec3 rb = p - r2->position;

    // entry (pre-step) approach velocity: prev_velocity holds the step-entry linear
    // velocity (saved by FIN); omegas approximate entry values (no prev_omega stored)
    daxa_f32vec3 vrel_pre = (r2->prev_velocity + cross(r2->omega, rb)) - (r1->prev_velocity + cross(r1->omega, ra));
    daxa_f32 vn_pre = dot(n, vrel_pre);
    if (vn_pre > -2.0f * 9.8f * sim_config->dt) // not a real approach: resting/sliding contact
    {
      continue;
    }
    // post-step (reconstructed) relative normal velocity
    daxa_f32vec3 vrel = (r2->velocity + cross(r2->omega, rb)) - (r1->velocity + cross(r1->omega, ra));
    daxa_f32 vn = dot(n, vrel);
    if (vn <= 1.0e-4f) // not rebounding
    {
      continue;
    }
    // effective mass along n and the impulse that zeroes the rebound (e = 0)
    daxa_f32vec3 t1 = dyn1 ? cross(mul(inv_i1, cross(ra, n)), ra) : daxa_f32vec3(0.0f);
    daxa_f32vec3 t2 = dyn2 ? cross(mul(inv_i2, cross(rb, n)), rb) : daxa_f32vec3(0.0f);
    daxa_f32 K = inv_m1 + inv_m2 + dot(n, t1 + t2);
    if (K <= 1.0e-9f)
    {
      continue;
    }
    m->contacts[j].normal_mass = -vn / K; // attractive: removes exactly the rebound
  }
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

  daxa_u32 my_prio = avbd_priority(r->id, AVBD_PC.color);

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
            if (avbd_priority(rp->id, AVBD_PC.color) > my_prio)
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
