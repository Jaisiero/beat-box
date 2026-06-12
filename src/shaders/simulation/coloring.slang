// Parallel graph coloring of the contact graph for the rigid-body solver.
//
// Each contact (Manifold) is an edge between two bodies (obb1_index, obb2_index). We assign every
// contact a color such that within one color no body appears twice. Then all contacts of a color can
// be solved in parallel with no data race (Box2D "SIMD Matters"). Static bodies (no DYNAMIC flag)
// never move, so they may appear in any number of colors and never consume a color slot.
//
// Algorithm: parallel speculative greedy, MAX_ROUNDS fixed rounds (the Daxa task graph is static).
//   per round: owner_reset -> assign_p1 (contend) -> assign_p2 (commit).
//   Correctness invariant (no repeated body per color) holds regardless of inter-thread timing,
//   because for a given (body, color) only the single InterlockedMax winner commits.
//
// All passes are dispatched over graph_color_dispatch = ceil(max(collision_count, rigid_body_count)/X)
// and guard each buffer access by its own bound.

#include "extensions.slang"

[[vk::push_constant]] GraphColorPushConstants GC;
[[vk::push_constant]] RigidBodyDispatcherPushConstants GCD; // coloring dispatcher reuses the dispatcher head (writes dispatch_buffer)

static const daxa_u32 GC_UNCOLORED = MAX_U32;

// ----- compute the indirect dispatch for all coloring passes: ceil(max(collision_count, rigid_body_count)/X) -----
// Runs after the narrow phase so g_c_info.collision_count is final. One thread.
[shader("compute")]
[numthreads(1, 1, 1)] void entry_graph_color_dispatcher(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  if (compute_i.x >= 1u) return;
  SimConfig* sim_config = (SimConfig*)(GCD.task_head.sim_config);
  DispatchBuffer* dispatch_buffer = (DispatchBuffer*)(GCD.task_head.dispatch_buffer);
  let coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  let n = max(coll, sim_config->rigid_body_count);
  let cnt = (n + RIGID_BODY_SIM_COMPUTE_X - 1u) / RIGID_BODY_SIM_COMPUTE_X;
  dispatch_buffer->graph_color_dispatch = daxa_u32vec3(cnt, 1, 1);
}

// A body is static (does not move) when it lacks the DYNAMIC flag (the floor is RigidBodyFlag::NONE).
// Detect by flag, NOT inv_mass (the floor's inv_mass is 1.0 by a latent scene bug).
bool gc_is_static(RigidBody* r)
{
  return (r->flags & RigidBodyFlag::DYNAMIC) == RigidBodyFlag::NONE;
}

// ----- once per frame: clear coloring state -----
[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_graph_color_reset(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(GC.task_head.sim_config);
  daxa_u32 i = compute_i.x;
  let rb_count = sim_config->rigid_body_count;
  let coll_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);

  daxa_u32* body_color_mask = (daxa_u32*)(GC.task_head.body_color_mask);
  daxa_u32* manifold_color = (daxa_u32*)(GC.task_head.manifold_color);
  daxa_u32* color_count = (daxa_u32*)(GC.task_head.color_count);

  if (i < rb_count) body_color_mask[i] = 0u;
  if (i < coll_count) manifold_color[i] = GC_UNCOLORED;
  if (i < BB_MAX_COLORS + 1u) color_count[i] = 0u;
  if (i == 0u)
  {
    sim_config->graph_color_count = 0u;
    sim_config->graph_color_violations = 0u;
    sim_config->graph_color_overflow = 0u;
    sim_config->gc_round = 0u;
    sim_config->gc_max_degree = 0u;
    sim_config->gc_max_degree_body = MAX_U32;
    sim_config->gc_max_degree_flags = 0u;
    sim_config->gc_satbody_degree = 0u;
    sim_config->gc_satbody_uncolored = 0u;
    sim_config->gc_satbody_pmin = MAX_U32;
    sim_config->gc_satbody_pmax = 0u;
    sim_config->gc_sat_nanflags = 0u;
    sim_config->gc_sat_pos_y = 0.0f;
    sim_config->dbg_maxv = 0u;   // per-frame stats reset; the dbg_ex_* latch is intentionally NOT reset
    sim_config->dbg_id_sum = 0u;
  }
}

// ----- once per round (and once before validate): clear the per-(body,color) arbitration owner -----
[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_graph_color_owner_reset(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(GC.task_head.sim_config);
  daxa_u32 i = compute_i.x;
  // advance the round counter once per round (seeds the fair-arbitration priority); thread 0 only
  if (i == 0u) sim_config->gc_round += 1u;
  if (i >= sim_config->rigid_body_count) return;
  daxa_u32* owner = (daxa_u32*)(GC.task_head.body_color_owner);
  for (daxa_u32 c = 0u; c < BB_MAX_COLORS; ++c)
    owner[i * BB_MAX_COLORS + c] = 0u; // InterlockedMax arbitration => init 0 (stores priorities now)
}

// Pick the lowest color free on both (dynamic) bodies; returns BB_MAX_COLORS if none free this round.
daxa_u32 gc_pick_color(daxa_u32 a, daxa_u32 b, bool sa, bool sb, daxa_u32* mask)
{
  daxa_u32 ma = sa ? 0u : mask[a];
  daxa_u32 mb = sb ? 0u : mask[b];
  daxa_u32 avail = ~(ma | mb);
  if (avail == 0u) return BB_MAX_COLORS;
  daxa_u32 c = firstbitlow(avail);
  return (c < BB_MAX_COLORS) ? c : BB_MAX_COLORS;
}

// Fair arbitration (Luby / Jones-Plassmann): each round a *different* contact should be able to win a
// contested (body,color), so low-id contacts don't starve. Priority = per-round hash in the high bits
// (rotates the winner each round) + a PERSISTENT pair-identity hash in the low bits.
// DETERMINISM (user-diagnosed dancing colors): the priority used to take the PACKED manifold index,
// which is InterlockedAdd allocation order - random every frame - so the whole coloring (and the
// PGS per-color solve order behind it) re-rolled per step. Keying on persistent body ids + manifold
// key makes the arbitration a pure function of the contact graph: identical colors at rest.
// (Low-bit hash collisions between two manifolds contending the same (body,color) cell are possible
// but ~1e-4 rare; the validator + overflow bucket already absorb them.)
daxa_u32 gc_hash(daxa_u32 x)
{
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}
// EXACT unique manifold code (user-found: the G view still danced at rest). The previous
// version HASHED the pair into the priority's low bits - hash collisions tie, and the up
// to six manifolds of one voxel pair (keys 90-95) share the pair hash entirely, so the
// InterlockedMax arbitration fell to a thread race and the coloring re-rolled every
// frame. Exact packing: lo(10) | hi(10) | key(4) - ids < 1024, and within one pair the
// low key nibble distinguishes the voxel axes (90-95 -> 10-15). Unique = tie-free =
// the coloring is a pure function of the contact graph (determinism + a still G view).
daxa_u32 gc_pair_id(daxa_u32 m)
{
  RigidBody* rigid_bodies = (RigidBody*)(GC.task_head.rigid_bodies);
  Manifold* collisions = (Manifold*)(GC.task_head.collisions);
  daxa_u32 ida = rigid_bodies[collisions[m].obb1_index].id;
  daxa_u32 idb = rigid_bodies[collisions[m].obb2_index].id;
  daxa_u32 lo = min(ida, idb);
  daxa_u32 hi = max(ida, idb);
  return ((lo & 0x3FFu) << 14) | ((hi & 0x3FFu) << 4) | (daxa_u32(collisions[m].key) & 0xFu);
}
daxa_u32 gc_priority(daxa_u32 pair_id, daxa_u32 round)
{
  daxa_u32 h = gc_hash(pair_id + round * 0x85ebca6bu) & 0xFFu; // 8-bit per-round fairness hash
  if (h == 0u) h = 1u;                                         // keep priority > 0 (owner resets to 0)
  return (h << 24) | (pair_id & 0xFFFFFFu);
}

// load the contact's two body indices; returns false if the contact is degenerate/out of range
bool gc_load_pair(daxa_u32 m, out daxa_u32 a, out daxa_u32 b)
{
  SimConfig* sim_config = (SimConfig*)(GC.task_head.sim_config);
  Manifold* collisions = (Manifold*)(GC.task_head.collisions);
  a = collisions[m].obb1_index;
  b = collisions[m].obb2_index;
  return !(a >= sim_config->rigid_body_count || b >= sim_config->rigid_body_count || a == b);
}

// ----- round phase 1: each uncolored contact contends for its desired color on both bodies -----
[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_graph_color_assign_p1(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(GC.task_head.sim_config);
  daxa_u32 m = compute_i.x;
  let coll_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (m >= coll_count) return;

  daxa_u32* manifold_color = (daxa_u32*)(GC.task_head.manifold_color);
  if (manifold_color[m] != GC_UNCOLORED) return;

  daxa_u32 a, b;
  if (!gc_load_pair(m, a, b)) return;

  RigidBody* rigid_bodies = (RigidBody*)(GC.task_head.rigid_bodies);
  bool sa = gc_is_static(&rigid_bodies[a]);
  bool sb = gc_is_static(&rigid_bodies[b]);
  daxa_u32* mask = (daxa_u32*)(GC.task_head.body_color_mask);
  daxa_u32 c = gc_pick_color(a, b, sa, sb, mask);
  if (c >= BB_MAX_COLORS) return;

  daxa_u32* owner = (daxa_u32*)(GC.task_head.body_color_owner);
  daxa_u32 prio = gc_priority(gc_pair_id(m), sim_config->gc_round);
  daxa_u32 prev;
  if (!sa) InterlockedMax(owner[a * BB_MAX_COLORS + c], prio, prev);
  if (!sb) InterlockedMax(owner[b * BB_MAX_COLORS + c], prio, prev);
}

// ----- round phase 2: commit the color if we won the arbitration on both bodies -----
[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_graph_color_assign_p2(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(GC.task_head.sim_config);
  daxa_u32 m = compute_i.x;
  let coll_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (m >= coll_count) return;

  daxa_u32* manifold_color = (daxa_u32*)(GC.task_head.manifold_color);
  if (manifold_color[m] != GC_UNCOLORED) return;

  daxa_u32 a, b;
  if (!gc_load_pair(m, a, b)) return;

  RigidBody* rigid_bodies = (RigidBody*)(GC.task_head.rigid_bodies);
  bool sa = gc_is_static(&rigid_bodies[a]);
  bool sb = gc_is_static(&rigid_bodies[b]);
  daxa_u32* mask = (daxa_u32*)(GC.task_head.body_color_mask);
  daxa_u32 c = gc_pick_color(a, b, sa, sb, mask);
  if (c >= BB_MAX_COLORS) return;

  daxa_u32* owner = (daxa_u32*)(GC.task_head.body_color_owner);
  daxa_u32 prio = gc_priority(gc_pair_id(m), sim_config->gc_round);
  bool won_a = sa || (owner[a * BB_MAX_COLORS + c] == prio);
  bool won_b = sb || (owner[b * BB_MAX_COLORS + c] == prio);
  if (won_a && won_b)
  {
    manifold_color[m] = c;
    daxa_u32 prev;
    if (!sa) InterlockedOr(mask[a], 1u << c, prev);
    if (!sb) InterlockedOr(mask[b], 1u << c, prev);
  }
}

// ----- validator: assert no color holds a repeated body; report colors used + violations -----
// Run AFTER an owner_reset (owner reused as a per-(body,color) seen counter, init 0).
[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_graph_color_validate(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(GC.task_head.sim_config);
  daxa_u32 m = compute_i.x;
  let coll_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);

  // DIAG: max colored-degree over dynamic bodies (popcount of the color mask). Contacts form an EDGE
  // coloring: a body of degree d forces >= d colors AND ~d rounds (one commit per (body,color) per
  // round), so a transient high-degree body is the structural cause of colors==rounds spike frames.
  // (This pass is dispatched over max(coll, rb), so body indices are covered.)
  if (m < sim_config->rigid_body_count)
  {
    RigidBody* rbs = (RigidBody*)(GC.task_head.rigid_bodies);
    if (!gc_is_static(&rbs[m]))
    {
      // DIAG probe stage 1 (pre-solve, post-advect): huge |v| here means the energy was already
      // present before this frame's solver ran (injected outside CS/CSR).
      bb_dbg_velocity_probe(sim_config, 1u, m, rbs[m].velocity, rbs[m].position.y);
      daxa_u32* mask_d = (daxa_u32*)(GC.task_head.body_color_mask);
      daxa_u32 d = countbits(mask_d[m]);
      daxa_u32 prevd;
      InterlockedMax(sim_config->gc_max_degree, d, prevd);
      if (d >= 30u) // DIAG: record a saturated body + the flags it presents (last writer wins, fine for diag)
      {
        sim_config->gc_max_degree_body = m;
        sim_config->gc_max_degree_flags = (daxa_u32)(rbs[m].flags);
      }
    }
  }
  if (m >= coll_count) return;

  daxa_u32* manifold_color = (daxa_u32*)(GC.task_head.manifold_color);
  daxa_u32 c = manifold_color[m];

  daxa_u32 a, b;
  if (!gc_load_pair(m, a, b)) return;

  RigidBody* rigid_bodies = (RigidBody*)(GC.task_head.rigid_bodies);
  bool sa = gc_is_static(&rigid_bodies[a]);
  bool sb = gc_is_static(&rigid_bodies[b]);

  // [b] runtime guard: count contacts the per-color solver won't touch
  // (uncolored, or color >= BB_MAX_COLORS_SOLVE dispatches). Must be 0 for a fully-solved pile.
  if (c == GC_UNCOLORED || c >= BB_MAX_COLORS_SOLVE)
  {
    daxa_u32 ov;
    InterlockedAdd(sim_config->graph_color_overflow, 1u, ov);
  }

  if (c == GC_UNCOLORED || c >= BB_MAX_COLORS)
  {
    // contact left for the serial overflow bucket; count it in the overflow slot
    daxa_u32* color_count = (daxa_u32*)(GC.task_head.color_count);
    daxa_u32 prev;
    InterlockedAdd(color_count[BB_COLOR_OVERFLOW], 1u, prev);
    return;
  }

  // colors used = max(color)+1
  daxa_u32 prevc;
  InterlockedMax(sim_config->graph_color_count, c + 1u, prevc);

  // seen-counter check: a dynamic body must not appear twice in the same color
  daxa_u32* owner = (daxa_u32*)(GC.task_head.body_color_owner);
  daxa_u32 prev;
  if (!sa)
  {
    InterlockedAdd(owner[a * BB_MAX_COLORS + c], 1u, prev);
    if (prev >= 1u) { daxa_u32 v; InterlockedAdd(sim_config->graph_color_violations, 1u, v); }
  }
  if (!sb)
  {
    InterlockedAdd(owner[b * BB_MAX_COLORS + c], 1u, prev);
    if (prev >= 1u) { daxa_u32 v; InterlockedAdd(sim_config->graph_color_violations, 1u, v); }
  }
}

// ----- DIAG pass (runs after validate): inspect the saturated body's manifolds -----
// Counts the true degree of gc_max_degree_body and the min/max partner index. Discriminates:
//   degree>=32 & pmin==pmax  -> one pair duplicated many times (multigraph edge)
//   degree>=32 & wide range  -> many manifolds reference this body (corrupt indices upstream)
//   degree small (~8)        -> mask got bits without backing manifolds (coloring-internal corruption)
[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_graph_color_validate2(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(GC.task_head.sim_config);
  daxa_u32 m = compute_i.x;
  let coll_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (m >= coll_count) return;

  daxa_u32 B = sim_config->gc_max_degree_body;
  if (B == MAX_U32) return;

  // thread 0: sanity-check the saturated body's state (NaN rotation makes SAT report collision vs everything)
  if (m == 0u)
  {
    RigidBody* rbs0 = (RigidBody*)(GC.task_head.rigid_bodies);
    RigidBody* rb = &rbs0[B];
    daxa_u32 f = 0u;
    if (isnan(rb->position.x) || isnan(rb->position.y) || isnan(rb->position.z)) f |= 1u;
    if (isnan(rb->rotation.v.x) || isnan(rb->rotation.v.y) || isnan(rb->rotation.v.z) || isnan(rb->rotation.w)) f |= 2u;
    if (isnan(rb->velocity.x) || isnan(rb->velocity.y) || isnan(rb->velocity.z)) f |= 4u;
    if (isnan(rb->omega.x) || isnan(rb->omega.y) || isnan(rb->omega.z)) f |= 8u;
    daxa_f32 qlen2 = rb->rotation.v.x * rb->rotation.v.x + rb->rotation.v.y * rb->rotation.v.y +
                     rb->rotation.v.z * rb->rotation.v.z + rb->rotation.w * rb->rotation.w;
    if (abs(qlen2 - 1.0f) > 0.01f) f |= 16u;
    sim_config->gc_sat_nanflags = f;
    sim_config->gc_sat_pos_y = rb->position.y;
  }

  daxa_u32 a, b;
  if (!gc_load_pair(m, a, b)) return;
  if (a != B && b != B) return;

  daxa_u32 partner = (a == B) ? b : a;
  daxa_u32 prev;
  InterlockedAdd(sim_config->gc_satbody_degree, 1u, prev);
  daxa_u32* manifold_color = (daxa_u32*)(GC.task_head.manifold_color);
  if (manifold_color[m] == GC_UNCOLORED)
    InterlockedAdd(sim_config->gc_satbody_uncolored, 1u, prev);
  InterlockedMin(sim_config->gc_satbody_pmin, partner, prev);
  InterlockedMax(sim_config->gc_satbody_pmax, partner, prev);
}
