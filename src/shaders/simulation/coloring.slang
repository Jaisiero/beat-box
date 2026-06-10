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
// (rotates the winner each round) + manifold id in the low bits (unique => no ties => no double-commit).
// Manifold ids are < BB_MAX_COLLISION_COUNT < 2^20, so they fit in the low 20 bits.
daxa_u32 gc_hash(daxa_u32 x)
{
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}
daxa_u32 gc_priority(daxa_u32 m, daxa_u32 round)
{
  daxa_u32 h = gc_hash(m * 0x9e3779b9u + round * 0x85ebca6bu) & 0xFFFu; // 12-bit per-round hash
  if (h == 0u) h = 1u;                                                  // keep priority > 0 (owner resets to 0)
  return (h << 20) | (m & 0xFFFFFu);
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
  daxa_u32 prio = gc_priority(m, sim_config->gc_round);
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
  daxa_u32 prio = gc_priority(m, sim_config->gc_round);
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
