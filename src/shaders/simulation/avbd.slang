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
