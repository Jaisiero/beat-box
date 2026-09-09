#include "avbd_common.slang"

static const daxa_u32 AVBD_UNCOLORED = MAX_U32;

daxa_u32 avbd_priority(daxa_u32 body, daxa_u32 round)
{
  daxa_u32 x = body * 0x9e3779b9u + round * 0x85ebca6bu;
  x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16;
  return ((x & 0xFFFu) << 20) | (body & 0xFFFFFu);
}

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
  if (sim_config->has_flag(SimFlag::PROFILE_AVBD_WORK) && i < 32u)
  {
    sim_config->avbd_work_bodies[i] = 0u;
    sim_config->avbd_work_contacts[i] = 0u;
    sim_config->avbd_work_max_contacts[i] = 0u;
    sim_config->avbd_work_max_manifolds[i] = 0u;
  }
  if (i >= sim_config->rigid_body_count)
  {
    return;
  }
  daxa_u32* body_color = (daxa_u32*)(AVBD_PC.task_head.body_color);
  body_color[i] = AVBD_UNCOLORED;
  ((AvbdBodyState*)(AVBD_PC.task_head.avbd_state))[i].proposed_color = AVBD_UNCOLORED;
}

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
  // PROPOSE, do not commit: write proposed_color, NOT body_color. entry_avbd_color_commit copies it
  // to body_color after a barrier. Writing body_color here raced the neighbor reads above (a thread
  // could see a neighbor's same-round commit or not depending on warp order -> nondeterministic
  // coloring -> nondeterministic per-color solve order -> the main-solve divergence).
  ((AvbdBodyState*)(AVBD_PC.task_head.avbd_state))[i].proposed_color = c;
}

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_color_commit(daxa_u32vec3 compute_i : SV_DispatchThreadID)
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
    return; // already committed in an earlier round
  }
  daxa_u32 prop = ((AvbdBodyState*)(AVBD_PC.task_head.avbd_state))[i].proposed_color;
  if (prop < BB_AVBD_MAX_BODY_COLORS)
  {
    body_color[i] = prop;
  }
}

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

  // DETERMINISM (debug): hash this body's assigned color. If dbg_color_hash diverges between two
  // runs at the same step, the BODY COLORING is non-deterministic (different valid coloring =>
  // different per-color sweep order => exposed by the non-converged post-stab).
  if (sim_config->has_flag(SimFlag::DET_HASHES))
  {
    daxa_u32 _ch;
    InterlockedXor(sim_config->dbg_color_hash, (i + 1u) * (my_color + 1u) * 0x9e3779b9u, _ch);
  }

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
