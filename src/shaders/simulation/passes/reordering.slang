#include "state_copy.slang"

[[vk::push_constant]] RigidBodyReorderingPushConstants RBR;
[[vk::push_constant]] ResetBodyLinkPushConstants RBL;
[[vk::push_constant]] NarrowPhasePushConstants NP;

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_rigid_body_reordering(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(RBR.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  daxa_u32 rb_count = sim_config->rigid_body_count;
  if (index >= rb_count)
  {
    return;
  }

  let rigid_bodies =
      (RigidBody*)(RBR.task_head.rigid_bodies);
  var lbvh_nodes = (LBVHNode*)(RBR.task_head.lbvh_nodes);
  let morton_codes =
      (MortonCode*)(RBR.task_head.morton_codes);
  var rigid_body_map =
      (RigidBodyEntry*)(RBR.task_head.rigid_body_map);
  var sorted_rigid_bodies = (RigidBody*)(RBR.task_head.rigid_body_sorted);

  // get the morton code rigid body index
  let rb_index = morton_codes[index].index;
  RigidBody* src_body = &rigid_bodies[rb_index];
  RigidBody* dst_body = &sorted_rigid_bodies[index];
  // get the rigid body id
  let rb_id = src_body->id;

  // update the rigid body map
  rigid_body_map[rb_id] = RigidBodyEntry(index);
  // put the rigid body in the sorted array by morton codes
  copy_rigid_body_fields(dst_body, src_body);
  // update the lbvh node
  lbvh_nodes[rb_count - 1 + index].primitive_idx = index;
}

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_reset_body_links(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(RBL.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  if (index >= sim_config->active_rigid_body_count)
  {
    return;
  }

  let active_rb = RBL.task_head.active_rigid_bodies[index];

  RigidBody* rigid_bodies =
      (RigidBody*)(RBL.task_head.rigid_bodies);

  let rb_map = (RigidBodyEntry*)(RBL.task_head.rigid_body_map);
  let rb_map_prev = (RigidBodyEntry*)(RBL.task_head.rigid_body_map_prev);

  rigid_bodies[rb_map[active_rb.rigid_body_id].index].active_index = index;

  RBL.task_head.scratch_body_links[index] = BodyLink(index, MAX_U32);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_chain_sort(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  var sim_config = (SimConfig*)(NP.task_head.sim_config);
  daxa_u32 i = compute_i.x;
  if (i >= sim_config->rigid_body_count)
  {
    return;
  }
  RigidBody* rigid_bodies = (RigidBody*)(NP.task_head.rigid_bodies);
  RigidBody* r = &rigid_bodies[i];
  if ((r->flags & RigidBodyFlag::DYNAMIC) == RigidBodyFlag::NONE)
  {
    return;
  }
  var nodes = (ManifoldNode*)(NP.task_head.rigid_body_link_manifolds);
  var cmap = (CollisionEntry*)(NP.task_head.collision_map);
  var colls = (Manifold*)(NP.task_head.collisions);
  daxa_u32 node_count = min(sim_config->manifold_node_count, BB_MAX_MANIFOLD_NODE_COUNT);
  daxa_u32 coll_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);

  const daxa_u32 CAP = 24u;
  daxa_u32 node_idx[24];
  daxa_u64 keys[24];
  daxa_u32 n = 0u;
  daxa_u32 current = r->manifold_node_index;
  daxa_u32 steps = 0u;
  daxa_u32 tail = MAX_U32; // first node beyond the cap: its sub-chain stays as-is
  while (current != MAX_U32 && current < node_count && steps < node_count)
  {
    if (n >= CAP)
    {
      tail = current;
      break;
    }
    daxa_u32 id = nodes[current].manifold_id;
    daxa_u64 key = 0xFFFFFFFFFFFFFFFFUL; // unresolvable entries sort last, stably
    if (id < coll_count)
    {
      daxa_u32 mi = cmap[id].index;
      if (mi < coll_count)
      {
        // canonical key: persistent pair ids (order-normalized) + manifold key
        daxa_u32 ida = rigid_bodies[colls[mi].obb1_index].id;
        daxa_u32 idb = rigid_bodies[colls[mi].obb2_index].id;
        daxa_u32 mn = min(ida, idb);
        daxa_u32 mx = max(ida, idb);
        key = (daxa_u64(mn) << 44) | (daxa_u64(mx) << 24) | daxa_u64(daxa_u32(colls[mi].key) & 0xFFFFFFu);
      }
    }
    node_idx[n] = current;
    keys[n] = key;
    ++n;
    daxa_i32 nx = nodes[current].next;
    current = nx < 0 ? MAX_U32 : daxa_u32(nx);
    ++steps;
  }
  if (n <= 1u)
  {
    return;
  }
  // in-register insertion sort, ascending by canonical key
  for (daxa_u32 a = 1u; a < n; ++a)
  {
    daxa_u64 k = keys[a];
    daxa_u32 v = node_idx[a];
    daxa_i32 b = daxa_i32(a) - 1;
    while (b >= 0 && keys[b] > k)
    {
      keys[b + 1] = keys[b];
      node_idx[b + 1] = node_idx[b];
      --b;
    }
    keys[b + 1] = k;
    node_idx[b + 1] = v;
  }
  // relink the chain in sorted order (chains are per-body disjoint: each node was
  // allocated by one body_link_manifold call, so parallel rewrites never collide)
  r->manifold_node_index = node_idx[0];
  for (daxa_u32 a = 0u; a + 1u < n; ++a)
  {
    nodes[node_idx[a]].next = daxa_i32(node_idx[a + 1u]);
  }
  nodes[node_idx[n - 1u]].next = tail == MAX_U32 ? -1 : daxa_i32(tail);
}
