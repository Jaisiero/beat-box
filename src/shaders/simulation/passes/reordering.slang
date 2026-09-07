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

// Each manifold node belongs to exactly one body's chain. Keys use persistent
// body ids, so atomic node allocation order is never a solver ordering key.
daxa_u64 chain_sort_key(daxa_u32 node, ManifoldNode* nodes, CollisionEntry* cmap,
                        Manifold* colls, RigidBody* bodies, daxa_u32 coll_count)
{
  daxa_u32 id = nodes[node].manifold_id;
  if (id >= coll_count) return 0xFFFFFFFFFFFFFFFFUL;
  daxa_u32 mi = cmap[id].index;
  if (mi >= coll_count) return 0xFFFFFFFFFFFFFFFFUL;
  daxa_u32 ida = bodies[colls[mi].obb1_index].id;
  daxa_u32 idb = bodies[colls[mi].obb2_index].id;
  return (daxa_u64(min(ida, idb)) << 44) | (daxa_u64(max(ida, idb)) << 24) |
         daxa_u64(daxa_u32(colls[mi].key) & 0xFFFFFFu);
}

daxa_u32 chain_sort_next(daxa_u32 node, ManifoldNode* nodes, daxa_u32 node_count)
{
  daxa_i32 next = nodes[node].next;
  return next >= 0 && daxa_u32(next) < node_count ? daxa_u32(next) : MAX_U32;
}

// Bottom-up linked-list merge sort: O(n log n), constant temporary storage.
// Used only beyond the register-array fast path. No contacts are dropped and
// no global scratch allocation or cross-workgroup synchronization is needed.
daxa_u32 chain_sort_long(daxa_u32 head, ManifoldNode* nodes, CollisionEntry* cmap,
                         Manifold* colls, RigidBody* bodies, daxa_u32 coll_count,
                         daxa_u32 node_count)
{
  for (daxa_u32 width = 1u; width < node_count; width *= 2u)
  {
    daxa_u32 left = head, output = MAX_U32, tail = MAX_U32;
    daxa_u32 merges = 0u, emitted = 0u;
    while (left != MAX_U32 && emitted < node_count)
    {
      ++merges;
      daxa_u32 right = left, left_count = 0u;
      for (daxa_u32 j = 0u; j < width && right != MAX_U32; ++j)
      {
        ++left_count;
        right = chain_sort_next(right, nodes, node_count);
      }
      daxa_u32 right_count = width;
      while ((left_count > 0u || (right_count > 0u && right != MAX_U32)) && emitted < node_count)
      {
        bool take_left = left_count > 0u;
        if (take_left && right_count > 0u && right != MAX_U32)
          take_left = chain_sort_key(left, nodes, cmap, colls, bodies, coll_count) <=
                      chain_sort_key(right, nodes, cmap, colls, bodies, coll_count);
        daxa_u32 chosen;
        if (take_left)
        {
          chosen = left;
          left = chain_sort_next(left, nodes, node_count);
          --left_count;
        }
        else
        {
          chosen = right;
          right = chain_sort_next(right, nodes, node_count);
          --right_count;
        }
        if (tail == MAX_U32) output = chosen;
        else nodes[tail].next = daxa_i32(chosen);
        tail = chosen;
        ++emitted;
      }
      left = right;
    }
    if (tail != MAX_U32) nodes[tail].next = -1;
    head = output;
    if (merges <= 1u) break;
  }
  return head;
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

  const bool verify = sim_config->has_flag(SimFlag::DET_HASHES);
  daxa_u32 original_count = 0u, original_sum = 0u, original_xor = 0u;
  if (verify)
  {
    daxa_u32 cursor = r->manifold_node_index;
    while (cursor != MAX_U32 && cursor < node_count && original_count < node_count)
    {
      ++original_count;
      original_sum += cursor;
      original_xor ^= cursor;
      daxa_i32 next = nodes[cursor].next;
      cursor = next < 0 ? MAX_U32 : daxa_u32(next);
    }
    daxa_u32 ignored;
    InterlockedMax(sim_config->dbg_chain_max, original_count, ignored);
    if (cursor != MAX_U32) InterlockedAdd(sim_config->dbg_chain_errors, 1u, ignored);
  }

  const daxa_u32 CAP = 24u;
  daxa_u32 node_idx[24];
  daxa_u64 keys[24];
  daxa_u32 n = 0u;
  daxa_u32 current = r->manifold_node_index;
  daxa_u32 steps = 0u;
  daxa_u32 tail = MAX_U32; // non-empty means the full-chain fallback is required
  while (current != MAX_U32 && current < node_count && steps < node_count)
  {
    if (n >= CAP)
    {
      tail = current;
      break;
    }
    daxa_u64 key = chain_sort_key(current, nodes, cmap, colls, rigid_bodies, coll_count);
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
  if (tail != MAX_U32)
  {
    r->manifold_node_index = chain_sort_long(r->manifold_node_index, nodes, cmap, colls,
                                             rigid_bodies, coll_count, node_count);
  }
  else
  {
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
    nodes[node_idx[n - 1u]].next = -1;
  }
  if (verify)
  {
    daxa_u32 count = 0u, sum = 0u, checksum = 0u;
    daxa_u32 cursor = r->manifold_node_index;
    daxa_u64 previous_key = 0UL;
    bool valid = true;
    while (cursor != MAX_U32 && cursor < node_count && count < node_count)
    {
      daxa_u64 key = chain_sort_key(cursor, nodes, cmap, colls, rigid_bodies, coll_count);
      valid = valid && (count == 0u || previous_key <= key);
      previous_key = key;
      ++count;
      sum += cursor;
      checksum ^= cursor;
      daxa_i32 next = nodes[cursor].next;
      cursor = next < 0 ? MAX_U32 : daxa_u32(next);
    }
    valid = valid && cursor == MAX_U32 && count == original_count &&
            sum == original_sum && checksum == original_xor;
    if (!valid)
    {
      daxa_u32 ignored;
      InterlockedAdd(sim_config->dbg_chain_errors, 1u, ignored);
    }
  }

}
