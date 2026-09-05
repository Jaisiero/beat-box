#pragma once

// AVBD (Augmented Vertex Block Descent, Giles/Diaz/Yuksel, SIGGRAPH 2025).
// Shared body-graph traversal; numerical constraints and individual passes stay separate.
#include "rigid_body_math.slang"

[[vk::push_constant]] AvbdPushConstants AVBD_PC;

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
