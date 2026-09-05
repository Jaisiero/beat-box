#include "rigid_body_math.slang"
#include "LBVH.slang"

[[vk::push_constant]] RigidBodyGenerateHierarchyLinearBVHPushConstants GHLBVH;
[[vk::push_constant]] RigidBodyBuildBoundingBoxesLBVHPushConstants BBBLBVH;
[[vk::push_constant]] RigidBodyConvertBoundingBoxesLBVHPushConstants CBBLBVH;

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_generate_hierarchy_linear_bvh(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  var sim_config =
      (SimConfig*)(GHLBVH.task_head.sim_config);

  var rigid_bodies =
      (RigidBody*)(GHLBVH.task_head.rigid_bodies);

  var morton_codes =
      (MortonCode*)(GHLBVH.task_head.morton_codes);

  var lbvh_nodes = (LBVHNode*)(GHLBVH.task_head.lbvh_nodes);

  var lbvh_construction_info = (LBVHConstructionInfo*)(GHLBVH.task_head.lbvh_construction_info);

  let num_bodies = sim_config->rigid_body_count;
  let leaf_offset = num_bodies - 1;

  let index = compute_i.x;
  // leaf nodes
  if (index < num_bodies)
  {
    let morton_code = morton_codes[index];
    // Pointer access (NOT value-copy): a whole-struct load through a daxa buffer
    // pointer pads vec3->16 in Daxa 3.6, corrupting position/min/max and thus the
    // computed AABB. That feeds every LBVH leaf, breaking broad-phase detection.
    RigidBody* r = &rigid_bodies[morton_code.index];
    let rb_aabb = r->get_AABB_from_OBB();
    lbvh_nodes[leaf_offset + index] = LBVHNode(rb_aabb, -1, -1, morton_code.index);
  }

  // internal nodes
  if(index < num_bodies - 1) {
    // Find out which range of objects the node corresponds to.
    // (This is where the magic happens!)

    daxa::i32vec2 range = LBVH.determine_range(morton_codes, num_bodies, (daxa::i32)(index));
    daxa::i32 first = range.x;
    daxa::i32 last = range.y;

    // Determine where to split the range.
    daxa::i32 split = LBVH.find_split(morton_codes, num_bodies, first, last);

    // Select child_a.
    daxa::i32 child_a = MAX_I32;
    if (split == first)
        child_a = leaf_offset + split;
    else
        child_a = split;

    // Select child_b.
    daxa::i32 child_b = MAX_I32;
    if (split + 1 == last)
        child_b = leaf_offset + split + 1;
    else
        child_b = split + 1;

    // Record parent-child relationships.
    // FIXME: record relative indices instead of absolute indices.
    lbvh_nodes[index] = LBVHNode(Aabb(), child_a, child_b, MAX_U32);
    // Initialize bounds to sentinels (min = +inf-mapped = 0xFFFFFFFF, max = -inf-mapped = 0)
    // so the integer atomic min/max merge in build_bounding_boxes accumulates correctly.
    lbvh_construction_info[child_a] = LBVHConstructionInfo(index, 0, daxa_u32vec3(0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu), daxa_u32vec3(0u, 0u, 0u));
    lbvh_construction_info[child_b] = LBVHConstructionInfo(index, 0, daxa_u32vec3(0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu), daxa_u32vec3(0u, 0u, 0u));
  }

  // root node
  if(index == 0) {
    lbvh_construction_info[0] = LBVHConstructionInfo(0, 0, daxa_u32vec3(0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu), daxa_u32vec3(0u, 0u, 0u));
  }
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_build_bounding_boxes_linear_bvh(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  var sim_config =
      (SimConfig*)(BBBLBVH.task_head.sim_config);

  var lbvh_nodes = (LBVHNode*)(BBBLBVH.task_head.lbvh_nodes);

  var lbvh_construction_info = (LBVHConstructionInfo*)(BBBLBVH.task_head.lbvh_construction_info);

  let num_bodies = sim_config->rigid_body_count;
  let leaf_offset = num_bodies - 1;

  let index = compute_i.x;
  if(index >= num_bodies) {
    return;
  }

  // Portable cross-workgroup AABB merge WITHOUT coherent pointer loads.
  // Slang cannot do coherent loads/stores through pointers (shader-slang/slang#3870,
  // filed by the Daxa author), so we cannot reliably read a sibling node's AABB that
  // another workgroup wrote. Instead each leaf scatters its own bounds up the tree
  // using integer atomics (InterlockedMin/Max on order-preserving-mapped floats),
  // which ARE device-coherent through pointers. Every ancestor thus ends up holding
  // the union of all descendant leaf bounds. A separate convert pass turns the
  // integer-mapped bounds back into float AABBs for the internal nodes.
  // This also avoids the read-after-write race the visitation-counter approach has
  // when reading children with plain (non-atomic) loads.
  let leaf_aabb = lbvh_nodes[leaf_offset + index].aabb;
  let leaf_min = order_preserving_float3_map(leaf_aabb.minimum);
  let leaf_max = order_preserving_float3_map(leaf_aabb.maximum);

  daxa_u32 node_index = lbvh_construction_info[leaf_offset + index].parent;
  while(true) {
    InterlockedMin(lbvh_construction_info[node_index].bound_min.x, leaf_min.x);
    InterlockedMin(lbvh_construction_info[node_index].bound_min.y, leaf_min.y);
    InterlockedMin(lbvh_construction_info[node_index].bound_min.z, leaf_min.z);
    InterlockedMax(lbvh_construction_info[node_index].bound_max.x, leaf_max.x);
    InterlockedMax(lbvh_construction_info[node_index].bound_max.y, leaf_max.y);
    InterlockedMax(lbvh_construction_info[node_index].bound_max.z, leaf_max.z);
    if(node_index == 0) {
      break;
    }
    node_index = lbvh_construction_info[node_index].parent;
  }
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_convert_bounding_boxes_linear_bvh(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  var sim_config =
      (SimConfig*)(CBBLBVH.task_head.sim_config);

  let num_bodies = sim_config->rigid_body_count;
  let index = compute_i.x;
  // Only internal nodes [0, num_bodies-1) have integer-mapped bounds to convert.
  // Leaf nodes already carry their float AABB from the hierarchy generation pass.
  if(num_bodies < 2u || index >= num_bodies - 1u) {
    return;
  }

  var lbvh_nodes = (LBVHNode*)(CBBLBVH.task_head.lbvh_nodes);
  let lbvh_construction_info = (LBVHConstructionInfo*)(CBBLBVH.task_head.lbvh_construction_info);

  daxa_f32vec3 node_min = inverse_order_preserving_float3_map(lbvh_construction_info[index].bound_min);
  daxa_f32vec3 node_max = inverse_order_preserving_float3_map(lbvh_construction_info[index].bound_max);
  lbvh_nodes[index].aabb = Aabb(node_min, node_max);
}
