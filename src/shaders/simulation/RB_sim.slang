// #include "extensions.slang"
#include "collision_detection.slang"
#include "solvers.slang"
#include "LBVH.slang"

[[vk::push_constant]] RigidBodyDispatcherPushConstants RBD;
[[vk::push_constant]] RigidBodyGenerateMortonCodePushConstants GMC;
[[vk::push_constant]] RigidBodyRadixSortHistogramPushConstants RBRSH;
[[vk::push_constant]] RigidBodySingleRadixSortPushConstants SRS;
[[vk::push_constant]] RigidBodyGenerateHierarchyLinearBVHPushConstants GHLBVH;
[[vk::push_constant]] RigidBodyBuildBoundingBoxesLBVHPushConstants BBBLBVH;
[[vk::push_constant]] RigidBodyConvertBoundingBoxesLBVHPushConstants CBBLBVH;
[[vk::push_constant]] RigidBodyReorderingPushConstants RBR;
[[vk::push_constant]] ResetBodyLinkPushConstants RBL;
[[vk::push_constant]] BroadPhasePushConstants BP;
[[vk::push_constant]] NarrowPhaseDispatcherPushConstants NPD;
[[vk::push_constant]] NarrowPhasePushConstants NP;
[[vk::push_constant]] CollisionSolverDispatcherPushConstants CSD;
[[vk::push_constant]] IslandDispatcherPushConstants ID;
[[vk::push_constant]] RigidBodySimPushConstants RB_sim;
[[vk::push_constant]] IslandCounterPushConstants IC;
[[vk::push_constant]] IslandBuilderPushConstants IB;
[[vk::push_constant]] IslandPrefixSumPushConstants IPS;
[[vk::push_constant]] IslandBuilderBodyLink2IslandPushConstants BLI;
[[vk::push_constant]] IslandBuilderSortBodyLinkInIslandPushConstants SBLI;
[[vk::push_constant]] ManifoldIslandBuilderPushConstants MIB;
[[vk::push_constant]] ContactIslandGatherPushConstants CIG;
[[vk::push_constant]] ContactIslandDispatcherPushConstants CID;
[[vk::push_constant]] ManifoldIslandPrefixSumPushConstants MIPS;
[[vk::push_constant]] IslandBuilderManifoldLink2IslandPushConstants MLI;
[[vk::push_constant]] IslandBuilderSortManifoldLinkInIslandPushConstants MBLI;
[[vk::push_constant]] CollisionPreSolverPushConstants CPS;
[[vk::push_constant]] CollisionSolverPushConstants CS;
[[vk::push_constant]] RigidBodyIntegratePositionsPushConstants RB_integrate;
[[vk::push_constant]] CollisionSolverRelaxationPushConstants CSR;
[[vk::push_constant]] CreatePointsPushConstants CP;
[[vk::push_constant]] RigidBodyUpdatePushConstants RB_update;
[[vk::push_constant]] GraphColorSolvePushConstants GCS; // per-color solver passes (CPS/CS/CSR)
[[vk::push_constant]] SleepPushConstants SLP;            // island sleeping (reduce + apply)

// A body takes part in dynamics this step iff it is dynamic and its island is awake.
bool rb_is_awake_dynamic(RigidBody* r)
{
  return ((r->flags & RigidBodyFlag::DYNAMIC) != RigidBodyFlag::NONE) &&
         ((r->flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE);
}

[shader("compute")]
    [numthreads(1, 1, 1)] void
    entry_rigid_body_dispatcher(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  if (compute_i.x >= 1u)
  {
    return;
  }

  SimConfig* sim_config =
      (SimConfig*)(RBD.task_head.sim_config);

  DispatchBuffer* dispatch_buffer =
      (DispatchBuffer*)(RBD.task_head.dispatch_buffer);

  let rb_count = (sim_config->rigid_body_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
  dispatch_buffer->rigid_body_dispatch = daxa_u32vec3(rb_count, 1, 1);

  let arb_count = (sim_config->active_rigid_body_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
  dispatch_buffer->active_rigid_body_dispatch = daxa_u32vec3(arb_count, 1, 1);

  let rs_rb_count = (sim_config->rigid_body_count + RADIX_SORT_WORKGROUP_SIZE - 1) / RADIX_SORT_WORKGROUP_SIZE;
  dispatch_buffer->radix_sort_rigid_body_dispatch = daxa_u32vec3(rs_rb_count, 1, 1);

  // printf("radix_sort_rigid_body_dispatch %d\n", rs_rb_count);
}

    [shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_generate_morton_codes(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(GMC.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  if (index >= sim_config->rigid_body_count)
  {
    return;
  }

  RigidBody* rigid_bodies =
      (RigidBody*)(GMC.task_head.rigid_bodies);

  MortonCode* morton_codes =
      (MortonCode*)(GMC.task_head.morton_codes);
  
  var r = &rigid_bodies[index];

  // Normalize the world position into the unit cube [0,1] expected by morton3D.
  // Without this, raw world coordinates (floor at y=-50, cubes spread over tens of
  // units) saturate to degenerate values -> duplicate Morton codes -> a malformed
  // Karras LBVH -> broad phase finds no overlaps -> no collisions at all.
  const daxa_f32vec3 WORLD_MIN = daxa_f32vec3(-64.0f, -64.0f, -64.0f);
  const daxa_f32vec3 WORLD_MAX = daxa_f32vec3(64.0f, 64.0f, 64.0f);
  daxa_f32vec3 normalized_position = (r->position - WORLD_MIN) / (WORLD_MAX - WORLD_MIN);

  let morton_code = MortonCodeGenerator.morton3D(normalized_position);

#if defined(BB_DEBUG)
  r->clear_flag(RigidBodyFlag::COLLIDING);
  r->face_collided = 0u;
#endif // BB_DEBUG

  morton_codes[index] = MortonCode(morton_code, index);
}

#if defined(BB_RADIX_SORT_HISTOGRAM)
// groupshared variables
groupshared Atomic<daxa_u32> histogram[RADIX_SORT_BINS];

// Entry point with HLSL semantics
[numthreads(RADIX_SORT_WORKGROUP_SIZE, 1, 1)] void entry_radix_sort_histogram(daxa_u32vec3 compute_i : SV_DispatchThreadID, daxa_u32vec3 GroupThreadID : SV_GroupThreadID, daxa_u32vec3 GroupID : SV_GroupID)
{
  let lID = GroupThreadID.x;
  let wID = GroupID.x;

  var global_histograms =
      (daxa_u32*)(RBRSH.task_head.global_histograms);

  var morton_codes =
      (MortonCode*)(RBRSH.task_head.morton_codes);

  let sim_config =
      (SimConfig*)(RBRSH.task_head.sim_config);

  let shift = sim_config->radix_shift;

  // Initialize histogram
  if (lID < RADIX_SORT_BINS)
  {
    histogram[lID] = 0U;
  }
  GroupMemoryBarrierWithGroupSync();

  // Compute histogram
  for (daxa_u32 index = 0; index < NUM_BLOCKS_PER_WORKGROUP; index++)
  {
    let element_id = wID * NUM_BLOCKS_PER_WORKGROUP * RADIX_SORT_WORKGROUP_SIZE + index * RADIX_SORT_WORKGROUP_SIZE + lID;
    if (element_id < sim_config->rigid_body_count)
    {
      let val = morton_codes[element_id].morton_code;
      let bin = (val >> shift) & (RADIX_SORT_BINS - 1);
      histogram[bin].add(1U);
    }
  }
  GroupMemoryBarrierWithGroupSync();

  // Write histogram to global memory
  if (lID < RADIX_SORT_BINS)
  {
    let value = histogram[lID].load();
    global_histograms[RADIX_SORT_BINS * wID + lID] = value;
  } 
}
#endif // BB_RADIX_SORT_HISTOGRAM

[shader("compute")]
    [numthreads(RADIX_SORT_WORKGROUP_SIZE, 1, 1)] void entry_single_radix_sort(
        daxa_u32vec3 dispatch_thread_id : SV_DispatchThreadID, daxa_u32vec3 group_thread_id : SV_GroupThreadID, daxa_u32vec3 group_id : SV_GroupID, daxa_u32 groud_index : SV_GroupIndex // Flattened thread index in the group
    )
{
  daxa_u32 gID = dispatch_thread_id.x;
  daxa_u32 lID = group_thread_id.x;
  daxa_u32 wID = group_id.x;

  // Lane index in the current wave (subgroup)
  daxa_u32 lsID = WaveGetLaneIndex();
  // Wave ID within the thread group:
  // We'll assume sID = (lID / SUBGROUP_SIZE), since SUBGROUP_SIZE divides WORKGROUP_SIZE evenly.
  daxa_u32 sID = lID / SUBGROUP_SIZE;

  DispatchBuffer* dispatch_buffer =
      (DispatchBuffer*)(SRS.task_head.dispatch_buffer);

  daxa_u32* global_histograms =
      (daxa_u32*)(SRS.task_head.global_histograms);

  MortonCode* morton_codes_in =
      (MortonCode*)(SRS.task_head.morton_codes_in);

  MortonCode* morton_codes_out =
      (MortonCode*)(SRS.task_head.morton_codes_out);

  SimConfig* sim_config =
      (SimConfig*)(SRS.task_head.sim_config);

  daxa_u32 num_of_workgroups = dispatch_buffer->radix_sort_rigid_body_dispatch.x;

  daxa_u32 shift = sim_config->radix_shift;

  daxa_u32 num_of_elements = sim_config->rigid_body_count;

#if defined(BB_RADIX_SORT)
  radix_sort(lID, wID, lsID, sID, num_of_elements, num_of_workgroups, shift,
             morton_codes_in, morton_codes_out, global_histograms);
#endif // BB_RADIX_SORT
}


// credits: https://developer.nvidia.com/blog/thinking-parallel-part-iii-tree-construction-gpu/
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
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_broad_phase(daxa_u32vec3 compute_i : SV_DispatchThreadID) 
{
  var sim_config =
      (SimConfig*)(BP.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  if (index >= sim_config->rigid_body_count)
  {
    return;
  }

  // FIXME: split between static and dynamic objects
  // Pointer access (NOT value-copy): a whole-struct load through a daxa buffer
  // pointer pads vec3->16 in Daxa 3.6, corrupting the OBB and thus the query AABB.
  RigidBody* rigid_bodies = (RigidBody*)(BP.task_head.rigid_bodies);
  RigidBody* rigid_body = &rigid_bodies[index];
  let lbvh_nodes = (LBVHNode*)(BP.task_head.lbvh_nodes);
  var bp_collisions =
      (BroadPhaseCollision*)(BP.task_head.broad_phase_collisions);

  Aabb aabb = rigid_body->get_AABB_from_OBB();

  LBVH::broad_phase(sim_config, index, aabb, lbvh_nodes, bp_collisions);
}

[shader("compute")]
    [numthreads(1, 1, 1)] void
    entry_narrow_phase_dispatcher(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  if (compute_i.x >= 1u)
  {
    return;
  }

  SimConfig* sim_config =
      (SimConfig*)(NPD.task_head.sim_config);

  DispatchBuffer* dispatch_buffer =
      (DispatchBuffer*)(NPD.task_head.dispatch_buffer);

  let broad_phase_collision_count = min(sim_config->broad_phase_collision_count, BB_MAX_COLLISION_COUNT);
  let np_count = (broad_phase_collision_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
  dispatch_buffer->narrow_phase_dispatch = daxa_u32vec3(np_count, 1, 1);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_narrow_phase(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  var sim_config =
      (SimConfig*)(NP.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  daxa_u32 collision_count = min(sim_config->broad_phase_collision_count, BB_MAX_COLLISION_COUNT);
  if (index >= collision_count)
  {
    return;
  }

  let previous_sim_config =
      *NP.task_head.previous_sim_config;

  var rigid_body_map =
      (RigidBodyEntry*)(NP.task_head.rigid_body_map);

  var rigid_bodies =
      (RigidBody*)(NP.task_head.rigid_bodies);

  var previous_rigid_body_map =
      (RigidBodyEntry*)(NP.task_head.rigid_body_map_prev);

  var previous_rigid_bodies =
      (RigidBody*)(NP.task_head.previous_rigid_bodies);

  var rigid_body_link_manifolds =
      (ManifoldNode*)(NP.task_head.rigid_body_link_manifolds);

  var collision_map =
      (CollisionEntry*)(NP.task_head.collision_map);

  var collisions =
      (Manifold*)(NP.task_head.collisions);

  var previous_rigid_body_link_manifolds =
      (ManifoldNode*)(NP.task_head.previous_rigid_body_link_manifolds);

  var collision_map_prev =
      (CollisionEntry*)(NP.task_head.collision_map_prev);

  var old_collisions =
      (Manifold*)(NP.task_head.old_collisions);

  var body_links =
      (BodyLink*)(NP.task_head.scratch_body_links);
    
  var bp_collision =
      (BroadPhaseCollision)(NP.task_head.broad_phase_collisions[index]);

  narrow_phase(bp_collision, rigid_body_map, rigid_bodies, sim_config, previous_sim_config, rigid_body_link_manifolds, collision_map, collisions, previous_rigid_body_map, previous_rigid_bodies, previous_rigid_body_link_manifolds, collision_map_prev, old_collisions, body_links);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_rigid_body_sim(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      SimConfig* sim_config =
          (SimConfig*)(RB_sim.task_head.sim_config);

      daxa_u32 index = compute_i.x;
      if (index >= sim_config->rigid_body_count)
      {
        return;
      }

      RigidBody* rigid_bodies = (RigidBody*)(RB_sim.task_head.rigid_bodies);
      RigidBody* r = &rigid_bodies[index];

      // Static bodies never move: skip them so solver-injected velocity (e.g. concurrent impulse
      // writes on a shared static body) can never accumulate or drift their state.
      // Sleeping bodies skip too: their island is at rest (velocities were zeroed at sleep onset).
      if (!rb_is_awake_dynamic(r))
      {
        return;
      }

      // AVBD folds gravity into its inertial target (entry_avbd_prepare) and reconstructs
      // velocities itself; this advect pass belongs to the PGS family only.
      if (sim_config->solver_type == SimSolverType::AVBD)
      {
        return;
      }

      if (sim_config->has_flag(SimFlag::ADVECTION))
      {
        r->advect(sim_config);
      }

      // p_r->manifold_node_index = MAX_U32;
    }

// ----- NEIGHBORHOOD SLEEPING -----
// entry_sleep_reduce: per-body quiet timer (bit 31 = partner veto, cleared here every step).
[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_sleep_reduce(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(SLP.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  if (index == 0u)
  {
    sim_config->sleeping_count = 0u; // recomputed by entry_sleep_apply (separate dispatch)
  }
  if (index >= sim_config->rigid_body_count)
  {
    return;
  }

  RigidBody* rigid_bodies = (RigidBody*)(SLP.task_head.rigid_bodies);
  RigidBody* r = &rigid_bodies[index];
  if (!r->has_flag(RigidBodyFlag::DYNAMIC))
  {
    return;
  }

  if (!sim_config->has_flag(SimFlag::SLEEPING_ENABLED))
  {
    // feature disabled: everything stays awake
    r->sleep_timer = 0u;
    r->flags = RigidBodyFlag(daxa_u32(r->flags) & ~daxa_u32(RigidBodyFlag::SLEEPING));
    return;
  }

  daxa_f32 lin2 = dot(r->velocity, r->velocity);
  daxa_f32 ang2 = dot(r->omega, r->omega);
  daxa_u32 timer = r->sleep_timer & BB_SLEEP_TIMER_MASK; // also clears last step's veto bit
  r->sleep_timer = (lin2 < BB_SLEEP_LIN_VEL2 && ang2 < BB_SLEEP_ANG_VEL2) ? (timer + 1u) : 0u;
}

// entry_sleep_veto: one thread per manifold; a contact whose partner is not quiet vetoes the body
// (bit 31 of sleep_timer). Runs over the narrow-phase manifolds (pre island-sort buffer).
[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_sleep_veto(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(SLP.task_head.sim_config);
  if (!sim_config->has_flag(SimFlag::SLEEPING_ENABLED))
  {
    return;
  }

  daxa_u32 m = compute_i.x;
  daxa_u32 coll_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (m >= coll_count)
  {
    return;
  }

  Manifold* manifolds = (Manifold*)(SLP.task_head.collisions);
  daxa_u32 a = manifolds[m].obb1_index;
  daxa_u32 b = manifolds[m].obb2_index;
  if (a >= sim_config->rigid_body_count || b >= sim_config->rigid_body_count || a == b)
  {
    return;
  }

  RigidBody* rigid_bodies = (RigidBody*)(SLP.task_head.rigid_bodies);
  RigidBody* ra = &rigid_bodies[a];
  RigidBody* rb = &rigid_bodies[b];
  bool a_dyn = ra->has_flag(RigidBodyFlag::DYNAMIC);
  bool b_dyn = rb->has_flag(RigidBodyFlag::DYNAMIC);
  // statics are always "quiet"; only a non-quiet DYNAMIC partner vetoes
  bool a_loud = a_dyn && ((ra->sleep_timer & BB_SLEEP_TIMER_MASK) < BB_SLEEP_STEPS);
  bool b_loud = b_dyn && ((rb->sleep_timer & BB_SLEEP_TIMER_MASK) < BB_SLEEP_STEPS);
  // DEPTH veto: AVBD's post-stabilization corrects penetration positionally, without
  // velocity, so the velocity-based sleep test can freeze a deep pile mid-extraction
  // (the box pool froze with 88-213mm of standing interpenetration). Keep the pair
  // awake while its deepest contact is far from the surface: the penalty ramp keeps
  // shrinking the overlap while the bodies stay quiet, and they finally sleep flush.
  // The threshold IS the standing rest depth, so it belongs just ABOVE the solver's
  // achievable equilibrium: under a ~7-cube column the penalty springs compress to
  // ~45mm at the pile BOTTOM (buried, invisible - surface contacts rest millimetric),
  // and a tighter veto just trapped those pairs awake forever at maxv=0. (Chasing
  // deeper landings with a bigger veto was measured unproductive - the occasional
  // 55-65mm pocket leaves one still body unflagged, which is acceptable.)
  daxa_i32 cc = manifolds[m].contact_count;
  bool deep = false;
  for (daxa_i32 ci = 0; ci < cc; ++ci)
  {
    if (manifolds[m].contacts[ci].penetration < -0.05f) { deep = true; break; }
  }
  daxa_u32 prev;
  if ((a_loud || deep) && b_dyn)
  {
    InterlockedOr(rb->sleep_timer, BB_SLEEP_VETO_BIT, prev);
  }
  if ((b_loud || deep) && a_dyn)
  {
    InterlockedOr(ra->sleep_timer, BB_SLEEP_VETO_BIT, prev);
  }
}

// entry_sleep_apply: a body sleeps when its own timer passed AND no contact partner vetoed it;
// it wakes as soon as it is loud itself or any partner is.
[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_sleep_apply(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(SLP.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  if (index >= sim_config->rigid_body_count)
  {
    return;
  }
  if (!sim_config->has_flag(SimFlag::SLEEPING_ENABLED))
  {
    return; // reduce already cleared the flags
  }

  RigidBody* rigid_bodies = (RigidBody*)(SLP.task_head.rigid_bodies);
  RigidBody* r = &rigid_bodies[index];
  if (!r->has_flag(RigidBodyFlag::DYNAMIC))
  {
    return;
  }

  bool vetoed = (r->sleep_timer & BB_SLEEP_VETO_BIT) != 0u;
  bool quiet = (r->sleep_timer & BB_SLEEP_TIMER_MASK) >= BB_SLEEP_STEPS;
  bool sleep = quiet && !vetoed;
  bool was_sleeping = r->has_flag(RigidBodyFlag::SLEEPING);
  if (sleep)
  {
    if (!was_sleeping)
    {
      // falling asleep: freeze residual motion so the body holds rock-solid and wakes cleanly
      r->velocity = daxa_f32vec3(0.0f, 0.0f, 0.0f);
      r->omega = daxa_f32vec3(0.0f, 0.0f, 0.0f);
      r->flags = RigidBodyFlag(daxa_u32(r->flags) | daxa_u32(RigidBodyFlag::SLEEPING));
    }
    daxa_u32 prev;
    InterlockedAdd(sim_config->sleeping_count, 1u, prev);
  }
  else if (was_sleeping)
  {
    r->flags = RigidBodyFlag(daxa_u32(r->flags) & ~daxa_u32(RigidBodyFlag::SLEEPING));
  }
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_island_counter(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(IC.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  if (index >= sim_config->active_rigid_body_count)
  {
    return;
  }

  BodyLink* body_links =
      (BodyLink*)(IC.task_head.scratch_body_links);

  Island* islands =
      (Island*)(IC.task_head.islands);

#if defined(BB_DEBUG)
  // printf("IC index %d, body_link_index %d\n", index, body_link_index);
#endif // BB_DEBUG

  IslandBuilder builder = IslandBuilder(body_links, islands);

  builder.add_island(&sim_config->island_count, index);
}

[shader("compute")]
    [numthreads(1, 1, 1)] void
    entry_island_dispatcher(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      if (compute_i.x >= 1u)
      {
        return;
      }

      SimConfig* sim_config =
          (SimConfig*)(ID.task_head.sim_config);

      DispatchBuffer* dispatch_buffer =
          (DispatchBuffer*)(ID.task_head.dispatch_buffer);

      let i_count = (sim_config->island_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
      dispatch_buffer->island_dispatch = daxa_u32vec3(i_count, 1, 1);
    }

[shader("compute")]
    [numthreads(1, 1, 1)] void entry_collision_solver_dispatcher(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{

  if (compute_i.x >= 1u)
  {
    return;
  }

  SimConfig* sim_config =
      (SimConfig*)(CSD.task_head.sim_config);

  DispatchBuffer* dispatch_buffer =
      (DispatchBuffer*)(CSD.task_head.dispatch_buffer);

  var collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  var collision_c = (collision_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
  dispatch_buffer->collision_dispatch = daxa_u32vec3(collision_c, 1, 1);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_island_builder(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      SimConfig* sim_config =
          (SimConfig*)(IB.task_head.sim_config);

      daxa_u32 index = compute_i.x;
      if (index >= sim_config->active_rigid_body_count)
      {
        return;
      }

      let body_links =
          (BodyLink*)(IB.task_head.scratch_body_links);

      let islands =
          (Island*)(IB.task_head.islands);

      let active_rigid_bodies =
          (ActiveRigidBody*)(IB.task_head.active_rigid_bodies);

      var rigid_bodies =
          (RigidBody*)(IB.task_head.rigid_bodies);

      let rb_map = (RigidBodyEntry*)(IB.task_head.rigid_body_map);

      IslandBuilder builder = IslandBuilder(sim_config->active_rigid_body_count, body_links, active_rigid_bodies, sim_config->island_count, islands, sim_config->rigid_body_count, rigid_bodies, rb_map);

      builder.expand_island(index);
    }

[shader("compute")]
    [numthreads(1, 1, 1)] void entry_island_prefix_sum(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{

  if (compute_i.x >= 1u)
  {
    return;
  }

  SimConfig* sim_config =
      (SimConfig*)(IPS.task_head.sim_config);

  Island* islands =
      (Island*)(IPS.task_head.islands);

  IslandBuilder builder = IslandBuilder(sim_config->island_count, islands);

  builder.prefix_sum_islands();
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_body_link_to_island(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      SimConfig* sim_config =
          (SimConfig*)(BLI.task_head.sim_config);

      daxa_u32 index = compute_i.x;
      if (index >= sim_config->active_rigid_body_count)
      {
        return;
      }

      BodyLink* scratch_body_links =
          (BodyLink*)(BLI.task_head.scratch_body_links);

      BodyLinkIsland* body_link_island =
          (BodyLinkIsland*)(BLI.task_head.body_links);

      Island* islands =
          (Island*)(BLI.task_head.islands);

      IslandBuilder builder = IslandBuilder(sim_config->active_rigid_body_count, scratch_body_links, sim_config->island_count, islands, body_link_island);

      builder.body_link_to_island(index);
    }

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_sort_body_links_in_island(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(SBLI.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  if (index >= sim_config->island_count)
  {
    return;
  }

  BodyLinkIsland* body_link_island =
      (BodyLinkIsland*)(SBLI.task_head.body_links);

  Island* islands =
      (Island*)(SBLI.task_head.islands);

  IslandBuilder builder = IslandBuilder(sim_config->island_count, islands, sim_config->active_rigid_body_count, body_link_island);

//   builder.sort_body_links_in_island(index);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_manifold_island_builder(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      SimConfig* sim_config =
          (SimConfig*)(MIB.task_head.sim_config);

      daxa_u32 index = compute_i.x;

      daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
      if (index >= collision_count)
      {
        return;
      }

      Manifold* manifolds = (Manifold*)(MIB.task_head.collisions);
      Manifold* m = &manifolds[index];

      daxa_u32 body_a_index = m->obb1_index;
      daxa_u32 body_b_index = m->obb2_index;

      if (body_a_index >= sim_config->rigid_body_count || body_b_index >= sim_config->rigid_body_count || body_a_index == body_b_index)
      {
#if defined(BB_DEBUG)
    // printf("Manifold index %d, body_a_index %d, body_b_index %d\n", index, body_a_index, body_b_index);
#endif // BB_DEBUG
        return;
      }

      RigidBody* rigid_bodies =
          (RigidBody*)(MIB.task_head.rigid_bodies);

      daxa_u32 a_active_body_index = rigid_bodies[body_a_index].active_index;
      daxa_u32 b_active_body_index = rigid_bodies[body_b_index].active_index;

#if defined(BB_DEBUG)
  // printf("Manifold index %d, body_a_index %d (active_idx: %d), body_b_index %d (active_idx: %d), manifold count: %d, active_count: %d, island_count: %d\n", index, body_a_index, a_active_body_index, body_b_index, b_active_body_index, sim_config->g_c_info.collision_count, sim_config->active_rigid_body_count, sim_config->island_count);
#endif // BB_DEBUG

      daxa_u32 min_body_index = min(a_active_body_index, b_active_body_index);

      Island* islands =
          (Island*)(MIB.task_head.islands);

      BodyLink* body_links =
          (BodyLink*)(MIB.task_head.scratch_body_links);

      IslandBuilder builder = IslandBuilder(sim_config->active_rigid_body_count, body_links, sim_config->island_count, islands);

      builder.expand_manifold_island(min_body_index);
    }

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_contact_island_gather(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(CIG.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  if (index >= sim_config->island_count)
  {
    return;
  }

  Island* islands =
      (Island*)(CIG.task_head.islands);

  ContactIsland* contact_islands =
      (ContactIsland*)(CIG.task_head.contact_islands);

  ContactIslandBuilder builder = ContactIslandBuilder(sim_config->island_count, islands, &sim_config->contact_island_count, contact_islands);

  builder.gather_contact_islands(index);
}

[shader("compute")]
    [numthreads(1, 1, 1)] void
    entry_contact_island_dispatcher(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      if (compute_i.x >= 1u)
      {
        return;
      }

      SimConfig* sim_config =
          (SimConfig*)(CID.task_head.sim_config);

      DispatchBuffer* dispatch_buffer =
          (DispatchBuffer*)(CID.task_head.dispatch_buffer);

      var contact_island_count = min(sim_config->contact_island_count, BB_MAX_RIGID_BODY_COUNT);
      var ci_count = (contact_island_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
      dispatch_buffer->contact_island_dispatch = daxa_u32vec3(ci_count, 1, 1);
    }

[shader("compute")]
    [numthreads(1, 1, 1)] void entry_manifold_island_prefix_sum(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{

  if (compute_i.x >= 1u)
  {
    return;
  }

  SimConfig* sim_config =
      (SimConfig*)(MIPS.task_head.sim_config);

  ContactIsland* contact_islands =
      (ContactIsland*)(MIPS.task_head.contact_islands);

  Island* islands =
      (Island*)(MIPS.task_head.islands);

  ContactIslandBuilder builder = ContactIslandBuilder(sim_config->island_count, islands, &sim_config->contact_island_count, contact_islands);

  let contact_collision_count = builder.manifold_prefix_sum_islands();

#if defined(BB_DEBUG)
  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (contact_collision_count != collision_count)
  {
    printf("Manifold prefix sum failed, contact_collision_count: %d, collision_count: %d\n", contact_collision_count, collision_count);
  }
#endif // BB_DEBUG
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_manifold_link_to_island(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      SimConfig* sim_config =
          (SimConfig*)(MLI.task_head.sim_config);

      daxa_u32 index = compute_i.x;
      daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
      if (index >= collision_count)
      {
        return;
      }

      Manifold* manifolds = (Manifold*)(MLI.task_head.collisions);
      Manifold* m = &manifolds[index];

      daxa_u32 body_a_index = m->obb1_index;
      daxa_u32 body_b_index = m->obb2_index;

      if (body_a_index >= sim_config->rigid_body_count || body_b_index >= sim_config->rigid_body_count || body_a_index == body_b_index)
      {
#if defined(BB_DEBUG)
    // printf("Manifold link index %d, body_a_index %d, body_b_index %d\n", index, body_a_index, body_b_index);
#endif // BB_DEBUG
        return;
      }

      RigidBody* rigid_bodies =
          (RigidBody*)(MLI.task_head.rigid_bodies);

      daxa_u32 a_active_body_index = rigid_bodies[body_a_index].active_index;
      daxa_u32 b_active_body_index = rigid_bodies[body_b_index].active_index;

      daxa_u32 min_body_index = min(a_active_body_index, b_active_body_index);

      Island* islands =
          (Island*)(MLI.task_head.islands);

      BodyLink* body_links =
          (BodyLink*)(MLI.task_head.scratch_body_links);

      ContactIsland* contact_islands =
          (ContactIsland*)(MLI.task_head.contact_islands);

      ManifoldLinkIsland* manifold_links =
          (ManifoldLinkIsland*)(MLI.task_head.manifold_links);

      var collision_map = (CollisionEntry*)(MLI.task_head.collision_map);

      var collision_sorted = (Manifold*)(MLI.task_head.collision_sorted);

      ContactIslandBuilder builder = ContactIslandBuilder(sim_config->active_rigid_body_count, body_links, sim_config->island_count, islands, &sim_config->contact_island_count, contact_islands, collision_count, manifold_links);

      // stable island color key: min persistent id among the manifold's DYNAMIC bodies
      // (statics excluded — the floor would otherwise give every island the same color)
      RigidBody* ra = &rigid_bodies[body_a_index];
      RigidBody* rb = &rigid_bodies[body_b_index];
      daxa_u32 id_a = ((ra->flags & RigidBodyFlag::DYNAMIC) != RigidBodyFlag::NONE) ? ra->id : MAX_U32;
      daxa_u32 id_b = ((rb->flags & RigidBodyFlag::DYNAMIC) != RigidBodyFlag::NONE) ? rb->id : MAX_U32;
      daxa_u32 stable_key = min(id_a, id_b);

      let manifold_index = builder.manifold_link_to_island(index, a_active_body_index, b_active_body_index, stable_key);

      if(manifold_index != MAX_U32) {
        copy_manifold_fields(&collision_sorted[manifold_index], m);
        collision_map[index].index = manifold_index;
      }
    }

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_sort_manifold_links_in_island(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(MBLI.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  daxa_u32 contact_island_count = min(sim_config->contact_island_count, BB_MAX_RIGID_BODY_COUNT);
  if (index >= contact_island_count)
  {
    return;
  }

  ManifoldLinkIsland* manifold_links =
      (ManifoldLinkIsland*)(MBLI.task_head.manifold_links);

  ContactIsland* islands =
      (ContactIsland*)(MBLI.task_head.contact_islands);

  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  ContactIslandBuilder builder = ContactIslandBuilder(&sim_config->contact_island_count, islands, collision_count, manifold_links);

//   builder.sort_manifold_links_in_island(index);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_collision_pre_solver(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      SimConfig* sim_config =
          (SimConfig*)(CPS.task_head.sim_config);

      daxa_u32 index = compute_i.x;
#if defined(BB_SIM_DEBUG)
      if (index >= 1)
#else
  daxa_u32 contact_island_count = min(sim_config->contact_island_count, BB_MAX_RIGID_BODY_COUNT);
  if (index >= contact_island_count)
#endif // BB_SIM_DEBUG
      {
        return;
      }

#if defined(BB_SIM_DEBUG)
      daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
      for (daxa_u32 manifold_index = 0; manifold_index < collision_count; ++manifold_index)
      {
#else
  ContactIsland* islands =
      (ContactIsland*)(CPS.task_head.contact_islands);

  ManifoldLinkIsland* manifold_links =
      (ManifoldLinkIsland*)(CPS.task_head.manifold_links);

  let collision_map =
      (CollisionEntry*)(CPS.task_head.collision_map);

  daxa_u32 start_index = islands[index].start_index;
  daxa_u32 max_count = islands[index].count;

  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (start_index + max_count > collision_count || max_count == 0)
  {
#if defined(BB_DEBUG)
    printf("Collision pre solver failed index %d, start_index %d, max_count %d\n", index, start_index, max_count);
#endif // BB_DEBUG
    return;
  }

  for (daxa_u32 manifold_index = start_index; manifold_index < start_index + max_count; ++manifold_index)
  {
#endif // BB_SIM_DEBUG

        Manifold* manifolds = (Manifold*)(CPS.task_head.collisions);
        Manifold* manifold = &manifolds[manifold_index];

        if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index)
        {
          continue;
        }

        RigidBody* rigid_bodies = (RigidBody*)(CPS.task_head.rigid_bodies);
        RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
        RigidBody* r2 = &rigid_bodies[manifold->obb2_index];

        if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2))
        {
          continue; // sleeping/static pair: at rest
        }

        daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
        daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
        daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
        daxa::f32 hertz = min(MIN_CONTACT_HERTZ, inv_dt * 0.3333333333333333f);

        if (sim_config->solver_type == SimSolverType::PGS_SOFT)
        {
          PGS_SOFT.pre_step(manifold, inv_dt, r1, r2, accum_impulse, friction, sim_config->dt, hertz);
        }
        else if (sim_config->solver_type == SimSolverType::PGS)
        {
          PGS.pre_step(manifold, inv_dt, r1, r2, accum_impulse, friction);
        }

        // manifold + r1/r2 are pointers into the buffers; pre_step wrote contacts/velocity directly.
      }
    }

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_collision_solver(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(CS.task_head.sim_config);

  daxa_u32 index = compute_i.x;
#if defined(BB_SIM_DEBUG)
  if (index >= 1)
#else
  daxa_u32 contact_island_count = min(sim_config->contact_island_count, BB_MAX_RIGID_BODY_COUNT);
  if (index >= contact_island_count)
#endif // BB_SIM_DEBUG
  {
    return;
  }

#if defined(BB_SIM_DEBUG)
  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  for (daxa_u32 manifold_index = 0; manifold_index < collision_count; ++manifold_index)
  {
#else
  ContactIsland* islands =
      (ContactIsland*)(CS.task_head.contact_islands);

  ManifoldLinkIsland* manifold_links =
      (ManifoldLinkIsland*)(CS.task_head.manifold_links);

  let collision_map =
      (CollisionEntry*)(CS.task_head.collision_map);

  daxa_u32 start_index = islands[index].start_index;
  daxa_u32 max_count = islands[index].count;

  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (start_index + max_count > collision_count || max_count == 0)
  {
#if defined(BB_DEBUG)
    printf("Collision solver failed index %d, start_index %d, max_count %d\n", index, start_index, max_count);
#endif // BB_DEBUG
    return;
  }

  for (daxa_u32 manifold_index = start_index; manifold_index < start_index + max_count; ++manifold_index)
  {
#endif // BB_SIM_DEBUG

    Manifold* manifolds = (Manifold*)(CS.task_head.collisions);
    Manifold* manifold = &manifolds[manifold_index];

    if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index)
    {
      continue;
    }

    // Raw `RigidBody*` (scalar layout) instead of value-copy through the daxa BufferPtr.
    RigidBody* rigid_bodies = (RigidBody*)(CS.task_head.rigid_bodies);
    RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
    RigidBody* r2 = &rigid_bodies[manifold->obb2_index];

    if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2))
    {
      continue; // sleeping/static pair: at rest
    }

    daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
    daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
    daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;

    // Solve the collision
    if (sim_config->solver_type == SimSolverType::PGS_SOFT)
    {
      PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, true, inv_dt);
    }
    else if (sim_config->solver_type == SimSolverType::PGS)
    {
      PGS.solve(manifold, r1, r2, accum_impulse, friction);
    }

    // manifold + r1/r2 are pointers into the buffers; solve wrote contacts/velocity directly.
  }
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_integrate_positions_rigid_bodies(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      SimConfig* sim_config =
          (SimConfig*)(RB_integrate.task_head.sim_config);

      if (compute_i.x >= sim_config->rigid_body_count)
      {
        return;
      }

      daxa_u32 index = compute_i.x;

      RigidBody* rigid_bodies = (RigidBody*)(RB_integrate.task_head.rigid_bodies);
      RigidBody* r = &rigid_bodies[index];

      // Static bodies never integrate (they'd drift from solver-raced velocity writes);
      // sleeping bodies hold their pose until their island wakes.
      if (!rb_is_awake_dynamic(r))
      {
        return;
      }

      // AVBD writes positions directly during the primal descent; entry_avbd_finalize
      // reconstructs velocities. This velocity-level integration is PGS-family only.
      if (sim_config->solver_type == SimSolverType::AVBD)
      {
        return;
      }

      // DIAG probe stage 2 (after the CS sweep, before positions move): huge |v| first seen here
      // means warm-start/CS injected it this frame.
      bb_dbg_velocity_probe(sim_config, 2u, index, r->velocity, r->position.y);

      // Integrate positions
      if (sim_config->has_flag(SimFlag::ADVECTION))
      {
        r->integrate_positions(sim_config->dt);
      }
    }

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_collision_solver_relaxation(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(CSR.task_head.sim_config);

  daxa_u32 index = compute_i.x;

#if defined(BB_SIM_DEBUG)
  if (index >= 1)
#else
  daxa_u32 contact_island_count = min(sim_config->contact_island_count, BB_MAX_RIGID_BODY_COUNT);
  if (index >= contact_island_count)
#endif // BB_SIM_DEBUG
  {
    return;
  }

#if defined(BB_SIM_DEBUG)
  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  for (daxa_u32 manifold_index = 0; manifold_index < collision_count; ++manifold_index)
  {
#else
  ContactIsland* islands =
      (ContactIsland*)(CSR.task_head.contact_islands);

  ManifoldLinkIsland* manifold_links =
      (ManifoldLinkIsland*)(CSR.task_head.manifold_links);

  let collision_map =
      (CollisionEntry*)(CSR.task_head.collision_map);

  daxa_u32 start_index = islands[index].start_index;
  daxa_u32 max_count = islands[index].count;

  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (start_index + max_count > collision_count || max_count == 0)
  {
#if defined(BB_DEBUG)
    printf("Collision solver relaxation failed index %d, start_index %d, max_count %d\n", index, start_index, max_count);
#endif // BB_DEBUG
    return;
  }

  for (daxa_u32 manifold_index = start_index; manifold_index < start_index + max_count; ++manifold_index)
  {
#endif // BB_SIM_DEBUG

    Manifold* manifolds = (Manifold*)(CSR.task_head.collisions);
    Manifold* manifold = &manifolds[manifold_index];

    if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index)
    {
      continue;
    }

    // Raw `RigidBody*` (scalar layout) instead of value-copy through the daxa BufferPtr.
    RigidBody* rigid_bodies = (RigidBody*)(CSR.task_head.rigid_bodies);
    RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
    RigidBody* r2 = &rigid_bodies[manifold->obb2_index];

    if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2))
    {
      continue; // sleeping/static pair: at rest
    }

    daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
    daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
    daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;

    // Solve the collision with relaxation
    if (sim_config->solver_type == SimSolverType::PGS_SOFT)
    {
      PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, false, inv_dt);
    }

    // manifold + r1/r2 are pointers into the buffers; solve wrote contacts/velocity directly.
  }
}

// ===== per-color solver passes (graph coloring): each dispatch solves only manifolds of GCS.color =====
// Dispatched over all manifolds (collision_dispatch); a thread returns unless manifold_color[m]==GCS.color.
// Within a color no body repeats, so all kept manifolds write distinct bodies in parallel (no race).
[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_collision_pre_solver_color(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(GCS.task_head.sim_config);
  daxa_u32 m = compute_i.x;
  let coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (m >= coll) return;
  daxa_u32* manifold_color = (daxa_u32*)(GCS.task_head.manifold_color);
  if (manifold_color[m] != GCS.color) return;
  Manifold* manifold = &((Manifold*)(GCS.task_head.collisions))[m];
  if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index) return;
  RigidBody* rigid_bodies = (RigidBody*)(GCS.task_head.rigid_bodies);
  RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
  RigidBody* r2 = &rigid_bodies[manifold->obb2_index];
  if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2)) return; // sleeping/static pair: at rest
  daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
  daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
  daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
  daxa::f32 hertz = min(MIN_CONTACT_HERTZ, inv_dt * 0.3333333333333333f);
  // DIAG stage 4: runaway accumulator INHERITED from last frame (warm-start bomb already armed)
  // DIAG stage 6: contact point absurdly far from the body (ill-conditioned closest-point solve)
  for (daxa_i32 ci = 0; ci < manifold->contact_count; ++ci)
  {
    bb_dbg_impulse_probe(sim_config, 4u, manifold->obb1_index, manifold->obb2_index,
                         manifold->contacts[ci].normal_impulse, manifold->contacts[ci].penetration, manifold->normal.y);
    bb_dbg_far_contact_probe(sim_config, manifold->obb1_index, manifold->obb2_index,
                             manifold->contacts[ci].position - manifold->anchor_a,
                             manifold->contacts[ci].penetration, manifold->normal.y);
  }
  if (sim_config->solver_type == SimSolverType::PGS_SOFT)
    PGS_SOFT.pre_step(manifold, inv_dt, r1, r2, accum_impulse, friction, sim_config->dt, hertz);
}

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_collision_solver_color(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(GCS.task_head.sim_config);
  daxa_u32 m = compute_i.x;
  let coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (m >= coll) return;
  daxa_u32* manifold_color = (daxa_u32*)(GCS.task_head.manifold_color);
  if (manifold_color[m] != GCS.color) return;
  Manifold* manifold = &((Manifold*)(GCS.task_head.collisions))[m];
  if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index) return;
  RigidBody* rigid_bodies = (RigidBody*)(GCS.task_head.rigid_bodies);
  RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
  RigidBody* r2 = &rigid_bodies[manifold->obb2_index];
  if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2)) return; // sleeping/static pair: at rest
  daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
  daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
  daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
  if (sim_config->solver_type == SimSolverType::PGS_SOFT)
    PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, true, inv_dt);
}

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_collision_solver_relax_color(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(GCS.task_head.sim_config);
  daxa_u32 m = compute_i.x;
  let coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (m >= coll) return;
  daxa_u32* manifold_color = (daxa_u32*)(GCS.task_head.manifold_color);
  if (manifold_color[m] != GCS.color) return;
  Manifold* manifold = &((Manifold*)(GCS.task_head.collisions))[m];
  if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index) return;
  RigidBody* rigid_bodies = (RigidBody*)(GCS.task_head.rigid_bodies);
  RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
  RigidBody* r2 = &rigid_bodies[manifold->obb2_index];
  if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2)) return; // sleeping/static pair: at rest
  daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
  daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
  daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
  // DIAG stage 5: runaway accumulator BUILT this frame by the CS sweep (seen entering relax)
  for (daxa_i32 ci = 0; ci < manifold->contact_count; ++ci)
    bb_dbg_impulse_probe(sim_config, 5u, manifold->obb1_index, manifold->obb2_index,
                         manifold->contacts[ci].normal_impulse, manifold->contacts[ci].penetration, manifold->normal.y);
  if (sim_config->solver_type == SimSolverType::PGS_SOFT)
    PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, false, inv_dt);
}

// ---- overflow bucket (mirrors Box2D's serial overflow set) ----
// Solves the manifolds the per-color dispatches skip: uncolored after the coloring rounds, or
// color >= BB_MAX_COLORS_SOLVE. Those manifolds may share bodies among themselves, so a single
// thread solves them sequentially. Free on normal frames (graph_color_overflow == 0 -> early out);
// only degenerate frames (e.g. a velocity-exploded body touching hundreds) pay the serial cost.

// true if the per-color dispatches do NOT cover this manifold
bool gc_overflowed(daxa_u32 c)
{
  return c == MAX_U32 || c >= BB_MAX_COLORS_SOLVE;
}

[shader("compute")]
[numthreads(1, 1, 1)] void entry_collision_pre_solver_overflow(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  if (compute_i.x >= 1u) return;
  SimConfig* sim_config = (SimConfig*)(GCS.task_head.sim_config);
  if (sim_config->graph_color_overflow == 0u) return;
  let coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  daxa_u32* manifold_color = (daxa_u32*)(GCS.task_head.manifold_color);
  RigidBody* rigid_bodies = (RigidBody*)(GCS.task_head.rigid_bodies);
  daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
  daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
  daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
  daxa::f32 hertz = min(MIN_CONTACT_HERTZ, inv_dt * 0.3333333333333333f);
  for (daxa_u32 m = 0u; m < coll; ++m)
  {
    if (!gc_overflowed(manifold_color[m])) continue;
    Manifold* manifold = &((Manifold*)(GCS.task_head.collisions))[m];
    if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index) continue;
    RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
    RigidBody* r2 = &rigid_bodies[manifold->obb2_index];
    if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2)) continue; // sleeping/static pair: at rest
    if (sim_config->solver_type == SimSolverType::PGS_SOFT)
      PGS_SOFT.pre_step(manifold, inv_dt, r1, r2, accum_impulse, friction, sim_config->dt, hertz);
  }
}

[shader("compute")]
[numthreads(1, 1, 1)] void entry_collision_solver_overflow(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  if (compute_i.x >= 1u) return;
  SimConfig* sim_config = (SimConfig*)(GCS.task_head.sim_config);
  if (sim_config->graph_color_overflow == 0u) return;
  let coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  daxa_u32* manifold_color = (daxa_u32*)(GCS.task_head.manifold_color);
  RigidBody* rigid_bodies = (RigidBody*)(GCS.task_head.rigid_bodies);
  daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
  daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
  daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
  for (daxa_u32 m = 0u; m < coll; ++m)
  {
    if (!gc_overflowed(manifold_color[m])) continue;
    Manifold* manifold = &((Manifold*)(GCS.task_head.collisions))[m];
    if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index) continue;
    RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
    RigidBody* r2 = &rigid_bodies[manifold->obb2_index];
    if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2)) continue; // sleeping/static pair: at rest
    if (sim_config->solver_type == SimSolverType::PGS_SOFT)
      PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, true, inv_dt);
  }
}

[shader("compute")]
[numthreads(1, 1, 1)] void entry_collision_solver_relax_overflow(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  if (compute_i.x >= 1u) return;
  SimConfig* sim_config = (SimConfig*)(GCS.task_head.sim_config);
  if (sim_config->graph_color_overflow == 0u) return;
  let coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  daxa_u32* manifold_color = (daxa_u32*)(GCS.task_head.manifold_color);
  RigidBody* rigid_bodies = (RigidBody*)(GCS.task_head.rigid_bodies);
  daxa::b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
  daxa::b32 friction = sim_config->has_flag(SimFlag::FRICTION);
  daxa::f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
  for (daxa_u32 m = 0u; m < coll; ++m)
  {
    if (!gc_overflowed(manifold_color[m])) continue;
    Manifold* manifold = &((Manifold*)(GCS.task_head.collisions))[m];
    if (manifold->obb1_index >= sim_config->rigid_body_count || manifold->obb2_index >= sim_config->rigid_body_count || manifold->obb1_index == manifold->obb2_index) continue;
    RigidBody* r1 = &rigid_bodies[manifold->obb1_index];
    RigidBody* r2 = &rigid_bodies[manifold->obb2_index];
    if (!rb_is_awake_dynamic(r1) && !rb_is_awake_dynamic(r2)) continue; // sleeping/static pair: at rest
    if (sim_config->solver_type == SimSolverType::PGS_SOFT)
      PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, false, inv_dt);
  }
}

const static daxa_u32 LINE_POINT_COUNT = 2;
const static daxa::f32 LINE_SIZE = 0.5f;

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_create_contact_points(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(CP.task_head.sim_config);

  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (compute_i.x >= collision_count)
  {
    return;
  }

  Manifold* manifolds = (Manifold*)(CP.task_head.collisions);
  Manifold* m = &manifolds[compute_i.x];

  daxa_u32 contact_count = m->contact_count > 0 ? (daxa_u32)m->contact_count : 0u;
  if(contact_count == 0u)
  {
    return;
  }

  daxa_u32 index = 0;
  InterlockedAdd(sim_config->g_c_info.collision_point_count, contact_count, index);
  if(index >= BB_MAX_DEBUG_CONTACT_POINT_COUNT)
  {
    return;
  }
  contact_count = min(contact_count, BB_MAX_DEBUG_CONTACT_POINT_COUNT - index);

  if (sim_config->has_flag(SimFlag::DEBUG_INFO))
  {
    var vertex_buffer = (GUIVertex*)(CP.task_head.vertex_buffer);
    var line_vertex_buffer = (GUIVertexLine*)(CP.task_head.line_vertex_buffer);

    // contact tint: by graph color (debug) or by warm-start
    bool by_graph = sim_config->has_flag(SimFlag::DEBUG_GRAPH_COLORS);
    daxa::f32vec3 gcol = daxa::f32vec3(1.0f, 1.0f, 1.0f); // uncolored/overflow => white
    if (by_graph)
    {
      daxa_u32* manifold_color = (daxa_u32*)(CP.task_head.manifold_color);
      daxa_u32 gc = manifold_color[compute_i.x];
      if (gc < BB_MAX_COLORS)
      {
        // golden-ratio hue scramble: adjacent graph colors land far apart on the hue wheel,
        // so the ~8 colors actually used stay clearly distinguishable
        daxa::f32 h = frac((daxa::f32)gc * 0.6180339887f) * 6.0f;
        gcol = clamp(daxa::f32vec3(abs(h - 3.0f) - 1.0f, 2.0f - abs(h - 2.0f), 2.0f - abs(h - 4.0f)), 0.0f, 1.0f);
      }
    }

    for (daxa_u32 i = 0; i < contact_count; i++)
    {
      daxa_u32 current_index = index + i;
      vertex_buffer[current_index].position = m->contacts[i].position;
      if (by_graph)
      {
        vertex_buffer[current_index].color = gcol;
      }
      else
      {
        daxa::f32 blue = (daxa::f32)(255 - m->contacts[i].warm_start) / 255.0f;
        daxa::f32 red = 1.0f - blue;
        vertex_buffer[current_index].color = daxa::f32vec3(red, 0.0f, blue);
      }

      line_vertex_buffer[current_index * LINE_POINT_COUNT].position = m->contacts[i].position;
      line_vertex_buffer[current_index * LINE_POINT_COUNT + 1].position = m->contacts[i].position + m->normal * LINE_SIZE;
    }
  }
}

const static daxa_u32 AXES_POINT_COUNT = 6;

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_update_rigid_bodies(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(RB_update.task_head.sim_config);

  if (compute_i.x >= sim_config->rigid_body_count)
  {
    return;
  }

  daxa_u32 index = compute_i.x;

  // FIXME: split between static and dynamic objects?
  RigidBody* rigid_bodies = (RigidBody*)(RB_update.task_head.rigid_bodies);
  RigidBody* rigid_bodies_update = (RigidBody*)(RB_update.task_head.rigid_bodies_update);
  RigidBody* r = &rigid_bodies[index];
  RigidBody* r_copy = &rigid_bodies_update[index];

  // DIAG: row-permutation checksum. Sum of ids over all rows is constant (n*(n-1)/2) unless the
  // sort/reorder duplicated one body row and dropped another (destroying a body).
  daxa_u32 _prev;
  InterlockedAdd(sim_config->dbg_id_sum, r->id, _prev);
  // DIAG probe stage 3 (end of frame, after the CSR relax sweep).
  if (r->has_flag(RigidBodyFlag::DYNAMIC))
  {
    bb_dbg_velocity_probe(sim_config, 3u, index, r->velocity, r->position.y);
  }

  copy_rigid_body_fields(r_copy, r);
  r_copy->island_index = MAX_U32;
  r_copy->active_index = MAX_U32;
  r_copy->manifold_node_index = MAX_U32;

  if (sim_config->has_flag(SimFlag::DEBUG_INFO))
  {
    daxa::f32vec3 _min = r->minimum;
    daxa::f32vec3 _max = r->maximum;
    daxa::f32vec3 center = (_min + _max) * 0.5f;
    var axes_vertex_buffer = (GUIVertexLine*)(RB_update.task_head.axes_vertex_buffer);
    let x_axis = r->rotate_vector(daxa::f32vec3(1.0f, 0.0f, 0.0f));
    let y_axis = r->rotate_vector(daxa::f32vec3(0.0f, 1.0f, 0.0f));
    let z_axis = r->rotate_vector(daxa::f32vec3(0.0f, 0.0f, 1.0f));

    center = r->object_to_world(center);

    axes_vertex_buffer[index * AXES_POINT_COUNT].position = center;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 1].position = center + x_axis;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 2].position = center;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 3].position = center + y_axis;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 4].position = center;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 5].position = center + z_axis;
  }

#if defined(BB_DEBUG)
  // printf("Updating rigid body %d sim frame: %lu c frame: %lu\n", index, r.frame_count, r_copy.frame_count);
#endif // BB_DEBUG
}
