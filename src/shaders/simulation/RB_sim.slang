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

  let morton_code = MortonCodeGenerator.morton3D(rigid_bodies[index].position);

  morton_codes[index] = MortonCode(morton_code, index);
}

#if defined(BB_RADIX_SORT_HISTOGRAM)
// groupshared variables
groupshared Atomic<daxa::u32> histogram[RADIX_SORT_BINS];

// Entry point with HLSL semantics
[numthreads(RADIX_SORT_WORKGROUP_SIZE, 1, 1)] void entry_radix_sort_histogram(daxa::u32vec3 compute_i : SV_DispatchThreadID, daxa::u32vec3 GroupThreadID : SV_GroupThreadID, daxa::u32vec3 GroupID : SV_GroupID)
{
  let lID = GroupThreadID.x;
  let wID = GroupID.x;

  var global_histograms =
      (daxa::u32*)(RBRSH.task_head.global_histograms);

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

  daxa::u32* global_histograms =
      (daxa::u32*)(SRS.task_head.global_histograms);

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
    let r = rigid_bodies[morton_code.index];
    let rb_aabb = r.get_AABB_from_OBB();
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
    lbvh_construction_info[child_a] = LBVHConstructionInfo(index, 0);
    lbvh_construction_info[child_b] = LBVHConstructionInfo(index, 0);
  }

  // root node
  if(index == 0) {
    lbvh_construction_info[0] = LBVHConstructionInfo(0, 0);
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

  daxa::u32 node_index = lbvh_construction_info[leaf_offset + index].parent;
  while(true) {
    var visitations = 0U;
    InterlockedAdd(lbvh_construction_info[node_index].visitation_count, 1U, visitations);
    if(visitations < 1) {
      // this is the first time we visit this node so it's finished
      break;
    }

    var bvh_node = lbvh_nodes[node_index];
    // FIXME: use relative indices instead of absolute indices
    let child_a = lbvh_nodes[bvh_node.left];
    let child_b = lbvh_nodes[bvh_node.right]; 

    let aabb = Aabb::merge(child_a.aabb, child_b.aabb);
    lbvh_nodes[node_index].aabb = aabb;
    if(node_index == 0) {
      break;
    }
    node_index = lbvh_construction_info[node_index].parent;
  }
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_rigid_body_reordering(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(RBR.task_head.sim_config);

  daxa::u32 index = compute_i.x;
  daxa::u32 rb_count = sim_config->rigid_body_count;
  if (index >= rb_count)
  {
    return;
  }

  let rigid_bodies =
      (RigidBody*)(RBR.task_head.rigid_bodies);
  var lbvh_nodes = (LBVHNode*)(RBR.task_head.lbvh_nodes);
  let morton_codes =
      (MortonCode*)(RBR.task_head.morton_codes);
  var active_rigid_bodies =
      (ActiveRigidBody*)(RBR.task_head.active_rigid_bodies);
  var rigid_body_map =
      (RigidBodyEntry*)(RBR.task_head.rigid_body_map);
  var sorted_rigid_bodies = (RigidBody*)(RBR.task_head.rigid_body_sorted);

  // get the morton code rigid body index
  let rb_index = morton_codes[index].index;
  // get the rigid body id
  let rb_id = rigid_bodies[rb_index].id;

  rigid_body_map[rb_id] = RigidBodyEntry(index);
  
  // // put the rigid body in the sorted array by morton codes
  sorted_rigid_bodies[index] = rigid_bodies[rb_index];
  // // update the lbvh node
  // lbvh_nodes[rb_count - 1 + index].rigid_body_index = index;
  // printf("RBR index %d, id: %d\n", index, rb_id);
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

  ActiveRigidBody active_rigid_body = RBL.task_head.active_rigid_bodies[index];

  RigidBody* rigid_bodies =
      (RigidBody*)(RBL.task_head.rigid_bodies);

  rigid_bodies[active_rigid_body.rigid_body_index].active_index = index;

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

  // let active_rigid_body = BP.task_head.active_rigid_bodies[index];
  // let rb_index = active_rigid_body.rigid_body_index;
  // let rigid_body = BP.task_head.rigid_bodies[rb_index];
  let rigid_body = BP.task_head.rigid_bodies[index];
  let lbvh_nodes = (LBVHNode*)(BP.task_head.lbvh_nodes);
  var bp_collisions =
      (BroadPhaseCollision*)(BP.task_head.broad_phase_collisions);
  
  Aabb aabb = rigid_body.get_AABB_from_OBB();

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
      (SimConfig*)(RBD.task_head.sim_config);

  DispatchBuffer* dispatch_buffer =
      (DispatchBuffer*)(RBD.task_head.dispatch_buffer);

  let np_count = (sim_config->broad_phase_collision_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
  dispatch_buffer->narrow_phase_dispatch = daxa_u32vec3(np_count, 1, 1);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_narrow_phase(daxa::u32vec3 compute_i : SV_DispatchThreadID)
{
  var sim_config =
      (SimConfig*)(NP.task_head.sim_config);

  daxa::u32 index = compute_i.x;
  daxa::u32 collision_count = sim_config->broad_phase_collision_count;
  if (index >= collision_count)
  {
    return;
  }

  let previous_sim_config =
      *NP.task_head.previous_sim_config;

  var rigid_bodies =
      (RigidBody*)(NP.task_head.rigid_bodies);

  var previous_rigid_bodies =
      (RigidBody*)(NP.task_head.previous_rigid_bodies);

  var rigid_body_link_manifolds =
      (ManifoldNode*)(NP.task_head.rigid_body_link_manifolds);

  var collisions =
      (Manifold*)(NP.task_head.collisions);

  var previous_rigid_body_link_manifolds =
      (ManifoldNode*)(NP.task_head.previous_rigid_body_link_manifolds);

  var old_collisions =
      (Manifold*)(NP.task_head.old_collisions);

  var body_links =
      (BodyLink*)(NP.task_head.scratch_body_links);
    
  var bp_collision =
      (BroadPhaseCollision)(NP.task_head.broad_phase_collisions[index]);

  narrow_phase(bp_collision, rigid_bodies, sim_config, previous_sim_config, rigid_body_link_manifolds, collisions, previous_rigid_bodies, previous_rigid_body_link_manifolds, old_collisions, body_links);

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

      RigidBody* r =
          (RigidBody*)(&RB_sim.task_head.rigid_bodies[index]);

      RigidBody* p_r =
          (RigidBody*)(&RB_sim.task_head.previous_rigid_bodies[index]);

      if (sim_config->has_flag(SimFlag::ADVECTION))
      {
        r->advect(sim_config);
      }

      p_r->manifold_node_index = MAX_U32;
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

  var collision_c = (sim_config->g_c_info.collision_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
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

      BodyLink* body_links =
          (BodyLink*)(IB.task_head.scratch_body_links);

      Island* islands =
          (Island*)(IB.task_head.islands);

      ActiveRigidBody* active_rigid_bodies =
          (ActiveRigidBody*)(IB.task_head.active_rigid_bodies);

      RigidBody* rigid_bodies =
          (RigidBody*)(IB.task_head.rigid_bodies);

      IslandBuilder builder = IslandBuilder(sim_config->active_rigid_body_count, body_links, active_rigid_bodies, sim_config->island_count, islands, sim_config->rigid_body_count, rigid_bodies);

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

  builder.sort_body_links_in_island(index);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_manifold_island_builder(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      SimConfig* sim_config =
          (SimConfig*)(MIB.task_head.sim_config);

      daxa_u32 index = compute_i.x;

      if (index >= sim_config->g_c_info.collision_count)
      {
        return;
      }

      Manifold* m =
          &MIB.task_head.collisions[index];

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

      var ci_count = (sim_config->contact_island_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
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
  if (contact_collision_count != sim_config->g_c_info.collision_count)
  {
    printf("Manifold prefix sum failed, contact_collision_count: %d, collision_count: %d\n", contact_collision_count, sim_config->g_c_info.collision_count);
  }
#endif // BB_DEBUG
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_manifold_link_to_island(daxa_u32vec3 compute_i : SV_DispatchThreadID) {
      SimConfig* sim_config =
          (SimConfig*)(MLI.task_head.sim_config);

      daxa_u32 index = compute_i.x;
      if (index >= sim_config->g_c_info.collision_count)
      {
        return;
      }

      Manifold* m =
          &MLI.task_head.collisions[index];

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

      ContactIslandBuilder builder = ContactIslandBuilder(sim_config->active_rigid_body_count, body_links, sim_config->island_count, islands, &sim_config->contact_island_count, contact_islands, sim_config->g_c_info.collision_count, manifold_links);

      builder.manifold_link_to_island(index, a_active_body_index, b_active_body_index);
    }

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_sort_manifold_links_in_island(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(MBLI.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  if (index >= sim_config->contact_island_count)
  {
    return;
  }

  ManifoldLinkIsland* manifold_links =
      (ManifoldLinkIsland*)(MBLI.task_head.manifold_links);

  ContactIsland* islands =
      (ContactIsland*)(MBLI.task_head.contact_islands);

  ContactIslandBuilder builder = ContactIslandBuilder(&sim_config->contact_island_count, islands, sim_config->g_c_info.collision_count, manifold_links);

  builder.sort_manifold_links_in_island(index);
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
  if (index >= sim_config->contact_island_count)
#endif // BB_SIM_DEBUG
      {
        return;
      }

#if defined(BB_SIM_DEBUG)
      for (daxa_u32 manifold_index = 0; manifold_index < sim_config->g_c_info.collision_count; ++manifold_index)
      {
#else
  ContactIsland* islands =
      (ContactIsland*)(CPS.task_head.contact_islands);

  ManifoldLinkIsland* manifold_links =
      (ManifoldLinkIsland*)(CPS.task_head.manifold_links);

  daxa_u32 start_index = islands[index].start_index;
  daxa_u32 max_count = islands[index].count;

  if (start_index + max_count > sim_config->g_c_info.collision_count || max_count == 0)
  {
#if defined(BB_DEBUG)
    printf("Collision pre solver failed index %d, start_index %d, max_count %d\n", index, start_index, max_count);
#endif // BB_DEBUG
    return;
  }

  for (daxa_u32 m = start_index; m < start_index + max_count; ++m)
  {

    daxa_u32 manifold_index = manifold_links[m].manifold_index;
#endif // BB_SIM_DEBUG

        Manifold manifold =
            CPS.task_head.collisions[manifold_index];

        if (manifold.obb1_index >= sim_config->rigid_body_count || manifold.obb2_index >= sim_config->rigid_body_count || manifold.obb1_index == manifold.obb2_index)
        {
          continue;
        }

        RigidBody r1 = CPS.task_head.rigid_bodies[manifold.obb1_index];
        RigidBody r2 = CPS.task_head.rigid_bodies[manifold.obb2_index];

        daxa_b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
        daxa_b32 friction = sim_config->has_flag(SimFlag::FRICTION);
        daxa_f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
        daxa_f32 hertz = min(MIN_CONTACT_HERTZ, inv_dt * 0.3333333333333333f);

        if (sim_config->solver_type == SimSolverType::PGS_SOFT)
        {
          PGS_SOFT.pre_step(manifold, inv_dt, r1, r2, accum_impulse, friction, sim_config->dt, hertz);
        }
        else if (sim_config->solver_type == SimSolverType::PGS)
        {
          PGS.pre_step(manifold, inv_dt, r1, r2, accum_impulse, friction);
        }

        CPS.task_head.collisions[manifold_index] = manifold;
        CPS.task_head.rigid_bodies[manifold.obb1_index] = r1;
        CPS.task_head.rigid_bodies[manifold.obb2_index] = r2;
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
  if (index >= sim_config->contact_island_count)
#endif // BB_SIM_DEBUG
  {
    return;
  }

#if defined(BB_SIM_DEBUG)
  for (daxa_u32 manifold_index = 0; manifold_index < sim_config->g_c_info.collision_count; ++manifold_index)
  {
#else
  ContactIsland* islands =
      (ContactIsland*)(CPS.task_head.contact_islands);

  ManifoldLinkIsland* manifold_links =
      (ManifoldLinkIsland*)(CPS.task_head.manifold_links);

  daxa_u32 start_index = islands[index].start_index;
  daxa_u32 max_count = islands[index].count;

  if (start_index + max_count > sim_config->g_c_info.collision_count || max_count == 0)
  {
#if defined(BB_DEBUG)
    printf("Collision solver failed index %d, start_index %d, max_count %d\n", index, start_index, max_count);
#endif // BB_DEBUG
    return;
  }

  for (daxa_u32 m = start_index; m < start_index + max_count; ++m)
  {

    daxa_u32 manifold_index = manifold_links[m].manifold_index;
#endif // BB_SIM_DEBUG

    Manifold manifold =
        CS.task_head.collisions[manifold_index];

    if (manifold.obb1_index >= sim_config->rigid_body_count || manifold.obb2_index >= sim_config->rigid_body_count || manifold.obb1_index == manifold.obb2_index)
    {
      continue;
    }

    // printf("Solving collision %d, rigid_body_a %d, rigid_body_b %d cc: %d\n", m, manifold.obb1_index, manifold.obb2_index, manifold.contact_count);

    RigidBody r1 = CS.task_head.rigid_bodies[manifold.obb1_index];
    RigidBody r2 = CS.task_head.rigid_bodies[manifold.obb2_index];

    daxa_b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
    daxa_b32 friction = sim_config->has_flag(SimFlag::FRICTION);
    daxa_f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;

    // Solve the collision
    if (sim_config->solver_type == SimSolverType::PGS_SOFT)
    {
      PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, true, inv_dt);
    }
    else if (sim_config->solver_type == SimSolverType::PGS)
    {
      PGS.solve(manifold, r1, r2, accum_impulse, friction);
    }

    CS.task_head.collisions[manifold_index] = manifold;
    CS.task_head.rigid_bodies[manifold.obb1_index] = r1;
    CS.task_head.rigid_bodies[manifold.obb2_index] = r2;
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

      RigidBody* r =
          (RigidBody*)(&RB_integrate.task_head.rigid_bodies[index]);

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
  if (index >= sim_config->contact_island_count)
#endif // BB_SIM_DEBUG
  {
    return;
  }

#if defined(BB_SIM_DEBUG)
  for (daxa_u32 manifold_index = 0; manifold_index < sim_config->g_c_info.collision_count; ++manifold_index)
  {
#else
  ContactIsland* islands =
      (ContactIsland*)(CPS.task_head.contact_islands);

  ManifoldLinkIsland* manifold_links =
      (ManifoldLinkIsland*)(CPS.task_head.manifold_links);

  daxa_u32 start_index = islands[index].start_index;
  daxa_u32 max_count = islands[index].count;

  if (start_index + max_count > sim_config->g_c_info.collision_count || max_count == 0)
  {
#if defined(BB_DEBUG)
    printf("Collision solver relaxation failed index %d, start_index %d, max_count %d\n", index, start_index, max_count);
#endif // BB_DEBUG
    return;
  }

  for (daxa_u32 m = start_index; m < start_index + max_count; ++m)
  {

    daxa_u32 manifold_index = manifold_links[m].manifold_index;
#endif // BB_SIM_DEBUG

    Manifold manifold =
        CSR.task_head.collisions[manifold_index];

    if (manifold.obb1_index >= sim_config->rigid_body_count || manifold.obb2_index >= sim_config->rigid_body_count || manifold.obb1_index == manifold.obb2_index)
    {
      continue;
    }

    RigidBody r1 = CSR.task_head.rigid_bodies[manifold.obb1_index];
    RigidBody r2 = CSR.task_head.rigid_bodies[manifold.obb2_index];

    daxa_b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
    daxa_b32 friction = sim_config->has_flag(SimFlag::FRICTION);
    daxa_f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;

    // Solve the collision with relaxation
    if (sim_config->solver_type == SimSolverType::PGS_SOFT)
    {
      PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, false, inv_dt);
    }

    CSR.task_head.collisions[manifold_index] = manifold;
    CSR.task_head.rigid_bodies[manifold.obb1_index] = r1;
    CSR.task_head.rigid_bodies[manifold.obb2_index] = r2;
  }
}

const static daxa_u32 LINE_POINT_COUNT = 2;
const static daxa_f32 LINE_SIZE = 0.5f;

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_create_contact_points(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(CP.task_head.sim_config);

  if (compute_i.x >= sim_config->g_c_info.collision_count)
  {
    return;
  }

  Manifold m =
      CP.task_head.collisions[compute_i.x];

  daxa_u32 index = 0;
  InterlockedAdd(sim_config->g_c_info.collision_point_count, m.contact_count, index);

  if (sim_config->has_flag(SimFlag::DEBUG_INFO))
  {
    var vertex_buffer = (GUIVertex*)(CP.task_head.vertex_buffer);
    var line_vertex_buffer = (GUIVertexLine*)(CP.task_head.line_vertex_buffer);

    for (int i = 0; i < m.contact_count; i++)
    {
      daxa_u32 current_index = index + i;
      vertex_buffer[current_index].position = m.contacts[i].position;
      daxa_f32 blue = (daxa_f32)(255 - m.contacts[i].warm_start) / 255.0f;
      daxa_f32 red = 1.0f - blue;
      vertex_buffer[current_index].color = daxa_f32vec3(red, 0.0f, blue);

      line_vertex_buffer[current_index * LINE_POINT_COUNT].position = m.contacts[i].position;
      line_vertex_buffer[current_index * LINE_POINT_COUNT + 1].position = m.contacts[i].position + m.normal * LINE_SIZE;
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

  RigidBody r = RB_update.task_head.rigid_bodies[index];
  RigidBody r_copy = RB_update.task_head.rigid_bodies_update[index];

  r_copy.island_index = MAX_U32;
  r_copy.active_index = MAX_U32;
  r_copy.position = r.position;
  r_copy.rotation = r.rotation;
  r_copy.velocity = r.velocity;
  r_copy.omega = r.omega;
  r_copy.flags = r.flags;
#if defined(BB_DEBUG)
  r_copy.face_collided = r.face_collided;
#endif // BB_DEBUG

  if (sim_config->has_flag(SimFlag::DEBUG_INFO))
  {
    daxa_f32vec3 _min = r.minimum;
    daxa_f32vec3 _max = r.maximum;
    daxa_f32vec3 center = (_min + _max) * 0.5f;
    var axes_vertex_buffer = (GUIVertexLine*)(RB_update.task_head.axes_vertex_buffer);
    let x_axis = r.rotate_vector(daxa_f32vec3(1.0f, 0.0f, 0.0f));
    let y_axis = r.rotate_vector(daxa_f32vec3(0.0f, 1.0f, 0.0f));
    let z_axis = r.rotate_vector(daxa_f32vec3(0.0f, 0.0f, 1.0f));

    center = r.object_to_world(center);

    axes_vertex_buffer[index * AXES_POINT_COUNT].position = center;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 1].position = center + x_axis;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 2].position = center;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 3].position = center + y_axis;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 4].position = center;
    axes_vertex_buffer[index * AXES_POINT_COUNT + 5].position = center + z_axis;
  }

  RB_update.task_head.rigid_bodies_update[index] = r_copy;

#if defined(BB_DEBUG)
  // printf("Updating rigid body %d sim frame: %lu c frame: %lu\n", index, r.frame_count, r_copy.frame_count);
#endif // BB_DEBUG
}