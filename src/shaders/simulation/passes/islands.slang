#include "../islands.slang"
#include "state_copy.slang"

[[vk::push_constant]] IslandCounterPushConstants IC;
[[vk::push_constant]] IslandBuilderPushConstants IB;
[[vk::push_constant]] IslandPrefixSumPushConstants IPS;
[[vk::push_constant]] IslandBuilderBodyLink2IslandPushConstants BLI;
[[vk::push_constant]] IslandBuilderSortBodyLinkInIslandPushConstants SBLI;
[[vk::push_constant]] ManifoldIslandBuilderPushConstants MIB;
[[vk::push_constant]] ContactIslandGatherPushConstants CIG;
[[vk::push_constant]] ManifoldIslandPrefixSumPushConstants MIPS;
[[vk::push_constant]] IslandBuilderManifoldLink2IslandPushConstants MLI;
[[vk::push_constant]] IslandBuilderSortManifoldLinkInIslandPushConstants MBLI;

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
