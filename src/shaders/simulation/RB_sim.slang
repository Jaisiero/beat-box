// #include "extensions.slang"
#include "collision_detection.slang"
#include "solvers.slang"

[[vk::push_constant]] ResetBodyLinkPushConstants RBL;
[[vk::push_constant]] BroadPhasePushConstants BP;
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
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_reset_body_links(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(RBL.task_head.sim_config);
    
  daxa_u32 index = pixel_i.x;
  if (index >= sim_config->active_rigid_body_count)
  {
    return;
  }

  ActiveRigidBody active_rigid_body = RBL.task_head.active_rigid_bodies[index];

  Ptr<RigidBody> rigid_bodies =
      Ptr<RigidBody>(RBL.task_head.rigid_bodies);

  rigid_bodies[active_rigid_body.rigid_body_index].active_index = index;

#if defined(BB_DEBUG)
  // printf("RBL index %d, rigid_body_index %d\n", index, active_rigid_body.rigid_body_index);
#endif // BB_DEBUG

  RBL.task_head.scratch_body_links[index] = BodyLink(index, MAX_U32);
}


[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_broad_phase(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(BP.task_head.sim_config);
    
  daxa_u32 index = pixel_i.x;
  if (index >= sim_config->rigid_body_count)
  {
    return;
  }
  
  let previous_sim_config =
      *BP.task_head.previous_sim_config;

  Ptr<RigidBody> rigid_bodies =
      Ptr<RigidBody>(BP.task_head.rigid_bodies);

  Ptr<Manifold> collisions =
      Ptr<Manifold>(BP.task_head.collisions);

  Ptr<Manifold> old_collisions =
      Ptr<Manifold>(BP.task_head.old_collisions);

  Ptr<BodyLink> body_links =
      Ptr<BodyLink>(BP.task_head.scratch_body_links);

  broad_phase(index, rigid_bodies, sim_config, previous_sim_config, collisions, old_collisions, body_links);

#if defined(BB_DEBUG)
  // if(index == 0)
  // {
  //   printf("Broad phase frame %lu\n", sim_config->frame_count);
  // }
#endif // BB_DEBUG
}


[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] 
    void entry_rigid_body_sim(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(RB_sim.task_head.sim_config);

  if (pixel_i.x >= sim_config->rigid_body_count)
  {
    return;
  }

  Ptr<RigidBody> r =
      Ptr<RigidBody>(&RB_sim.task_head.rigid_bodies[pixel_i.x]);

  if(sim_config->has_flag(SimFlag::ADVECTION))
  {
    r->advect(sim_config);
  }
}




[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_island_counter(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(IC.task_head.sim_config);
    
  daxa_u32 index = pixel_i.x;
  if (index >= sim_config->active_rigid_body_count)
  {
    return;
  }

  Ptr<BodyLink> body_links =
      Ptr<BodyLink>(IC.task_head.scratch_body_links);

  Ptr<Island> islands =
      Ptr<Island>(IC.task_head.islands);

#if defined(BB_DEBUG)
  // printf("IC index %d, body_link_index %d\n", index, body_link_index);
#endif // BB_DEBUG

  IslandBuilder builder = IslandBuilder(body_links, islands);

  builder.add_island(&sim_config->island_count, index);
}

[shader("compute")]
    [numthreads(1, 1, 1)] void
    entry_island_dispatcher(uint3 pixel_i : SV_DispatchThreadID)
{

  if (pixel_i.x >= 1u)
  {
    return;
  }

  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(ID.task_head.sim_config);

  Ptr<DispatchBuffer> dispatch_buffer =
      Ptr<DispatchBuffer>(ID.task_head.dispatch_buffer);

  dispatch_buffer->island_dispatch.x = (sim_config->island_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
}

[shader("compute")]
    [numthreads(1, 1, 1)] void
    entry_collision_solver_dispatcher(uint3 pixel_i : SV_DispatchThreadID)
{

  if (pixel_i.x >= 1u)
  {
    return;
  }

  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(CSD.task_head.sim_config);

  Ptr<DispatchBuffer> dispatch_buffer =
      Ptr<DispatchBuffer>(CSD.task_head.dispatch_buffer);

  dispatch_buffer->collision_dispatch.x = (sim_config->g_c_info.collision_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_island_builder(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(IB.task_head.sim_config);
    
  daxa_u32 index = pixel_i.x;
  if (index >= sim_config->active_rigid_body_count)
  {
    return;
  }

  Ptr<BodyLink> body_links =
      Ptr<BodyLink>(IB.task_head.scratch_body_links);

  Ptr<Island> islands =
      Ptr<Island>(IB.task_head.islands);

  IslandBuilder builder = IslandBuilder(sim_config->active_rigid_body_count, body_links, sim_config->island_count, islands);

  builder.expand_island(index);
}

[shader("compute")]
    [numthreads(1, 1, 1)] void
    entry_island_prefix_sum(uint3 pixel_i : SV_DispatchThreadID)
{

  if (pixel_i.x >= 1u)
  {
    return;
  }

  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(IPS.task_head.sim_config);
      

  Ptr<Island> islands =
      Ptr<Island>(IPS.task_head.islands);

  IslandBuilder builder = IslandBuilder(sim_config->island_count, islands);

  builder.prefix_sum_islands();
}


[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_body_link_to_island(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(BLI.task_head.sim_config);
    
  daxa_u32 index = pixel_i.x;
  if (index >= sim_config->active_rigid_body_count)
  {
    return;
  }

  Ptr<BodyLink> scratch_body_links =
      Ptr<BodyLink>(BLI.task_head.scratch_body_links);

  Ptr<BodyLinkIsland> body_link_island =
      Ptr<BodyLinkIsland>(BLI.task_head.body_links);

  Ptr<Island> islands =
      Ptr<Island>(BLI.task_head.islands);

  IslandBuilder builder = IslandBuilder(sim_config->active_rigid_body_count, scratch_body_links, sim_config->island_count, islands, body_link_island);

  builder.body_link_to_island(index);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_sort_body_links_in_island(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(SBLI.task_head.sim_config);
    
  daxa_u32 index = pixel_i.x;
  if (index >= sim_config->island_count)
  {
    return;
  }

  Ptr<BodyLinkIsland> body_link_island =
      Ptr<BodyLinkIsland>(SBLI.task_head.body_links);

  Ptr<Island> islands =
      Ptr<Island>(SBLI.task_head.islands);
  
  IslandBuilder builder = IslandBuilder(sim_config->island_count, islands, sim_config->active_rigid_body_count, body_link_island);

  builder.sort_body_links_in_island(index);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_manifold_island_builder(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(MIB.task_head.sim_config);

  daxa_u32 index = pixel_i.x;

  if (index >= sim_config->g_c_info.collision_count)
  {
    return;
  }

  Ptr<Manifold> m =
      &MIB.task_head.collisions[index];

  daxa_u32 body_a_index = m->obb1_index;
  daxa_u32 body_b_index = m->obb2_index;

  if(body_a_index >= sim_config->rigid_body_count || body_b_index >= sim_config->rigid_body_count || body_a_index == body_b_index)
  {
#if defined(BB_DEBUG)
    // printf("Manifold index %d, body_a_index %d, body_b_index %d\n", index, body_a_index, body_b_index);
#endif // BB_DEBUG
    return;
  }

  Ptr<RigidBody> rigid_bodies =
      Ptr<RigidBody>(MIB.task_head.rigid_bodies);

  daxa_u32 a_active_body_index = rigid_bodies[body_a_index].active_index;
  daxa_u32 b_active_body_index = rigid_bodies[body_b_index].active_index;
  
#if defined(BB_DEBUG)
  // printf("Manifold index %d, body_a_index %d (active_idx: %d), body_b_index %d (active_idx: %d), manifold count: %d, active_count: %d, island_count: %d\n", index, body_a_index, a_active_body_index, body_b_index, b_active_body_index, sim_config->g_c_info.collision_count, sim_config->active_rigid_body_count, sim_config->island_count);
#endif // BB_DEBUG

  daxa_u32 min_body_index = min(a_active_body_index, b_active_body_index);

  Ptr<Island> islands =
      Ptr<Island>(MIB.task_head.islands);
  
  Ptr<BodyLink> body_links =
      Ptr<BodyLink>(MIB.task_head.scratch_body_links);

  IslandBuilder builder = IslandBuilder(sim_config->active_rigid_body_count, body_links, sim_config->island_count, islands);

  builder.expand_manifold_island(min_body_index);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_contact_island_gather(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(CIG.task_head.sim_config);
    
  daxa_u32 index = pixel_i.x;
  if (index >= sim_config->island_count)
  {
    return;
  }

  Ptr<Island> islands =
      Ptr<Island>(CIG.task_head.islands);

  Ptr<ContactIsland> contact_islands =
      Ptr<ContactIsland>(CIG.task_head.contact_islands);
  
  ContactIslandBuilder builder = ContactIslandBuilder(sim_config->island_count, islands, &sim_config->contact_island_count, contact_islands);

  builder.gather_contact_islands(index);
}

[shader("compute")]
    [numthreads(1, 1, 1)] void
    entry_manifold_island_prefix_sum(uint3 pixel_i : SV_DispatchThreadID)
{

  if (pixel_i.x >= 1u)
  {
    return;
  }

  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(MIPS.task_head.sim_config);
      

  Ptr<Island> islands =
      Ptr<Island>(MIPS.task_head.islands);

  IslandBuilder builder = IslandBuilder(sim_config->island_count, islands);

  builder.manifold_prefix_sum_islands();
}


[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_manifold_link_to_island(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(MLI.task_head.sim_config);
    
  daxa_u32 index = pixel_i.x;
  if (index >= sim_config->g_c_info.collision_count)
  {
    return;
  }

  Ptr<Manifold> m =
      &MLI.task_head.collisions[index];

  daxa_u32 body_a_index = m->obb1_index;
  daxa_u32 body_b_index = m->obb2_index;

  if(body_a_index >= sim_config->rigid_body_count || body_b_index >= sim_config->rigid_body_count || body_a_index == body_b_index)
  {
#if defined(BB_DEBUG)
    // printf("Manifold link index %d, body_a_index %d, body_b_index %d\n", index, body_a_index, body_b_index);
#endif // BB_DEBUG
    return;
  }

  Ptr<RigidBody> rigid_bodies =
      Ptr<RigidBody>(MLI.task_head.rigid_bodies);

  daxa_u32 a_active_body_index = rigid_bodies[body_a_index].active_index;
  daxa_u32 b_active_body_index = rigid_bodies[body_b_index].active_index;

  daxa_u32 min_body_index = min(a_active_body_index, b_active_body_index);

  Ptr<Island> islands =
      Ptr<Island>(MLI.task_head.islands);

  Ptr<BodyLink> body_links =
      Ptr<BodyLink>(MLI.task_head.scratch_body_links);

  Ptr<ManifoldLinkIsland> manifold_links =
      Ptr<ManifoldLinkIsland>(MLI.task_head.manifold_links);

  IslandBuilder builder = IslandBuilder(sim_config->active_rigid_body_count, body_links, sim_config->island_count, islands, sim_config->g_c_info.collision_count, manifold_links);

  builder.manifold_link_to_island(index, a_active_body_index, b_active_body_index);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_sort_manifold_links_in_island(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(MBLI.task_head.sim_config);
    
  daxa_u32 index = pixel_i.x;
  if (index >= sim_config->island_count)
  {
    return;
  }

  Ptr<ManifoldLinkIsland> manifold_links =
      Ptr<ManifoldLinkIsland>(MBLI.task_head.manifold_links);

  Ptr<Island> islands =
      Ptr<Island>(MBLI.task_head.islands);
  
  IslandBuilder builder = IslandBuilder(sim_config->island_count, islands, sim_config->g_c_info.collision_count, manifold_links);

  // TODO: fix this
  // builder.sort_manifold_links_in_island(index);
}





[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_collision_pre_solver(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(CPS.task_head.sim_config);

  daxa_u32 index = pixel_i.x;
  if (index >= sim_config->island_count)
  {
    return;
  }

  Ptr<Island> islands =
      Ptr<Island>(CPS.task_head.islands);

  Ptr<ManifoldLinkIsland> manifold_links =
      Ptr<ManifoldLinkIsland>(CPS.task_head.manifold_links);

  // printf("Pre solver collision count %d, tmp_collision_count: %d\n", sim_config->g_c_info.collision_count, sim_config->g_c_info.tmp_collision_count);
  
  daxa_u32 start_index = islands[index].manifold_start_index;
  daxa_u32 max_count = islands[index].max_manifold_count;

  if(start_index + max_count > sim_config->g_c_info.collision_count || max_count == 0)
  {
#if defined(BB_DEBUG)
    daxa_u32 count = islands[index].manifold_count;
    // printf("Collision pre solver failed index %d, start_index %d, max_count %d, count: %d\n", index, start_index, max_count, count);
#endif // BB_DEBUG
    return;
  }
  
  // TODO: Implement islands
  for(daxa_u32 m = start_index; m < start_index + max_count; ++m)
  {

    daxa_u32 manifold_index = manifold_links[m].manifold_index;

    Manifold manifold =
      CPS.task_head.collisions[manifold_index];

    if(manifold.obb1_index >= sim_config->rigid_body_count || manifold.obb2_index >= sim_config->rigid_body_count || manifold.obb1_index == manifold.obb2_index)
    {
      continue;
    }

    RigidBody r1 = CPS.task_head.rigid_bodies[manifold.obb1_index];
    RigidBody r2 = CPS.task_head.rigid_bodies[manifold.obb2_index];

    daxa_b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
    daxa_b32 friction = sim_config->has_flag(SimFlag::FRICTION);
    daxa_f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
    daxa_f32 hertz = min(MIN_CONTACT_HERTZ, inv_dt * 0.3333333333333333f);

    if(sim_config->solver_type == SimSolverType::PGS_SOFT) {
      PGS_SOFT.pre_step(manifold, inv_dt, r1, r2, accum_impulse, friction, sim_config->dt, hertz);
    } else if(sim_config->solver_type == SimSolverType::PGS) {
      PGS.pre_step(manifold, inv_dt, r1, r2, accum_impulse, friction);
    }

    CPS.task_head.collisions[manifold_index] = manifold;
    CPS.task_head.rigid_bodies[manifold.obb1_index] = r1;
    CPS.task_head.rigid_bodies[manifold.obb2_index] = r2;
  }
}


[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_collision_solver(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(CS.task_head.sim_config);

  daxa_u32 index = pixel_i.x;
  if (index >= sim_config->island_count)
  {
    return;
  }

  Ptr<Island> islands =
      Ptr<Island>(CS.task_head.islands);

  Ptr<ManifoldLinkIsland> manifold_links =
      Ptr<ManifoldLinkIsland>(CS.task_head.manifold_links);
  
  daxa_u32 start_index = islands[index].manifold_start_index;
  daxa_u32 max_count = islands[index].max_manifold_count;

  if(start_index + max_count > sim_config->g_c_info.collision_count || max_count == 0)
  {
#if defined(BB_DEBUG)
    // printf("Collision solver failed index %d, start_index %d, max_count %d\n", index, start_index, max_count);
#endif // BB_DEBUG
    return;
  }
  
  // TODO: Implement islands
  for(daxa_u32 m = start_index; m < start_index + max_count; ++m)
  {

    daxa_u32 manifold_index = manifold_links[m].manifold_index;

    Manifold manifold =
      CS.task_head.collisions[manifold_index];

    if(manifold.obb1_index >= sim_config->rigid_body_count || manifold.obb2_index >= sim_config->rigid_body_count || manifold.obb1_index == manifold.obb2_index)
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
    if(sim_config->solver_type == SimSolverType::PGS_SOFT) {
      PGS_SOFT.solve(manifold, r1, r2, accum_impulse, friction, true, inv_dt);
    } else if(sim_config->solver_type == SimSolverType::PGS) {
      PGS.solve(manifold, r1, r2, accum_impulse, friction);
    }
    
    CS.task_head.collisions[manifold_index] = manifold;
    CS.task_head.rigid_bodies[manifold.obb1_index] = r1;
    CS.task_head.rigid_bodies[manifold.obb2_index] = r2;
  }
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] 
    void entry_integrate_positions_rigid_bodies(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(RB_integrate.task_head.sim_config);

  if (pixel_i.x >= sim_config->rigid_body_count)
  {
    return;
  }

  daxa_u32 index = pixel_i.x;

  Ptr<RigidBody> r =
      Ptr<RigidBody>(&RB_integrate.task_head.rigid_bodies[index]);

  // Integrate positions
  if(sim_config->has_flag(SimFlag::ADVECTION))
  {
    r->integrate_positions(sim_config->dt);
  }
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_collision_solver_relaxation(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(CSR.task_head.sim_config);

  daxa_u32 index = pixel_i.x;
  if (index >= sim_config->island_count)
  {
    return;
  }

  Ptr<Island> islands =
      Ptr<Island>(CSR.task_head.islands);

  Ptr<ManifoldLinkIsland> manifold_links =
      Ptr<ManifoldLinkIsland>(CSR.task_head.manifold_links);
  
  daxa_u32 start_index = islands[index].manifold_start_index;
  daxa_u32 max_count = islands[index].max_manifold_count;

  if(start_index + max_count > sim_config->g_c_info.collision_count || max_count == 0)
  {
#if defined(BB_DEBUG)
    // printf("Collision solver relaxation failed index %d, start_index %d, max_count %d\n", index, start_index, max_count);
#endif // BB_DEBUG
    return;
  }
  
  // TODO: Implement islands
  for(daxa_u32 m = start_index; m < start_index + max_count; ++m)
  {

    daxa_u32 manifold_index = manifold_links[m].manifold_index;

    Manifold manifold =
      CSR.task_head.collisions[manifold_index];

    if(manifold.obb1_index >= sim_config->rigid_body_count || manifold.obb2_index >= sim_config->rigid_body_count || manifold.obb1_index == manifold.obb2_index)
    {
      continue;
    }

    RigidBody r1 = CSR.task_head.rigid_bodies[manifold.obb1_index];
    RigidBody r2 = CSR.task_head.rigid_bodies[manifold.obb2_index];


    daxa_b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
    daxa_b32 friction = sim_config->has_flag(SimFlag::FRICTION);
    daxa_f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;

    // Solve the collision with relaxation
    if(sim_config->solver_type == SimSolverType::PGS_SOFT) {
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
    entry_create_contact_points(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(CP.task_head.sim_config);

  if (pixel_i.x >= sim_config->g_c_info.collision_count)
  {
    return;
  }

  Manifold m =
      CP.task_head.collisions[pixel_i.x];

  daxa_u32 index = 0;
  InterlockedAdd(sim_config->g_c_info.collision_point_count, m.contact_count, index);

  if(sim_config->has_flag(SimFlag::DEBUG_INFO))
  {
    var vertex_buffer = Ptr<GUIVertex>(CP.task_head.vertex_buffer);
    var line_vertex_buffer = Ptr<GUIVertexLine>(CP.task_head.line_vertex_buffer);

    for(int i = 0; i < m.contact_count; i++)
    {
      daxa_u32 current_index = index + i;
        vertex_buffer[current_index].position = m.contacts[i].position;
        daxa_f32 blue = (daxa_f32)(255 - m.contacts[i].warm_start) / 255.0f;
        daxa_f32 red = 1.0f - blue;
        vertex_buffer[current_index].color = daxa_f32vec3(red, 0.0f, blue);

        line_vertex_buffer[current_index*LINE_POINT_COUNT].position = m.contacts[i].position;
        line_vertex_buffer[current_index*LINE_POINT_COUNT+1].position = m.contacts[i].position + m.normal * LINE_SIZE;
    }
  }
}

const static daxa_u32 AXES_POINT_COUNT = 6;

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] 
    void entry_update_rigid_bodies(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(RB_update.task_head.sim_config);

  if (pixel_i.x >= sim_config->rigid_body_count)
  {
    return;
  }

  daxa_u32 index = pixel_i.x;

  RigidBody r = RB_update.task_head.rigid_bodies[index];
  RigidBody r_copy = RB_update.task_head.rigid_bodies_update[index];

  r_copy.position = r.position;
  r_copy.rotation = r.rotation;
  r_copy.velocity = r.velocity;
  r_copy.omega = r.omega;
  r_copy.flags = r.flags;
  r_copy.active_index = MAX_U32;
#if defined(BB_DEBUG)
  r_copy.face_collided = r.face_collided;
#endif // BB_DEBUG

  if(sim_config->has_flag(SimFlag::DEBUG_INFO))
  {
    daxa_f32vec3 _min = r.minimum;
    daxa_f32vec3 _max = r.maximum;
    daxa_f32vec3 center = (_min + _max) * 0.5f;
    var axes_vertex_buffer = Ptr<GUIVertexLine>(RB_update.task_head.axes_vertex_buffer);
    let x_axis = r.rotate_vector(daxa_f32vec3(1.0f, 0.0f, 0.0f));
    let y_axis = r.rotate_vector(daxa_f32vec3(0.0f, 1.0f, 0.0f));
    let z_axis = r.rotate_vector(daxa_f32vec3(0.0f, 0.0f, 1.0f));

    center = r.object_to_world(center);

    axes_vertex_buffer[index*AXES_POINT_COUNT].position = center;
    axes_vertex_buffer[index*AXES_POINT_COUNT+1].position = center + x_axis;
    axes_vertex_buffer[index*AXES_POINT_COUNT+2].position = center;
    axes_vertex_buffer[index*AXES_POINT_COUNT+3].position = center + y_axis;
    axes_vertex_buffer[index*AXES_POINT_COUNT+4].position = center;
    axes_vertex_buffer[index*AXES_POINT_COUNT+5].position = center + z_axis;
  }

  RB_update.task_head.rigid_bodies_update[index] = r_copy;


#if defined(BB_DEBUG)
  // printf("Updating rigid body %d sim frame: %lu c frame: %lu p frame: %lu\n", index, sim_config->frame_count, r.frame_count, r_copy.frame_count);
#endif // BB_DEBUG
}