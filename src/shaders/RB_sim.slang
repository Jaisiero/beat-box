// #include "extensions.slang"
#include "collision_detection.slang"

[[vk::push_constant]] BroadPhasePushConstants BP;
[[vk::push_constant]] CollisionSolverDispatcherPushConstants CSD;
[[vk::push_constant]] CollisionPreSolverPushConstants CPS;
[[vk::push_constant]] CollisionSolverPushConstants CS;
[[vk::push_constant]] RigidBodySimPushConstants RB_sim;
[[vk::push_constant]] CreatePointsPushConstants CP;



[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_broad_phase(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(BP.task_head.sim_config);

  if (pixel_i.x >= sim_config->rigid_body_count)
  {
    return;
  }

  Ptr<RigidBody> r =
      Ptr<RigidBody>(&BP.task_head.rigid_bodies[pixel_i.x]);

  Ptr<Manifold> collisions =
      Ptr<Manifold>(BP.task_head.collisions);

  Ptr<Manifold> old_collisions =
      Ptr<Manifold>(BP.task_head.old_collisions);


  broad_phase(pixel_i.x, r, sim_config, collisions, old_collisions);
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

  dispatch_buffer->solver_dispatch.x = (sim_config->g_c_info.collision_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X;

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

  r->advect(sim_config);

  // printf("rigid body %d position %f %f %f velocity %f %f %f omega %f %f %f\n", pixel_i.x, r->position.x, r->position.y, r->position.z, r->velocity.x, r->velocity.y, r->velocity.z, r->omega.x, r->omega.y, r->omega.z);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_collision_pre_solver(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(CPS.task_head.sim_config);

  if (pixel_i.x >= 1)
  {
    return;
  }
  
  // TODO: Implement parallel collision solver
  // atomicAdd to velocity and omega
  for(daxa_u32 m = 0u; m < sim_config->g_c_info.collision_count; ++m)
  {
    Manifold manifold =
      CPS.task_head.collisions[m];

    if(manifold.obb1_index >= sim_config->rigid_body_count || manifold.obb2_index >= sim_config->rigid_body_count)
    {
      return;
    }

    Ptr<RigidBody> r1 =
        Ptr<RigidBody>(&CPS.task_head.rigid_bodies[manifold.obb1_index]);
    Ptr<RigidBody> r2 =
        Ptr<RigidBody>(&CPS.task_head.rigid_bodies[manifold.obb2_index]);

    daxa_b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
    daxa_b32 friction = sim_config->has_flag(SimFlag::FRICTION);
    daxa_f32 inv_dt = sim_config->dt > 0.0f ? 1.0f / sim_config->dt : 0.0f;
    manifold.pre_step(inv_dt, r1, r2, accum_impulse, friction);

    CPS.task_head.collisions[m] = manifold;
  }
}


[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_collision_solver(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(CS.task_head.sim_config);

  if (pixel_i.x >= 1)
  {
    return;
  }
  
  // TODO: Implement parallel collision solver
  // atomicAdd to velocity and omega
  for(daxa_u32 m = 0u; m < sim_config->g_c_info.collision_count; ++m)
  {
    Manifold manifold =
      CS.task_head.collisions[m];

    if(manifold.obb1_index >= sim_config->rigid_body_count || manifold.obb2_index >= sim_config->rigid_body_count)
    {
      return;
    }

    Ptr<RigidBody> r1 =
        Ptr<RigidBody>(&CS.task_head.rigid_bodies[manifold.obb1_index]);
    Ptr<RigidBody> r2 =
        Ptr<RigidBody>(&CS.task_head.rigid_bodies[manifold.obb2_index]);

    daxa_b32 accum_impulse = sim_config->has_flag(SimFlag::ACCUM_IMPULSE);
    daxa_b32 friction = sim_config->has_flag(SimFlag::FRICTION);

    // Solve the collision
    manifold.solve(r1, r2, accum_impulse, friction);
    
    CS.task_head.collisions[m] = manifold;
  }
}

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

  var vertex_buffer = Ptr<GUIVertex>(CP.task_head.vertex_buffer);

  for(int i = 0; i < m.contact_count; i++)
  {
    daxa_u32 current_index = index + i;
    Ptr<Aabb> aabb = Ptr<Aabb>(&CP.task_head.point_aabbs[current_index]);
    aabb->minimum = m.contacts[i].position + daxa_f32vec3(-POINT_SIZE, -POINT_SIZE, -POINT_SIZE);
    aabb->maximum = m.contacts[i].position + daxa_f32vec3(POINT_SIZE, POINT_SIZE, POINT_SIZE);

    vertex_buffer[current_index].position = m.contacts[i].position;
  }
}