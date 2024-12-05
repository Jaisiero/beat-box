// #include "extensions.slang"
#include "collision_detection.slang"
#include "solvers.slang"

[[vk::push_constant]] BroadPhasePushConstants BP;
[[vk::push_constant]] CollisionSolverDispatcherPushConstants CSD;
[[vk::push_constant]] RigidBodySimPushConstants RB_sim;
[[vk::push_constant]] CollisionPreSolverPushConstants CPS;
[[vk::push_constant]] CollisionSolverPushConstants CS;
[[vk::push_constant]] RigidBodyIntegratePositionsPushConstants RB_integrate;
[[vk::push_constant]] CollisionSolverRelaxationPushConstants CSR;
[[vk::push_constant]] CreatePointsPushConstants CP;
[[vk::push_constant]] RigidBodyUpdatePushConstants RB_update;



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

  Ptr<Manifold> collisions =
      Ptr<Manifold>(BP.task_head.collisions);

  Ptr<Manifold> old_collisions =
      Ptr<Manifold>(BP.task_head.old_collisions);

  broad_phase(index, BP.task_head.rigid_bodies, sim_config, previous_sim_config, collisions, old_collisions);

#if defined(BB_DEBUG)
  // printf("Broad phase frame %lu\n", sim_config->frame_count);
#endif // BB_DEBUG
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

  if(sim_config->has_flag(SimFlag::ADVECTION))
  {
    r->advect(sim_config);
  }
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

  // printf("Pre solver collision count %d, tmp_collision_count: %d\n", sim_config->g_c_info.collision_count, sim_config->g_c_info.tmp_collision_count);
  
  // TODO: Implement islands
  for(daxa_u32 m = 0u; m < sim_config->g_c_info.collision_count; ++m)
  {
    Manifold manifold =
      CPS.task_head.collisions[m];

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

    CPS.task_head.collisions[m] = manifold;
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

  if (pixel_i.x >= 1)
  {
    return;
  }
  
  // TODO: Implement islands
  for(daxa_u32 m = 0u; m < sim_config->g_c_info.collision_count; ++m)
  {
    Manifold manifold =
      CS.task_head.collisions[m];

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
    
    CS.task_head.collisions[m] = manifold;
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

  if (pixel_i.x >= 1)
  {
    return;
  }
  
  // TODO: Implement islands
  for(daxa_u32 m = 0u; m < sim_config->g_c_info.collision_count; ++m)
  {
    Manifold manifold =
      CSR.task_head.collisions[m];

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
    
    CSR.task_head.collisions[m] = manifold;
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
  
  // printf("Updating pos rigid body %d, pos: %f %f %f, rot: %f %f %f %f\n", index, r.position.x, r.position.y, r.position.z, r.rotation.v.x, r.rotation.v.y, r.rotation.v.z, r.rotation.w);
  // printf("Updating rigid body %d, frame: %lu, vel: %f %f %f, omega: %f %f %f\n", index, r.frame_count, r.velocity.x, r.velocity.y, r.velocity.z, r.omega.x, r.omega.y, r.omega.z);

  r_copy.position = r.position;
  r_copy.rotation = r.rotation;
  r_copy.velocity = r.velocity;
  r_copy.omega = r.omega;
  r_copy.flags = r.flags;
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

  RB_update.task_head.rigid_bodies[index] = r;
  RB_update.task_head.rigid_bodies_update[index] = r_copy;


#if defined(BB_DEBUG)
  // printf("Updating rigid body %d sim frame: %lu c frame: %lu p frame: %lu\n", index, sim_config->frame_count, r.frame_count, r_copy.frame_count);
#endif // BB_DEBUG
}