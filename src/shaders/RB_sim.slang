#include "shared.inl"

[[vk::push_constant]] RigidBodySimPushConstants p;

func GJK(daxa_u32 rigid_body_index, Ptr<RigidBody> r,
         Ptr<SimConfig> sim_config){
    // GJK algorithm
}

    [shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_GJK(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(p.task_head.sim_config);

  if (pixel_i.x >= sim_config->rigid_body_count)
  {
    return;
  }

  Ptr<RigidBody> rigid_body =
      Ptr<RigidBody>(&p.task_head.rigid_bodies[pixel_i.x]);

  // GJK(pixel_i.x, rigid_body, sim_config);
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_rigid_body_sim(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(p.task_head.sim_config);

  if (pixel_i.x >= sim_config->rigid_body_count)
  {
    return;
  }

  Ptr<RigidBody> rigid_body =
      Ptr<RigidBody>(&p.task_head.rigid_bodies[pixel_i.x]);

  rigid_body->advect(sim_config);
}