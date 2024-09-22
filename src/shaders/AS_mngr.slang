#include "shared.inl"

[[vk::push_constant]] UpdateInstancesPushConstants p;

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_update_acceleration_structures(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(p.task_head.sim_config);

  if (pixel_i.x >= sim_config->rigid_body_count)
  {
    return;
  }

  RigidBody r =
      p.task_head.rigid_bodies[pixel_i.x];

  Ptr<daxa_BlasInstanceData> instance = Ptr<daxa_BlasInstanceData>(&p.task_head.blas_instance_data[pixel_i.x]);

  instance->transform = r.get_instance_transform();
}