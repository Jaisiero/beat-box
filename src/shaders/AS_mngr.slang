#include "shared.inl"

[[vk::push_constant]] UpdateInstancesPushConstants UI;
[[vk::push_constant]] CreatePointsPushConstants CP;

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_update_acceleration_structures(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(UI.task_head.sim_config);

  if (pixel_i.x >= sim_config->rigid_body_count)
  {
    return;
  }

  RigidBody r =
      UI.task_head.rigid_bodies[pixel_i.x];

  Ptr<daxa_BlasInstanceData> instance = Ptr<daxa_BlasInstanceData>(&UI.task_head.blas_instance_data[pixel_i.x]);

  instance->transform = r.get_instance_transform();
}

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_create_contact_points(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(CP.task_head.sim_config);

  if (pixel_i.x >= sim_config->collision_count)
  {
    return;
  }

  Manifold m =
      CP.task_head.collisions[pixel_i.x];

  daxa_u32 index = 0;
  InterlockedAdd(sim_config->collision_point_count, m.contact_count, index);

  for(int i = 0; i < m.contact_count; i++)
  {
    daxa_u32 current_index = index + i;
    Ptr<Aabb> aabb = Ptr<Aabb>(&CP.task_head.point_aabbs[current_index]);
    aabb->minimum = m.contacts[i].position + daxa_f32vec3(-0.1, -0.1, -0.1);
    aabb->maximum = m.contacts[i].position + daxa_f32vec3(0.1, 0.1, 0.1);
  }
}