#include "shared.inl"

[[vk::push_constant]] UpdateInstancesPushConstants UI;

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_update_acceleration_structures(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(UI.task_head.sim_config);

  daxa_u32 index = pixel_i.x;

  if (index >= sim_config->rigid_body_count)
  {
    return;
  }

  RigidBody r =
      UI.task_head.rigid_bodies[index];

  daxa_BlasInstanceData* instance = (daxa_BlasInstanceData*)(&UI.task_head.blas_instance_data[index]);

  instance->instance_custom_index = index;
  instance->mask = 0xFF;
  instance->instance_shader_binding_table_record_offset = 0;
  instance->flags = 0;
  instance->transform = r.get_instance_transform();
}