
#include "defines.hpp"
#include "window.hpp"
#include "gpu_context.hpp"
#include "pipeline_manager.hpp"
#include "ray_tracing_task_graph.hpp"
#include "ray_tracing_SBT.hpp"
#include "acceleration_structure_manager.hpp"

#include "shared.inl"

using namespace beatbox;

int main(int argc, char const *argv[])
{
  AppWindow window("Beat Box", 860, 640);
  GPUcontext gpu("RT device", "Swapchain", window);
  PipelineManager pipeline_manager("Pipeline Manager", gpu.device);

  auto ray_tracing_pipeline = pipeline_manager.create_ray_tracing(MainRayTracingPipeline{}.info);
  auto rt_pipeline_sbt = RayTracingSBT(ray_tracing_pipeline, gpu.device);

  auto accel_struct_mngr = AccelerationStructureManager(gpu.device);
  accel_struct_mngr.create();

  // TODO: Create Scene here
  auto camera_buffer = gpu.device.create_buffer({
      .size = sizeof(Camera),
      .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
      .name = "camera_buffer",
  });

  RayTracingTaskGraph loop_TG("RT TaskGraph", gpu, {
      .ray_tracing_pipeline = ray_tracing_pipeline,
      .shader_binding_table = rt_pipeline_sbt.build_sbt(),
      .camera = camera_buffer,
  });

  {
    // TODO: temporary scene
    std::vector<RigidBody> rigid_bodies = {{.primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0, 0.0, 1.0), .rotation = daxa_f32vec4(1.0, 0.0, 0.0, 0.0)}, {.primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0, 2.0, 1.0), .rotation = daxa_f32vec4(1.0, 0.0, 0.0, 0.0)}};
    std::vector<Aabb> const &aabb = {{.min = daxa_f32vec3(-0.5, -0.5, -0.5), .max = daxa_f32vec3(0.5, 0.5, 0.5)}, {.min = daxa_f32vec3(-0.5, -0.5, -0.5), .max = daxa_f32vec3(0.5, 0.5, 0.5)}};
    accel_struct_mngr.build_accel_structs(rigid_bodies, aabb);
    accel_struct_mngr.build_accel_struct_execute();
    gpu.synchronize();
  }

  while (!window.should_close())
  {
      window.update();

      if (window.swapchain_out_of_date)
      {
          gpu.swapchain_resize();
          window.swapchain_out_of_date = false;
          loop_TG.SBT = rt_pipeline_sbt.rebuild_sbt();
      }

      auto swapchain_image = gpu.swapchain.acquire_next_image();
      if (!swapchain_image.is_empty())
      {
          loop_TG.update_resources(swapchain_image, camera_buffer, accel_struct_mngr.tlas, accel_struct_mngr.rigid_body_buffer, accel_struct_mngr.primitive_buffer);
          loop_TG.execute();
          gpu.garbage_collector();
      }
  }

  accel_struct_mngr.destroy();

  // TODO: Handled by RAII
  if (!camera_buffer.is_empty()) {
      gpu.device.destroy_buffer(camera_buffer);
  }

  return 0;
}