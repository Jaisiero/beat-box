
#include "defines.hpp"
#include "window.hpp"
#include "gpu_context.hpp"
#include "pipeline_manager.hpp"
#include "ray_tracing_task_graph.hpp"
#include "ray_tracing_SBT.hpp"

#include "shared.inl"

using namespace beatbox;

int main(int argc, char const *argv[])
{
  AppWindow window("Beat Box", 860, 640);
  GPUcontext gpu("RT device", "Swapchain", window);
  PipelineManager pipeline_manager("Pipeline Manager", gpu.device);

  auto ray_tracing_pipeline = pipeline_manager.create_ray_tracing(MainRayTracingPipeline{}.info);
  auto rt_pipeline_sbt = RayTracingSBT(ray_tracing_pipeline, gpu.device);

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

  while (!window.should_close())
  {
      window.update();

      if (window.swapchain_out_of_date)
      {
          gpu.swapchain.resize();
          window.swapchain_out_of_date = false;
          loop_TG.SBT = rt_pipeline_sbt.rebuild_sbt();
      }

      auto swapchain_image = gpu.swapchain.acquire_next_image();
      if (!swapchain_image.is_empty())
      {
          loop_TG.task_swapchain_image.set_images({.images = std::span{&swapchain_image, 1}});
          loop_TG.ray_tracing_task_graph.execute({});
          gpu.device.collect_garbage();
      }
  }

  if (!camera_buffer.is_empty()) {
      gpu.device.destroy_buffer(camera_buffer);
  }

  return 0;
}