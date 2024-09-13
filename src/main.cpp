
#include "defines.hpp"
#include "window.hpp"
#include "gpu_context.hpp"
#include "pipeline_manager.hpp"
#include "ray_tracing_task_graph.hpp"

#include "shared.inl"

using namespace beatbox;

int main(int argc, char const *argv[])
{
  // Create a window
  auto window = AppWindow("Beat Box", 860, 640);

  auto gpu = GPUcontext(window);

  auto pipeline_manager = PipelineManager(gpu.device);

  std::shared_ptr<daxa::RayTracingPipeline> ray_tracing_pipeline;

  {
    auto ray_tracing_pipeline_info = MainRayTracingPipeline{};
    ray_tracing_pipeline = pipeline_manager.create_ray_tracing(ray_tracing_pipeline_info.info);
  }
  daxa::RayTracingPipeline::SbtPair sbt_pair = ray_tracing_pipeline->create_default_sbt();

  auto loop_TG = RayTracingTaskGraph(gpu, ray_tracing_pipeline);

  auto build_sbt = [&]() -> daxa::RayTracingShaderBindingTable {
    return {
        .raygen_region = sbt_pair.entries.group_regions.at(0).region,
        .miss_region = sbt_pair.entries.group_regions.at(1).region,
        .hit_region = sbt_pair.entries.group_regions.at(2).region,
        .callable_region = {},
    };
  };

  auto free_sbt = [&](daxa::RayTracingPipeline::SbtPair &sbt) {
        gpu.device.destroy_buffer(sbt.buffer);
        gpu.device.destroy_buffer(sbt.entries.buffer);
  };

  daxa::RayTracingShaderBindingTable shader_binding_table = build_sbt();

  loop_TG.shader_binding_table = shader_binding_table;

  while (!window.should_close())
  {
    // Poll events
    window.update();

    // Resize the swapchain if it is out of date
    if (window.swapchain_out_of_date)
    {
      gpu.swapchain.resize();
      window.swapchain_out_of_date = false;
      if(!sbt_pair.buffer.is_empty()) {
        free_sbt(sbt_pair);
      }
      sbt_pair = ray_tracing_pipeline->create_default_sbt();
      shader_binding_table = build_sbt();
      loop_TG.shader_binding_table = shader_binding_table;
    }

    // acquire the next image
    auto swapchain_image = gpu.swapchain.acquire_next_image();
    if (swapchain_image.is_empty())
    {
      continue;
    }

    // We update the image id of the task swapchain image.
    loop_TG.task_swapchain_image.set_images({.images = std::span{&swapchain_image, 1}});
    loop_TG.size = gpu.swapchain.get_surface_extent();

    // So, now all we need to do is execute our task graph!
    loop_TG.ray_tracing_task_graph.execute({});
    gpu.device.collect_garbage();
  }

  if(!sbt_pair.buffer.is_empty()) {
    free_sbt(sbt_pair);
  }

  return 0;
}