
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
  // Create a window
  auto window = AppWindow("Beat Box", 860, 640);

  // Create a GPU context
  auto gpu = GPUcontext("RT device", window);

  // Create the pipeline manager
  auto pipeline_manager = PipelineManager("Pipeline Manager", gpu.device);

  std::shared_ptr<daxa::RayTracingPipeline> ray_tracing_pipeline;

  // Create the ray tracing pipeline
  {
    auto ray_tracing_pipeline_info = MainRayTracingPipeline{};
    ray_tracing_pipeline = pipeline_manager.create_ray_tracing(ray_tracing_pipeline_info.info);
  }

  // Create the shader binding table
  auto rt_pipeline_sbt = RayTracingSBT(ray_tracing_pipeline, gpu.device);

  auto camera_buffer = gpu.device.create_buffer({
        .size = sizeof(Camera),
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
        .name = "camera_buffer",
    });

  // Create the task graph
  auto loop_TG_params = RayTracingParams{
    .ray_tracing_pipeline = ray_tracing_pipeline,
    .shader_binding_table = rt_pipeline_sbt.build_sbt(),
    .camera = camera_buffer,
  };
  auto loop_TG = RayTracingTaskGraph("ray_tracing_task_graph", gpu, loop_TG_params);
  // Main loop
  while (!window.should_close())
  {
    // Poll events
    window.update();

    // Resize the swapchain if it is out of date
    if (window.swapchain_out_of_date)
    {
      gpu.swapchain.resize();
      window.swapchain_out_of_date = false;
      loop_TG.SBT = rt_pipeline_sbt.rebuild_sbt();
    }

    // acquire the next image
    auto swapchain_image = gpu.swapchain.acquire_next_image();
    if (swapchain_image.is_empty())
    {
      continue;
    }

    // We update the image id of the task swapchain image.
    loop_TG.task_swapchain_image.set_images({.images = std::span{&swapchain_image, 1}});

    // So, now all we need to do is execute our task graph!
    loop_TG.ray_tracing_task_graph.execute({});
    gpu.device.collect_garbage();
  }

  if(!camera_buffer.is_empty()) {
    gpu.device.destroy_buffer(camera_buffer);
  }

  return 0;
}