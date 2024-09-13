#pragma once
#include "defines.hpp"

BB_NAMESPACE_BEGIN

struct RayTracingTaskGraph
{

  daxa::TaskGraph ray_tracing_task_graph;
  daxa::TaskImage task_swapchain_image{{.swapchain_image = true, .name = "swapchain_image"}};
  daxa::RayTracingShaderBindingTable shader_binding_table;
  daxa::Extent2D size{860, 640};
  std::shared_ptr<daxa::RayTracingPipeline> pipeline;

  explicit RayTracingTaskGraph(GPUcontext &gpu, std::shared_ptr<daxa::RayTracingPipeline> ray_tracing_pipeline) : pipeline(ray_tracing_pipeline)
  {
    ray_tracing_task_graph = daxa::TaskGraph({
        .device = gpu.device,
        .swapchain = gpu.swapchain,
        .use_split_barriers = false,
        .record_debug_information = true,
        .name = "ray_tracing_task_graph",
    });
    ray_tracing_task_graph.use_persistent_image(task_swapchain_image);

    ray_tracing_task_graph.add_task({
      .attachments = {daxa::inl_attachment(daxa::TaskImageAccess::RAY_TRACING_SHADER_STORAGE_WRITE_ONLY, task_swapchain_image)},
      .task = [this](daxa::TaskInterface ti) {
        ti.recorder.set_pipeline(*pipeline);
        ti.recorder.push_constant(PushConstants{
          .size = daxa_u32vec2{size.x, size.y},
          .swapchain = task_swapchain_image.get_state().images.front().default_view(),
        });
        ti.recorder.trace_rays({
            .width = size.x,
            .height = size.y,
            .depth = 1,
            .shader_binding_table = shader_binding_table,
        });
      },
      .name = "ray_tracing",
    });

    ray_tracing_task_graph.submit({});
    ray_tracing_task_graph.present({});
    ray_tracing_task_graph.complete({});
  }
};

BB_NAMESPACE_END