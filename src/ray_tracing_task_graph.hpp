#pragma once
#include "defines.hpp"
#include <daxa/utils/task_graph.hpp>

BB_NAMESPACE_BEGIN

struct RayTracingTask : RayTracingTaskHead::Task
{
  AttachmentViews views = {};
  std::shared_ptr<daxa::RayTracingPipeline> pipeline = {};
  daxa::RayTracingShaderBindingTable SBT;
  
  void callback(daxa::TaskInterface ti)
  {
    auto const image_info = ti.device.info_image(ti.get(RayTracingTask::AT.swapchain).ids[0]).value();
    ti.recorder.set_pipeline(*pipeline);
    ti.recorder.push_constant(PushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.trace_rays({
        .width = image_info.size.x,
        .height = image_info.size.y,
        .depth = 1,
        .shader_binding_table = SBT,
    });
  };
};

struct RayTracingParams
{ 
  std::shared_ptr<daxa::RayTracingPipeline> ray_tracing_pipeline;
  daxa::RayTracingShaderBindingTable shader_binding_table;
  daxa::BufferId camera;
};

struct RayTracingTaskGraph
{
  daxa::TaskGraph ray_tracing_task_graph;
  daxa::TaskImage task_swapchain_image{{.swapchain_image = true, .name = "swapchain_image"}};
  daxa::TaskBuffer task_camera_buffer{{.initial_buffers = {}, .name = "camera_buffer"}};
  daxa::TaskTlas task_tlas{{.name = "tlas"}};
  daxa::TaskBuffer task_rigid_bodies{{.initial_buffers = {}, .name = "rigid_bodies"}};
  daxa::TaskBuffer task_aabbs{{.initial_buffers = {}, .name = "aabbs"}};
  std::shared_ptr<daxa::RayTracingPipeline> pipeline;
  daxa::RayTracingShaderBindingTable SBT;

  explicit RayTracingTaskGraph(char const * RT_TG_name, GPUcontext &gpu, RayTracingParams params) : pipeline(params.ray_tracing_pipeline), SBT(params.shader_binding_table)
  {
    task_camera_buffer.set_buffers({.buffers = std::array{params.camera}});
    ray_tracing_task_graph = daxa::TaskGraph({
        .device = gpu.device,
        .swapchain = gpu.swapchain,
        .use_split_barriers = false,
        .record_debug_information = true,
        .name = RT_TG_name,
    });
    ray_tracing_task_graph.use_persistent_image(task_swapchain_image);
    ray_tracing_task_graph.use_persistent_buffer(task_camera_buffer);
    ray_tracing_task_graph.use_persistent_tlas(task_tlas);
    ray_tracing_task_graph.use_persistent_buffer(task_rigid_bodies);
    ray_tracing_task_graph.use_persistent_buffer(task_aabbs);

    ray_tracing_task_graph.add_task(RayTracingTask{
        .views = std::array{
            daxa::attachment_view(RayTracingTaskHead::AT.swapchain, task_swapchain_image),
            daxa::attachment_view(RayTracingTaskHead::AT.camera, task_camera_buffer),
            daxa::attachment_view(RayTracingTaskHead::AT.tlas, task_tlas),
            daxa::attachment_view(RayTracingTaskHead::AT.rigid_bodies, task_rigid_bodies),
            daxa::attachment_view(RayTracingTaskHead::AT.aabbs, task_aabbs),
        },
        .pipeline = pipeline,
        .SBT = SBT,
    });

    ray_tracing_task_graph.submit({});
    ray_tracing_task_graph.present({});
    ray_tracing_task_graph.complete({});
  }

  void execute()
  {
    ray_tracing_task_graph.execute({});
  }

  void update_resources(daxa::ImageId swapchain_image, CameraManager& cam_mngr, daxa::TlasId tlas, daxa::BufferId rigid_bodies, daxa::BufferId aabbs)
  {
    task_swapchain_image.set_images({.images = std::array{swapchain_image}});
    task_camera_buffer.set_buffers({.buffers = std::array{cam_mngr.camera_buffer}});
    task_tlas.set_tlas({.tlas = std::array{tlas}});
    task_rigid_bodies.set_buffers({.buffers = std::array{rigid_bodies}});
    task_aabbs.set_buffers({.buffers = std::array{aabbs}});
  }
};

BB_NAMESPACE_END