#pragma once
#include "defines.hpp"
#include <daxa/utils/task_graph.hpp>

BB_NAMESPACE_BEGIN

struct RayTracingParams
{
  std::shared_ptr<daxa::RayTracingPipeline> pipeline;
  daxa::RayTracingShaderBindingTable SBT;
};

struct RayTracingTaskGraph
{
  // Gpu context reference
  GPUcontext &gpu;
  // Initialization flag
  bool initialized = false;
  // Task manager reference
  TaskManager &task_manager;

  // Task graph information for ray tracing
  TaskGraph RT_TG;
  daxa::TaskImage task_swapchain_image{{.swapchain_image = true, .name = "swapchain_image"}};
  daxa::TaskBuffer task_camera_buffer{{.initial_buffers = {}, .name = "camera_buffer"}};
  daxa::TaskTlas task_tlas{{.name = "tlas"}};
  daxa::TaskBuffer task_rigid_bodies{{.initial_buffers = {}, .name = "rigid_bodies"}};
  daxa::TaskBuffer task_aabbs{{.initial_buffers = {}, .name = "aabbs"}};

  explicit RayTracingTaskGraph(GPUcontext &gpu, TaskManager& task_manager) : gpu(gpu), task_manager(task_manager)
  {
  }

  ~RayTracingTaskGraph() {}

  bool create(char const *RT_TG_name, 
  std::shared_ptr<daxa::RayTracingPipeline> pipeline,
  daxa::RayTracingShaderBindingTable SBT)
  {
    if (initialized)
    {
      return false;
    }

    auto user_callback = [pipeline, SBT](daxa::TaskInterface ti, auto& self) {
        auto const image_info = ti.device.info_image(ti.get(RayTracingTaskHead::AT.swapchain).ids[0]).value();
        ti.recorder.set_pipeline(*pipeline);
        ti.recorder.push_constant(RTPushConstants{.task_head = ti.attachment_shader_blob});
        ti.recorder.trace_rays({
            .width = image_info.size.x,
            .height = image_info.size.y,
            .depth = 1,
            .shader_binding_table = SBT,
        });
    };
    
    using TTask = TaskTemplate<RayTracingTaskHead::Task, decltype(user_callback)>;

    // Instantiate the task using the template class
    TTask task1(std::array{
        daxa::attachment_view(RayTracingTaskHead::AT.camera, task_camera_buffer),
        daxa::attachment_view(RayTracingTaskHead::AT.swapchain, task_swapchain_image),
        daxa::attachment_view(RayTracingTaskHead::AT.tlas, task_tlas),
        daxa::attachment_view(RayTracingTaskHead::AT.aabbs, task_aabbs),
        daxa::attachment_view(RayTracingTaskHead::AT.rigid_bodies, task_rigid_bodies),
      }, user_callback);

    std::array<daxa::TaskBuffer, 3> buffers = {task_camera_buffer, task_rigid_bodies, task_aabbs};

    std::array<daxa::TaskImage, 1> images = {task_swapchain_image};

    std::array<daxa::TaskTlas, 1> tlases = {task_tlas};
    
    std::array<TTask, 1> tasks = {task1};

    RT_TG = task_manager.create_task_graph(RT_TG_name, std::span<TTask>(tasks), buffers, images, {}, tlases, true);

    RT_TG.submit();
    RT_TG.present();
    RT_TG.complete();

    return initialized = true;
  }

  void destroy()
  {
    if (initialized)
    {
      initialized = false;
    }
  }

  bool execute()
  {
    if(!initialized)
    {
      return false;
    }
    RT_TG.execute();
    return true;
  }

  bool update_resources(daxa::ImageId swapchain_image, CameraManager &cam_mngr, daxa::TlasId tlas, daxa::BufferId rigid_bodies, daxa::BufferId aabbs)
  {
    if (!initialized)
    {
      return false;
    }

    task_swapchain_image.set_images({.images = std::array{swapchain_image}});
    task_camera_buffer.set_buffers({.buffers = std::array{cam_mngr.camera_buffer}});
    task_tlas.set_tlas({.tlas = std::array{tlas}});
    task_rigid_bodies.set_buffers({.buffers = std::array{rigid_bodies}});
    task_aabbs.set_buffers({.buffers = std::array{aabbs}});

    return true;
  }
};

BB_NAMESPACE_END