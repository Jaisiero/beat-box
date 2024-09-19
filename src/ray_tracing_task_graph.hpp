#pragma once
#include "defines.hpp"
#include <daxa/utils/task_graph.hpp>
#include "acceleration_structure_manager.hpp"
#include "camera_manager.hpp"
#include "rigid_body_manager.hpp"
#include "ray_tracing_pipeline.hpp"
#include "scene_manager.hpp"

BB_NAMESPACE_BEGIN

struct RayTracingParams
{
  std::shared_ptr<daxa::RayTracingPipeline> pipeline;
  daxa::RayTracingShaderBindingTable SBT;
};

struct RendererManager
{
  // Gpu context reference
  std::shared_ptr<GPUcontext> gpu;
  // Initialization flag
  bool initialized = false;
  // Task manager reference
  std::shared_ptr<TaskManager> task_manager;
  // Window reference
  WindowManager& window;
  // Camera manager reference
  std::shared_ptr<CameraManager> camera_manager;
  // Acceleration structure manager reference
  std::shared_ptr<AccelerationStructureManager> accel_struct_mngr;
  // Rigid body manager reference
  std::shared_ptr<RigidBodyManager> rigid_body_manager;
  // Ray tracing pipeline
  std::shared_ptr<RayTracingPipeline> RT_pipeline;
  // Scene manager reference
  std::shared_ptr<SceneManager> scene_manager;

  // Task graph information for ray tracing
  TaskGraph RT_TG;
  daxa::TaskImage task_swapchain_image{{.swapchain_image = true, .name = "swapchain_image"}};
  daxa::TaskBuffer task_camera_buffer{{.initial_buffers = {}, .name = "camera_buffer"}};
  daxa::TaskTlas task_tlas{{.name = "tlas"}};
  daxa::TaskBuffer task_rigid_bodies{{.initial_buffers = {}, .name = "rigid_bodies"}};
  daxa::TaskBuffer task_aabbs{{.initial_buffers = {}, .name = "aabbs"}};

  explicit RendererManager(std::shared_ptr<GPUcontext> gpu, std::shared_ptr<TaskManager> task_manager, WindowManager& window, std::shared_ptr<CameraManager> camera_manager, std::shared_ptr<AccelerationStructureManager> accel_struct_mngr, std::shared_ptr<RigidBodyManager> rigid_body_manager, std::shared_ptr<SceneManager> scene_manager)
      : gpu(gpu), task_manager(task_manager), window(window), camera_manager(camera_manager), accel_struct_mngr(accel_struct_mngr), rigid_body_manager(rigid_body_manager), scene_manager(scene_manager)
  {
  }

  ~RendererManager() {}

  bool create(char const *RT_TG_name, std::shared_ptr<RayTracingPipeline> pipeline, daxa::RayTracingShaderBindingTable SBT)
  {
    if (initialized)
    {
      return false;
    }

    RT_pipeline = pipeline;

    auto user_callback = [this, SBT](daxa::TaskInterface ti, auto& self) {
        auto const image_info = ti.device.info_image(ti.get(RayTracingTaskHead::AT.swapchain).ids[0]).value();
        ti.recorder.set_pipeline(*RT_pipeline->pipeline);
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

    RT_TG = task_manager->create_task_graph(RT_TG_name, std::span<TTask>(tasks), buffers, images, {}, tlases, true);

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


  void render() {
    while (!window.should_close())
    {
      rigid_body_manager->simulate();
      accel_struct_mngr->update();
      accel_struct_mngr->update_TLAS();

      if (!window.update())
        continue;

      if (window.swapchain_out_of_date)
      {
        gpu->swapchain_resize();
        window.swapchain_out_of_date = false;
      }

      auto handle_reload_result = [&](daxa::PipelineReloadResult reload_error, std::shared_ptr<RayTracingPipeline> RT_pipeline, RendererManager* TG) -> void
      {
        if (auto error = daxa::get_if<daxa::PipelineReloadError>(&reload_error)) {
              std::cout << "Failed to reload " << error->message << std::endl;
        } else if (daxa::get_if<daxa::PipelineReloadSuccess>(&reload_error)) {
          TG->destroy();
          TG->create("Ray Tracing Task Graph", RT_pipeline, RT_pipeline->rebuild_SBT());
          std::cout << "Successfully reloaded!" << std::endl;
        }
      };

      auto swapchain_image = gpu->swapchain_acquire_next_image();
      if (!swapchain_image.is_empty())
      {
        handle_reload_result(task_manager->reload(), RT_pipeline, this);
        
        camera_manager->update(gpu->swapchain_get_extent());
        update_resources(swapchain_image, *camera_manager, accel_struct_mngr->tlas, accel_struct_mngr->rigid_body_buffer, accel_struct_mngr->primitive_buffer);
        execute();
        gpu->garbage_collector();
      }
    }
    gpu->synchronize();
    gpu->garbage_collector();
  }
};

BB_NAMESPACE_END