#include "renderer_manager.hpp"

BB_NAMESPACE_BEGIN

RendererManager::RendererManager(std::shared_ptr<GPUcontext> gpu, std::shared_ptr<TaskManager> task_manager, WindowManager &window, std::shared_ptr<CameraManager> camera_manager, std::shared_ptr<AccelerationStructureManager> accel_struct_mngr, std::shared_ptr<RigidBodyManager> rigid_body_manager, std::shared_ptr<SceneManager> scene_manager, std::shared_ptr<StatusManager> status_manager, std::shared_ptr<GUIManager> gui_manager)
    : gpu(gpu), task_manager(task_manager), window(window), camera_manager(camera_manager), accel_struct_mngr(accel_struct_mngr), rigid_body_manager(rigid_body_manager), scene_manager(scene_manager), status_manager(status_manager), gui_manager(gui_manager) {}

bool RendererManager::create(char const *RT_TG_name, std::shared_ptr<RayTracingPipeline> pipeline, daxa::RayTracingShaderBindingTable SBT)
{
  if (initialized)
  {
    return false;
  }

  RT_pipeline = pipeline;

  auto user_callback = [this, SBT](daxa::TaskInterface ti, auto &self)
  {
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
  TTask task_RT(std::array{
                  daxa::attachment_view(RayTracingTaskHead::AT.camera, task_camera_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.swapchain, task_swapchain_image),
                  daxa::attachment_view(RayTracingTaskHead::AT.tlas, accel_struct_mngr->task_tlas),
                  daxa::attachment_view(RayTracingTaskHead::AT.rigid_bodies,
                                        rigid_body_manager->task_rigid_bodies),
                  daxa::attachment_view(RayTracingTaskHead::AT.aabbs, accel_struct_mngr->task_aabb_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.materials, scene_manager->task_material_buffer),
              },
              user_callback);
  

  std::array<daxa::TaskBuffer, 7> buffers = {task_camera_buffer, rigid_body_manager->task_rigid_bodies, accel_struct_mngr->task_aabb_buffer,  gui_manager->task_vertex_buffer,
  gui_manager->task_line_vertex_buffer, gui_manager->task_axes_vertex_buffer,
  scene_manager->task_material_buffer};

  std::array<daxa::TaskImage, 1> images = {task_swapchain_image};

  std::array<daxa::TaskTlas, 1> tlases = {accel_struct_mngr->task_tlas};

  RT_TG = task_manager->create_task_graph(RT_TG_name, buffers, images, {}, tlases, true);

  RT_TG.add_task(task_RT);
  RT_TG.add_task(gui_manager->gui_axes_task_info);
  RT_TG.add_task(gui_manager->gui_line_task_info);
  RT_TG.add_task(gui_manager->gui_task_info);

  RT_TG.submit();
  RT_TG.present();
  RT_TG.complete();

  return initialized = true;
}

void RendererManager::destroy()
{
  if (initialized)
  {
    initialized = false;
  }
}

bool RendererManager::execute()
{
  if (!initialized)
  {
    return false;
  }
  RT_TG.execute();
  return true;
}

bool RendererManager::update_resources(daxa::ImageId swapchain_image, CameraManager &cam_mngr)
{
  if (!initialized)
  {
    return false;
  }

  task_swapchain_image.set_images({.images = std::array{swapchain_image}});
  task_camera_buffer.set_buffers({.buffers = std::array{cam_mngr.camera_buffer}});

  return true;
}

void RendererManager::render()
{
  while (!window.should_close())
  {
    // Update the GUI
    gui_manager->update();

    if(rigid_body_manager->is_dirty()) {
      rigid_body_manager->clean_dirty();
      rigid_body_manager->update_sim();
    }
    
    // Simulate rigid bodies
    if(status_manager->is_simulating()) {
      rigid_body_manager->simulate();
    }

    if (!window.update())
      continue;

    // Update the acceleration structures
    if(status_manager->is_simulating() || status_manager->is_updating()) {
      rigid_body_manager->read_back_sim_config();
      // TODO: change for wait compute queue
      gpu->synchronize();
      if(status_manager->is_updating()) {
        if(!status_manager->reset_update_sim_buffer()) {
          accel_struct_mngr->update_AS_buffers();
        }
      }
      accel_struct_mngr->update_TLAS();
    }

    // Rebuild swapchain
    if (window.swapchain_out_of_date)
    {
      gpu->swapchain_resize();
      window.swapchain_out_of_date = false;
    }

    auto handle_reload_result = [&](daxa::PipelineReloadResult reload_error, std::shared_ptr<RayTracingPipeline> RT_pipeline, RendererManager *TG) -> void
    {
      if (auto error = daxa::get_if<daxa::PipelineReloadError>(&reload_error))
      {
        std::cout << "Failed to reload " << error->message << std::endl;
      }
      else if (daxa::get_if<daxa::PipelineReloadSuccess>(&reload_error))
      {
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
      update_resources(swapchain_image, *camera_manager);
      execute();
      gpu->garbage_collector();

      status_manager->next_frame();
    }
  }
  gpu->synchronize();
  gpu->garbage_collector();
}

RendererManager::~RendererManager() {}

daxa_u32 RendererManager::get_previous_frame_index()
{
  return (status_manager->get_frame_index() + DOUBLE_BUFFERING - 1) % DOUBLE_BUFFERING;
}

daxa_u32 RendererManager::get_frame_index()
{
  return status_manager->get_frame_index();
}

daxa_u32 RendererManager::get_next_frame_index()
{
  return (status_manager->get_frame_index() + 1) % DOUBLE_BUFFERING;
}

BB_NAMESPACE_END