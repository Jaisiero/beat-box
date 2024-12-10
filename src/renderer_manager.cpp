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

  for(auto f = 0u; f < DOUBLE_BUFFERING; f++) {
    ray_tracing_config_buffer[f] = gpu->device.create_buffer({
        .size = sizeof(RayTracingConfig),
        .name = "ray_tracing_config" + std::to_string(f),
    });

    ray_tracing_config_host_buffer[f] = gpu->device.create_buffer({
        .size = sizeof(RayTracingConfig),
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
        .name = "ray_tracing_config_host" + std::to_string(f),
    });
  }
  
  RT_pipeline = pipeline;

  accumulation_buffer = gpu->device.create_image({
      .format = gpu->swapchain.get_format(),
      .size = daxa::Extent3D(gpu->swapchain_get_extent().x, gpu->swapchain_get_extent().y, 1),
      .usage = daxa::ImageUsageFlagBits::SHADER_STORAGE | daxa::ImageUsageFlagBits::TRANSFER_DST | daxa::ImageUsageFlagBits::TRANSFER_SRC,
      .name = "accumulation_buffer",
  });
  
  daxa::InlineTaskInfo task_update_RT_config({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, 
          task_ray_tracing_config_host),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, 
          task_ray_tracing_config),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        auto const accumulating = status_manager->is_accumulating();
        auto const show_islands = status_manager->is_showing_islands();
        auto flags = accumulating ? RayTracingFlag::RT_ACCUMULATE : RayTracingFlag::RT_NONE;
        flags |= show_islands ? RayTracingFlag::RT_SHOW_ISLANDS : RayTracingFlag::RT_NONE;
        ti.device.buffer_host_address_as<RayTracingConfig>(ti.get(task_ray_tracing_config_host).ids[0]).value()[0] = RayTracingConfig{
            .flags = flags,
            .max_bounces = MAX_BOUNCES,
            .current_frame_index = status_manager->get_frame_count(),
            .frame_count = accumulating ? status_manager->get_accumulation_count() : 0,
            .light_count = scene_manager->get_light_count(),
        };

        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = ti.get(task_ray_tracing_config_host).ids[0],
            .dst_buffer = ti.get(task_ray_tracing_config).ids[0],
            .size = sizeof(RayTracingConfig),
        });
      },
      .name = "copy rigid bodies and primitives",
  });

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
                  daxa::attachment_view(RayTracingTaskHead::AT.ray_tracing_config, task_ray_tracing_config),
                  daxa::attachment_view(RayTracingTaskHead::AT.swapchain, task_swapchain_image),
                  daxa::attachment_view(RayTracingTaskHead::AT.accumulation_buffer, task_accumulation_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.tlas, accel_struct_mngr->task_tlas),
                  daxa::attachment_view(RayTracingTaskHead::AT.rigid_bodies,
                                        rigid_body_manager->task_rigid_bodies),
                  daxa::attachment_view(RayTracingTaskHead::AT.aabbs, accel_struct_mngr->task_aabb_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.lights, scene_manager->task_lights_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.materials, scene_manager->task_material_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.islands, rigid_body_manager->task_islands),
                  daxa::attachment_view(RayTracingTaskHead::AT.contact_islands, rigid_body_manager->task_contact_islands),
              },
              user_callback);

  daxa::InlineTaskInfo task_cpy_to_accum_buffer({
      .attachments = {
          daxa::inl_attachment(daxa::TaskImageAccess::TRANSFER_READ, 
          task_swapchain_image),
          daxa::inl_attachment(daxa::TaskImageAccess::TRANSFER_WRITE, 
          task_accumulation_buffer),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        auto const accumulating = status_manager->is_accumulating();
        auto const frame_count = status_manager->get_accumulation_count();
        if (accumulating && frame_count == 0)
        {
          auto const image_info = ti.device.info_image(ti.get(task_swapchain_image).ids[0]).value();
          // zero out the accumulation buffer
          ti.recorder.clear_image({
              .dst_image_layout = daxa::ImageLayout::GENERAL,
              .clear_value = {std::array<f32, 4>{0, 0, 0, 0}},
              .dst_image = ti.get(task_accumulation_buffer).ids[0],
          });
        }
      },
      .name = "copy to accumulation buffer",
  });
  

  std::array<daxa::TaskBuffer, 12> buffers = {task_camera_buffer, rigid_body_manager->task_rigid_bodies, accel_struct_mngr->task_aabb_buffer,  gui_manager->task_vertex_buffer,
  gui_manager->task_line_vertex_buffer, gui_manager->task_axes_vertex_buffer,
  scene_manager->task_material_buffer, task_ray_tracing_config, task_ray_tracing_config_host, scene_manager->task_lights_buffer, 
  rigid_body_manager->task_islands, rigid_body_manager->task_contact_islands};

  std::array<daxa::TaskImage, 2> images = {task_swapchain_image, task_accumulation_buffer};

  std::array<daxa::TaskTlas, 1> tlases = {accel_struct_mngr->task_tlas};

  RT_TG = task_manager->create_task_graph(RT_TG_name, buffers, images, {}, tlases, true);

  RT_TG.add_task(task_update_RT_config);
  RT_TG.add_task(task_RT);
  RT_TG.add_task(task_cpy_to_accum_buffer);
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
  if (!initialized)
  {
    return;
  }

  for(auto f = 0u; f < DOUBLE_BUFFERING; f++) {
    gpu->device.destroy_buffer(ray_tracing_config_buffer[f]);
    gpu->device.destroy_buffer(ray_tracing_config_host_buffer[f]);
  }

  initialized = false;
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
  task_ray_tracing_config.set_buffers({.buffers = std::array{ray_tracing_config_buffer[get_frame_index()]}});
  task_ray_tracing_config_host.set_buffers({.buffers = std::array{ray_tracing_config_host_buffer[get_frame_index()]}});
  task_accumulation_buffer.set_images({.images = std::array{accumulation_buffer}});

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
      gpu->device.destroy_image(accumulation_buffer);
      // TODO: refactor this
      accumulation_buffer = gpu->device.create_image({
          .format = gpu->swapchain.get_format(),
          .size = daxa::Extent3D(gpu->swapchain_get_extent().x, gpu->swapchain_get_extent().y, 1),
          .usage = daxa::ImageUsageFlagBits::SHADER_STORAGE | daxa::ImageUsageFlagBits::TRANSFER_DST | daxa::ImageUsageFlagBits::TRANSFER_SRC,
          .name = "accumulation_buffer",
      });
      status_manager->reset_accumulation_count();
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