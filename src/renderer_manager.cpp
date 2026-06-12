#include "renderer_manager.hpp"
#include <iostream>

BB_NAMESPACE_BEGIN

RendererManager::RendererManager(std::shared_ptr<GPUcontext> gpu, std::shared_ptr<TaskManager> task_manager, WindowManager &window, std::shared_ptr<CameraManager> camera_manager, std::shared_ptr<AccelerationStructureManager> accel_struct_mngr, std::shared_ptr<RigidBodyManager> rigid_body_manager, std::shared_ptr<SceneManager> scene_manager, std::shared_ptr<StatusManager> status_manager, std::shared_ptr<GUIManager> gui_manager, std::shared_ptr<ImageManager> image_manager)
    : gpu(gpu), task_manager(task_manager), window(window), camera_manager(camera_manager), accel_struct_mngr(accel_struct_mngr), rigid_body_manager(rigid_body_manager), scene_manager(scene_manager), status_manager(status_manager), gui_manager(gui_manager), image_manager(image_manager) {}

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
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
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
        auto const show_normals = status_manager->is_showing_normals();
        auto const show_collisions = status_manager->is_showing_collisions();
        auto flags = accumulating ? RayTracingFlag::RT_ACCUMULATE : RayTracingFlag::RT_NONE;
        flags |= show_islands ? RayTracingFlag::RT_SHOW_ISLANDS : show_normals ? RayTracingFlag::RT_SHOW_NORMALS : show_collisions ? RayTracingFlag::RT_SHOW_COLLISIONS : RayTracingFlag::RT_NONE;
        ti.device.buffer_host_address_as<RayTracingConfig>(ti.get(task_ray_tracing_config_host).id).value()[0] = RayTracingConfig{
            .flags = flags,
            .max_bounces = MAX_BOUNCES,
            .current_frame_index = status_manager->get_frame_count(),
            .frame_count = accumulating ? status_manager->get_accumulation_count() : 0,
            .light_count = scene_manager->get_light_count(),
            .instance_count = rigid_body_manager->get_sim_config_reference().rigid_body_count,
        };

        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = ti.get(task_ray_tracing_config_host).id,
            .dst_buffer = ti.get(task_ray_tracing_config).id,
            .size = sizeof(RayTracingConfig),
        });
      },
      .name = "copy rigid bodies and primitives",
  });

  auto user_callback = [this, SBT](daxa::TaskInterface ti, auto &)
  {
    auto const image_info = ti.device.image_info(ti.get(RayTracingTaskHead::AT.swapchain).id).value();
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
                  daxa::attachment_view(RayTracingTaskHead::AT.rigid_body_map, rigid_body_manager->task_rigid_body_entries),
                  daxa::attachment_view(RayTracingTaskHead::AT.rigid_bodies,
                                        rigid_body_manager->task_rigid_bodies),
                  daxa::attachment_view(RayTracingTaskHead::AT.aabbs, accel_struct_mngr->task_aabb_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.lbvh_nodes, rigid_body_manager->task_lbvh_nodes),
                  daxa::attachment_view(RayTracingTaskHead::AT.lights, scene_manager->task_lights_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.materials, scene_manager->task_material_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.islands, rigid_body_manager->task_islands),
                  daxa::attachment_view(RayTracingTaskHead::AT.contact_islands, rigid_body_manager->task_contact_islands),
                  daxa::attachment_view(RayTracingTaskHead::AT.stbn_texture, task_stbn_texture),
              },
              user_callback);

  daxa::InlineTaskInfo task_cpy_to_accum_buffer({
      .attachments = {
          daxa::inl_attachment(daxa::TaskImageAccess::TRANSFER_WRITE, 
          task_accumulation_buffer),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        auto const accumulating = status_manager->is_accumulating();
        auto const frame_count = status_manager->get_accumulation_count();
        if (accumulating && frame_count == 0)
        {
          // zero out the accumulation buffer
          ti.recorder.clear_image({
              .image = ti.get(task_accumulation_buffer).id,
              .clear_value = {std::array<f32, 4>{0.0f, 0.0f, 0.0f, 0.0f}},
          });
        }
      },
      .name = "copy to accumulation buffer",
  });

  

  std::array<daxa::TaskBuffer, 14> buffers = {
    task_camera_buffer, 
    rigid_body_manager->task_rigid_body_entries,
    rigid_body_manager->task_rigid_bodies,
    accel_struct_mngr->task_aabb_buffer, 
    rigid_body_manager->task_lbvh_nodes, 
    gui_manager->task_vertex_buffer,
    gui_manager->task_line_vertex_buffer, 
    gui_manager->task_axes_vertex_buffer,
    scene_manager->task_material_buffer, 
    task_ray_tracing_config, 
    task_ray_tracing_config_host, 
    scene_manager->task_lights_buffer, 
    rigid_body_manager->task_islands, 
    rigid_body_manager->task_contact_islands};

  std::array<daxa::TaskImage, 3> images = {task_swapchain_image, task_accumulation_buffer, task_stbn_texture};

  std::array<daxa::TaskTlas, 1> tlases = {accel_struct_mngr->task_tlas};

  RT_TG = task_manager->create_task_graph(RT_TG_name, buffers, images, {}, tlases, true);

  RT_TG.add_task(task_update_RT_config);
  RT_TG.add_task(task_RT);
  RT_TG.add_task(task_cpy_to_accum_buffer);
  RT_TG.add_task(gui_manager->gui_axes_task_info);
  RT_TG.add_task(gui_manager->gui_line_task_info);
  RT_TG.add_task(gui_manager->gui_task_info);

  // the render submit waits the sim timeline: it consumes the latest sim+TLAS publication
  // produced on the async compute queue (value set per frame in execute())
  RT_TG.submit({.additional_wait_timeline_semaphores = &gpu->sim_wait_span});
  RT_TG.present();
  RT_TG.complete();

  // TODO: parameterize this
  if(!image_manager->upload_images()) {
    return false;
  }

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

  gpu->device.destroy_image(accumulation_buffer);

  initialized = false;
}

bool RendererManager::execute()
{
  if (!initialized)
  {
    return false;
  }
  gpu->sync_render_to_sim_timeline(); // wait the latest published sim+TLAS state
  RT_TG.execute();
  return true;
}

bool RendererManager::update_resources(daxa::ImageId swapchain_image, CameraManager &cam_mngr)
{
  if (!initialized)
  {
    return false;
  }

  task_swapchain_image.set_image(swapchain_image);
  task_camera_buffer.set_buffer(cam_mngr.camera_buffer);
  task_ray_tracing_config.set_buffer(ray_tracing_config_buffer[get_frame_index()]);
  task_ray_tracing_config_host.set_buffer(ray_tracing_config_host_buffer[get_frame_index()]);
  task_accumulation_buffer.set_image(accumulation_buffer);
  task_stbn_texture.set_image(image_manager->get_spatiotemporal_blue_noise_image());

  return true;
}

void RendererManager::render()
{
  double _sim_ms_accum = 0.0; daxa_u64 _sim_ms_n = 0;   // [PERF] isolated sim timing
  // Fixed-timestep simulation, decoupled from the render rate: the sim advances TIME_STEP
  // (1/60 s) of simulated time only when 1/60 s of real time has accumulated, so its speed is
  // real-time at any fps. The sim double buffers rotate per STEP (sim clock in StatusManager),
  // so the loop below can run several steps inside one render frame to CATCH UP when the
  // render dips under 60 fps. The accumulator is clamped so stalls don't queue a burst.
  auto sim_clock_prev = std::chrono::steady_clock::now();
  double sim_accum_s = 0.0;
  constexpr double SIM_DT_S = static_cast<double>(TIME_STEP);
  while (!window.should_close())
  {
    // Update the GUI
    gui_manager->update();

    if(rigid_body_manager->is_dirty()) {
      rigid_body_manager->clean_dirty();
      rigid_body_manager->update_sim();
    }

    // Simulate rigid bodies: fixed 60 Hz with REAL catch-up. The sim double buffers rotate
    // per STEP (begin_sim_step inside simulate()), so several steps inside one render frame
    // are legal; only the last step of the burst gets published (TLAS build below). Below
    // 60/MAX_CATCHUP_STEPS fps the sim slows down instead of spiraling.
    constexpr daxa_u32 MAX_CATCHUP_STEPS = 4u;
    daxa_u32 sim_steps_this_frame = 0u;
    {
      auto const sim_clock_now = std::chrono::steady_clock::now();
      double const elapsed_s = std::chrono::duration<double>(sim_clock_now - sim_clock_prev).count();
      sim_clock_prev = sim_clock_now;
      if (status_manager->is_simulating())
      {
        sim_accum_s = std::min(sim_accum_s + elapsed_s, (MAX_CATCHUP_STEPS + 1.0) * SIM_DT_S);
        while (sim_accum_s >= SIM_DT_S && sim_steps_this_frame < MAX_CATCHUP_STEPS)
        {
          sim_accum_s -= SIM_DT_S;
          gpu->synchronize();                                          // [PERF] flush prior GPU work
          auto _s0 = std::chrono::high_resolution_clock::now();        // [PERF]
          rigid_body_manager->simulate();
          gpu->synchronize();                                          // [PERF] wait sim GPU completion
          _sim_ms_accum += std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - _s0).count();  // [PERF]
          _sim_ms_n++;                                                 // [PERF]
          ++sim_steps_this_frame;
        }
      }
      else
      {
        sim_accum_s = 0.0; // don't burst-step on resume
      }
    }
    bool const sim_stepped = sim_steps_this_frame > 0u;
    if (!window.update())
      continue;

    // Update the acceleration structures (only when the sim actually stepped — skipped frames
    // render the unchanged state and avoid the full-pipeline synchronize + readback)
    if(sim_stepped || status_manager->is_updating()) {
      rigid_body_manager->read_back_sim_config();
      { static daxa_u64 _cf = 0; static auto _t0 = std::chrono::high_resolution_clock::now();
        // sample every 31 frames (odd) so the readback alternates between the two double-buffered
        // SimConfigs — each holds an independent dbg_ex latch; an even cadence would only ever show one.
        if ((++_cf % 31) == 0) {
          auto _t1 = std::chrono::high_resolution_clock::now();
          double _ms = std::chrono::duration<double, std::milli>(_t1 - _t0).count() / 31.0; _t0 = _t1;
          double _sim_ms = _sim_ms_n ? (_sim_ms_accum / (double)_sim_ms_n) : 0.0;   // [PERF]
          _sim_ms_accum = 0.0; _sim_ms_n = 0;                                        // [PERF]
          // deep-MISS walk diagnostic decode (see BodyLinkManifold::walk_diag)
          auto dm_walk_str = [](daxa_u32 w) {
            static char const *reasons[4] = {"EMPTY", "IDBRK", "MAPBRK", "END"};
            std::string s = reasons[(w >> 30u) & 3u];
            if ((w >> 29u) & 1u) s += "+FOUND";
            s += " s" + std::to_string((w >> 16u) & 0x1FFFu) + " o" + std::to_string(w & 0xFFFFu);
            return s;
          };
          auto const &sc = rigid_body_manager->get_sim_config_reference();
          std::cout << "[PERF] step=" << sc.frame_count << " manifolds=" << sc.g_c_info.collision_count
                    << " sleeping=" << sc.sleeping_count
                    << " avbdc=" << sc.avbd_color_count << " avbdv=" << sc.avbd_violations
                    << " astick=" << sc.avbd_stick_count
                    << " colors=" << sc.graph_color_count << " violations=" << sc.graph_color_violations
                    << " overflow=" << sc.graph_color_overflow
                    << " maxdeg=" << sc.gc_max_degree
                    << " satbody=" << sc.gc_max_degree_body << " satflags=" << sc.gc_max_degree_flags
                    << " satdeg=" << sc.gc_satbody_degree << " satunc=" << sc.gc_satbody_uncolored
                    << " p=[" << sc.gc_satbody_pmin << "," << sc.gc_satbody_pmax << "]"
                    << " nan=" << sc.gc_sat_nanflags << " y=" << sc.gc_sat_pos_y
                    << " maxv=" << sc.dbg_maxv
                    << " fresh=" << sc.dbg_fresh
                    << " ftag=[" << ((sc.dbg_fresh_tag >> 22u) & 0x1FFu) << "," << ((sc.dbg_fresh_tag >> 12u) & 0x3FFu)
                    << " k" << ((sc.dbg_fresh_tag >> 4u) & 0xFFu) << " n" << (sc.dbg_fresh_tag & 0xFu)
                    << (sc.dbg_fresh_tag >> 31u ? " MISS" : "") << "]"
                    << " fa=[f" << ((sc.dbg_pad1 >> 28u) & 0xFu) << " n" << ((sc.dbg_pad1 >> 24u) & 0xFu)
                    << " o" << (((sc.dbg_pad1 >> 23u) & 1u) ? std::to_string((sc.dbg_pad1 >> 19u) & 0xFu) : std::string("-"))
                    << " " << ((sc.dbg_pad1 >> 10u) & 0x1FFu) << "," << (sc.dbg_pad1 & 0x3FFu) << "]"
                    << " dm=[n" << sc.dbg_dm_count << " mon" << sc.dbg_dm_mon
                    << " " << ((sc.dbg_dm_ids >> 16u) & 0xFFFFu) << "," << (sc.dbg_dm_ids & 0xFFFFu)
                    << " A:" << dm_walk_str(sc.dbg_dm_walk_a) << " B:" << dm_walk_str(sc.dbg_dm_walk_b) << "]"
                    << " np=" << sc.dbg_np_processed << "/" << sc.broad_phase_collision_count
                    << std::hex << " ph=" << sc.dbg_poshash << " rh=" << sc.dbg_rothash << std::dec
                    << " pen=" << sc.dbg_pen
                    << " idsum=" << (daxa_i64)sc.dbg_id_sum - (daxa_i64)((daxa_u64)sc.rigid_body_count * (sc.rigid_body_count - 1) / 2)
                    << " EX[s=" << sc.dbg_ex_stage << " b=" << sc.dbg_ex_body << " f=" << sc.dbg_ex_frame
                    << " v=" << sc.dbg_ex_vel << " y=" << sc.dbg_ex_y << " vy=" << sc.dbg_ex_vy << "]"
                    << " | frame=" << _ms << " ms (" << (1000.0 / _ms) << " fps)"
                    << "  sim=" << _sim_ms << " ms" << std::endl; } }
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
      // TODO: re-enable hot-reload once migration is stable
      // handle_reload_result(task_manager->reload(), RT_pipeline, this);
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

daxa_u32 RendererManager::get_sim_frame_index()
{
  return status_manager->get_sim_frame_index();
}

daxa_u32 RendererManager::get_sim_next_frame_index()
{
  return (status_manager->get_sim_frame_index() + 1) % DOUBLE_BUFFERING;
}

daxa_u32 RendererManager::get_sim_previous_frame_index()
{
  return (status_manager->get_sim_frame_index() + DOUBLE_BUFFERING - 1) % DOUBLE_BUFFERING;
}

void RendererManager::begin_sim_step()
{
  status_manager->begin_sim_step();
}

daxa_u32 RendererManager::get_next_frame_index()
{
  return (status_manager->get_frame_index() + 1) % DOUBLE_BUFFERING;
}

BB_NAMESPACE_END
