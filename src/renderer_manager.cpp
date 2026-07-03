#define _CRT_SECURE_NO_WARNINGS // std::getenv (BB_RUN_SECONDS) on MSVC
#include "renderer_manager.hpp"
#include "scene_manager.hpp" // forward-declared in the header (rebuild-cascade cut, review v3)
#include <iostream>
#include <fstream> // deep-pocket trace CSV (diagnostic)
#include <cstdlib> // std::getenv / std::atof (BB_RUN_SECONDS auto-exit)

BB_NAMESPACE_BEGIN

RendererManager::RendererManager(std::shared_ptr<GPUcontext> gpu, std::shared_ptr<TaskManager> task_manager, WindowManager &window, std::shared_ptr<CameraManager> camera_manager, std::shared_ptr<AccelerationStructureManager> accel_struct_mngr, std::shared_ptr<RigidBodyManager> rigid_body_manager, std::shared_ptr<SceneManager> scene_manager, std::shared_ptr<StatusManager> status_manager, std::shared_ptr<GUIManager> gui_manager, std::shared_ptr<ImageManager> image_manager)
    : gpu(gpu), task_manager(task_manager), window(window), camera_manager(camera_manager), accel_struct_mngr(accel_struct_mngr), rigid_body_manager(rigid_body_manager), scene_manager(scene_manager), status_manager(status_manager), gui_manager(gui_manager), image_manager(image_manager) {}

// SceneManager getters live here (not inline in the header) so the 1200+-line scene_manager.hpp
// stays out of every TU that includes renderer_manager.hpp (rebuild-cascade cut, review v3)
daxa_u32 RendererManager::get_rigid_body_count() { return scene_manager->get_rigid_body_count(); }
daxa_u32 RendererManager::get_active_rigid_body_count() { return scene_manager->get_active_rigid_body_count(); }
std::vector<ActiveRigidBody> RendererManager::get_active_rigid_bodies() { return scene_manager->get_active_rigid_bodies(); }

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

int RendererManager::render()
{
  double _sim_ms_accum = 0.0; daxa_u64 _sim_ms_n = 0;   // [PERF] isolated sim timing
  // Fixed-timestep simulation, decoupled from the render rate: the sim advances TIME_STEP
  // (1/60 s) of simulated time only when 1/60 s of real time has accumulated, so its speed is
  // real-time at any fps. The sim double buffers rotate per STEP (sim clock in StatusManager),
  // so the loop below can run several steps inside one render frame to CATCH UP when the
  // render dips under 60 fps. The accumulator is clamped so stalls don't queue a burst.
  auto sim_clock_prev = std::chrono::steady_clock::now();
  double sim_accum_s = 0.0;
  int gui_hitch_cooldown = 0;                               // suppress the sim catch-up BURST for a
  bool prev_gui_enabled = status_manager->is_gui_enabled(); // couple of frames after a GUI (TAB) toggle
  constexpr double SIM_DT_S = static_cast<double>(TIME_STEP);
  // optional auto-exit (env BB_RUN_SECONDS=N): close the app cleanly after N wall-clock seconds, so a
  // captured [PERF] log self-terminates and A/B solver measurement runs are reproducible. 0 = no limit.
  double run_limit_s = 0.0;
  if (const char *e = std::getenv("BB_RUN_SECONDS")) run_limit_s = std::atof(e);
  // DETERMINISM debug mode (BB_DET_STEPS=N): exactly one sim step per render frame, wall-clock
  // ignored, so two runs execute a bit-identical step sequence (isolates kernel races from the
  // real-time catch-up). Prints every step's pose hash; exits after N steps. 0 = off.
  int det_steps = 0;
  if (const char *e = std::getenv("BB_DET_STEPS")) det_steps = std::atoi(e);
  bool det_inited = false; daxa_u32 det_count = 0u;
  // BB_DET_INPROC=1: after N steps, reset to the IDENTICAL initial state and run N again IN THE SAME
  // PROCESS, comparing a cumulative path-hash. If the two passes DIFFER, the AVBD solve is genuinely
  // GPU-kernel non-deterministic (same process, GPU, buffer addresses); if they MATCH but cross-
  // process diverges, the source is process-specific (addresses/driver state).
  bool det_inproc = std::getenv("BB_DET_INPROC") != nullptr;
  int det_pass = 1; daxa_u32 det_hashA = 0u; daxa_u32 det_acc = 0u;
  if (det_steps > 0) { status_manager->request_scene(3); } // self-contained: load scene_3 fresh
  auto const run_start = std::chrono::steady_clock::now();
  // The acceleration-structure build runs async on COMPUTE_0, and the render graph waits the SIM
  // timeline (sim_wait_span) -- which is ONLY advanced/signalled by a sim step, never by the scene-load
  // AS build. So after a scene load/switch the render never waits for the build and traverses an
  // in-flight BLAS -> cubes render with rounded ("dented") corners until the first sim step. Run ONE
  // real sim step on scene load/switch: it publishes the AS through the render-synced timeline path
  // (the only thing that reliably fixes it). Bodies advance one 1/60s step (~3mm of gravity --
  // imperceptible; the pool is floating mid-air at rest anyway).
  // C1 HEADLESS METRICS (env-gated). The ground-truth quality metrics (dbg_pen/deep100/deep200/
  // maxv/min_y) are computed every sim step UNCONDITIONALLY in the narrow phase, so this needs no
  // extra flag. BB_METRICS_CSV=path writes one clean CSV row per stepped frame; BB_ASSERT_MAX_DEEP200
  // / BB_ASSERT_MAX_PEN / BB_ASSERT_MAX_MAXV fail the run (exit code 2) if the metric exceeds the
  // threshold after BB_ASSERT_AFTER warmup steps (default 60). Turns "run and eyeball" into a
  // scriptable per-solver A/B with a real exit code — pair with BB_SOLVER / BB_AUTOSTART / BB_RUN_SECONDS.
  std::ofstream metrics_csv;
  bool metrics_hdr = false;
  if (const char *e = std::getenv("BB_METRICS_CSV")) metrics_csv.open(e, std::ios::trunc);
  double assert_max_deep200 = -1.0, assert_max_pen = -1.0, assert_max_maxv = -1.0;
  if (const char *e = std::getenv("BB_ASSERT_MAX_DEEP200")) assert_max_deep200 = std::atof(e);
  if (const char *e = std::getenv("BB_ASSERT_MAX_PEN"))     assert_max_pen     = std::atof(e);
  if (const char *e = std::getenv("BB_ASSERT_MAX_MAXV"))    assert_max_maxv    = std::atof(e);
  // floor-escape / explosion gate: fail when the LOWEST body sinks below this y (meters). miny is
  // already read back + printed; without this assert an explosion that launches bodies through the
  // floor passed every MAX_* gate (pen/deep can look fine while a body free-falls at -50m).
  double assert_min_miny = -1e30;
  bool assert_min_miny_on = false;
  if (const char *e = std::getenv("BB_ASSERT_MIN_MINY")) { assert_min_miny = std::atof(e); assert_min_miny_on = true; }
  daxa_u64 assert_after = 60u;
  if (const char *e = std::getenv("BB_ASSERT_AFTER")) assert_after = static_cast<daxa_u64>(std::atoll(e));
  int metrics_exit_code = 0;
  // BB_DUMP_AT_SECONDS=N: headless F9 - dump the LIVE poses once, N seconds into the run
  // (no window focus needed; SendKeys-based captures race against the user's foreground)
  double dump_at_s = -1.0;
  if (const char *e = std::getenv("BB_DUMP_AT_SECONDS")) dump_at_s = std::atof(e);
  bool dump_at_done = false;

  bool force_sim_step = true;
  while (!window.should_close())
  {
    if (run_limit_s > 0.0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - run_start).count() > run_limit_s)
    {
      std::cout << "[PERF] BB_RUN_SECONDS=" << run_limit_s << " elapsed -> exiting." << std::endl;
      break;
    }
    // Update the GUI
    gui_manager->update();

    // A GUI toggle (TAB) rebuilds the ImGui overlay (+ the contact-point debug pass), hitching this
    // frame and the next; suppress the sim's REAL catch-up for those frames so the (AVBD-jittering)
    // pile doesn't advance several steps at once and visibly jerk. Same intent as the reset clamp.
    {
      bool const cur_gui = status_manager->is_gui_enabled();
      if (cur_gui != prev_gui_enabled) { gui_hitch_cooldown = 2; }
      prev_gui_enabled = cur_gui;
    }

    // reset request (key R): restart the sim from the initial scene at this frame boundary (prior
    // GPU work is already synchronized here), and clear the catch-up accumulator so it doesn't burst.
    if (status_manager->consume_reset()) {
      scene_manager->reset();
      sim_accum_s = 0.0;
      force_sim_step = true;
    }

    // scene switch request (F1-F8): rebuild from the chosen scene at this same frame boundary,
    // paused, and clear the catch-up accumulator so it doesn't burst on the first resumed step.
    if (int const requested_scene = status_manager->consume_scene(); requested_scene >= 0) {
      scene_manager->switch_scene(requested_scene);
      sim_accum_s = 0.0;
      force_sim_step = true;
    }

    // live scene dump request (F9): capture the CURRENT GPU poses to a scene file at this
    // frame boundary (safe to one-off copy + wait here; it's a debug capture path)
    if (status_manager->consume_dump()) {
      scene_manager->dump_scene_live("scene_dump.txt");
    }
    if (dump_at_s > 0.0 && !dump_at_done &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - run_start).count() >= dump_at_s) {
      dump_at_done = true;
      scene_manager->dump_scene_live("scene_dump.txt");
    }

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
    if (det_steps > 0)
    {
      // Deterministic stepping: one step per frame, wall-clock ignored, scene_3 self-loaded fresh.
      // Two runs thus execute a bit-identical step sequence from an identical initial state; any
      // divergence is a true kernel race, not real-time pacing. Local counter = same step ids both runs.
      if (!det_inited)
      {
        // iter 1: scene_3 was just consumed/loaded at the top of this frame; enable simulating and
        // begin counting from the fresh initial state on the NEXT frame.
        if (!status_manager->is_simulating()) { status_manager->switch_simulating(); }
        det_inited = true;
      }
      else
      {
        gpu->synchronize();
        rigid_body_manager->simulate();
        gpu->synchronize();
        sim_steps_this_frame = 1u;
        ++det_count;
        rigid_body_manager->read_back_sim_config();
        auto const &dsc = rigid_body_manager->get_sim_config_reference();
        std::cout << "DET step=" << det_count << std::hex << " ph=" << dsc.dbg_poshash
                  << " rh=" << dsc.dbg_rothash << " cp2=" << dsc.dbg_cp2_poshash
                  << " cph=" << dsc.dbg_cp_poshash << " vhf=" << dsc.dbg_vh_fin
                  << " vhi=" << dsc.dbg_vh_imp << " chash=" << dsc.dbg_color_hash
                  << " lh=" << dsc.dbg_color_pad << " sh=" << dsc.dbg_state_hash
                  << " wh=" << dsc.dbg_state_pad
                  << std::dec << " viol=" << dsc.avbd_violations << std::endl;
        det_acc = det_acc * 0x9e3779b9u + dsc.dbg_poshash; // cumulative path hash (catches transient divergence)
        if (det_count >= (daxa_u32)det_steps)
        {
          if (det_inproc && det_pass == 1)
          {
            det_hashA = det_acc;
            std::cout << "INPROC passA acc=" << std::hex << det_acc << std::dec << " -> reset + replay" << std::endl;
            scene_manager->reset();   // reload scene_3 to the IDENTICAL initial state
            sim_accum_s = 0.0;
            det_count = 0u; det_acc = 0u; det_pass = 2; det_inited = false; // re-arm for pass 2
          }
          else
          {
            if (det_inproc)
            {
              std::cout << "INPROC A=" << std::hex << det_hashA << " B=" << det_acc << std::dec
                        << ((det_hashA == det_acc) ? "  => IN-PROCESS MATCH (deterministic; cross-process source)"
                                                   : "  => IN-PROCESS DIFF (GPU-kernel non-determinism)") << std::endl;
            }
            break;
          }
        }
      }
    }
    else
    {
      auto const sim_clock_now = std::chrono::steady_clock::now();
      double const elapsed_s = std::chrono::duration<double>(sim_clock_now - sim_clock_prev).count();
      sim_clock_prev = sim_clock_now;
      // MOUSE PICK-AND-DRAG input (once per render frame, BEFORE the sim steps consume it):
      // build the camera ray under the cursor — the same math as create_ray() in the raygen
      // shader (shared.inl) — and hand it to the GPU pick/spring pass with the button edges.
      // LEFT button: grab + drag a body (statics occlude the ray; empty space grabs nothing).
      // Camera orbit lives on the RIGHT button (input_manager), so there is no conflict.
      {
        auto &cam = camera_manager->camera;
        bool const left_held = glfwGetMouseButton(window.glfw_window_ptr, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        static bool prev_left = false;
        f64 cx = 0.0, cy = 0.0;
        glfwGetCursorPos(window.glfw_window_ptr, &cx, &cy);
        glm::vec2 const pixel_center = glm::vec2(static_cast<f32>(cx), static_cast<f32>(cy)) + 0.5f;
        glm::vec2 const inv_uv = pixel_center / glm::vec2(static_cast<f32>(window.width), static_cast<f32>(window.height));
        glm::vec2 const d = inv_uv * 2.0f - 1.0f;
        glm::mat4 const inv_view = _get_inverse_view_matrix(cam);
        glm::mat4 const inv_proj = _get_inverse_projection_matrix(cam, true);
        glm::vec4 const origin = inv_view * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
        glm::vec4 const target = inv_proj * glm::vec4(d.x, d.y, 1.0f, 1.0f);
        glm::vec4 const direction = inv_view * glm::vec4(glm::normalize(glm::vec3(target)), 0.0f);
        rigid_body_manager->set_pick_input(
            daxa_f32vec3(origin.x, origin.y, origin.z),
            daxa_f32vec3(direction.x, direction.y, direction.z),
            left_held && !prev_left, left_held);
        // BB_PICK_TRACE: one stderr line per frame while the button is involved — which body id is
        // grabbed, and whether the request edge fired. Diagnostic for "the drag touches other boxes".
        static bool const _pick_trace = std::getenv("BB_PICK_TRACE") != nullptr;
        if (_pick_trace && (left_held || prev_left))
        {
          std::cerr << "[PICK] held=" << left_held << " edge=" << (left_held && !prev_left)
                    << " picked_id=" << rigid_body_manager->get_picked_body()
                    << " grabs=" << rigid_body_manager->get_grab_count()
                    << " cursor=" << cx << "," << cy << std::endl;
        }
        prev_left = left_held;
      }
      // ONE forced step after a scene load/switch to publish the async AS through the render-synced
      // timeline path (cures the at-rest "dented/rounded cubes"). At rest is_simulating() is false, so
      // only this runs; sim_steps_this_frame=1 makes the AS-update block below rebuild + signal.
      if (force_sim_step)
      {
        gpu->synchronize();
        rigid_body_manager->simulate();
        gpu->synchronize();
        sim_steps_this_frame = 1u;
        force_sim_step = false;
      }
      // FULL-SLEEP STASIS: when every active body sleeps, a step is an identity - sleepers
      // skip advect/solve/integrate and nothing can wake them without a host-visible event
      // (from full sleep there IS no moving partner; the only wake sources are the pick, a
      // reset or a scene switch, all of which force stepping). Skipping the whole GPU
      // pipeline drops the at-rest sim cost to zero. The left mouse button disables the
      // skip so the pick pass runs and can grab/wake; the readback is from the last stepped
      // frame, which stays valid precisely because nothing steps.
      bool const stasis = [&] {
        auto const &ssc = rigid_body_manager->get_sim_config_reference();
        if (ssc.active_rigid_body_count == 0u ||
            ssc.sleeping_count < ssc.active_rigid_body_count) { return false; }
        return glfwGetMouseButton(window.glfw_window_ptr, GLFW_MOUSE_BUTTON_LEFT) != GLFW_PRESS;
      }();
      if (status_manager->is_simulating() && !stasis)
      {
        // during a GUI-toggle hitch, cap the accumulator to ONE step (no burst -> no jerk); the few
        // ms of lost real-time sync over the toggle is imperceptible and resyncs once cooldown ends.
        double const accum_cap = gui_hitch_cooldown > 0 ? SIM_DT_S : (MAX_CATCHUP_STEPS + 1.0) * SIM_DT_S;
        if (gui_hitch_cooldown > 0) { --gui_hitch_cooldown; }
        sim_accum_s = std::min(sim_accum_s + elapsed_s, accum_cap);
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
      // DEEP-POCKET TRACE: one CSV row per stepped frame with the deepest awake contact's
      // {pen,lambda,k,vn,pair,stick} latched by entry_avbd_pocket_trace. Full-rate (every
      // frame) so a 1-frame-period oscillation isn't aliased. Truncates at startup.
      // C3: env-gated + portable. BB_POCKET_TRACE=path enables the trace at that path; OFF by default,
      // so a normal run pays no per-frame disk I/O and there is no hardcoded C:/ path that fails on
      // other machines. Read once (static); the ofstream opens lazily only when the trace is enabled.
      static char const *_pk_path = std::getenv("BB_POCKET_TRACE");
      if (sim_stepped && _pk_path) {
        static std::ofstream _pk(_pk_path, std::ios::trunc);
        static bool _pk_hdr = false;
        auto const &pk = rigid_body_manager->get_sim_config_reference();
        if (!_pk_hdr) { _pk << "frame,pk_pen_mm,pk_lambda,pk_k,pk_vn,b1,b2,cc,stick,global_pen_mm,maxv_mm,omega_mrad,manifolds,sleeping\n"; _pk_hdr = true; }
        _pk << (daxa_u64)pk.frame_count
            << "," << pk.dbg_pk_pen << "," << pk.dbg_pk_lambda << "," << pk.dbg_pk_k << "," << pk.dbg_pk_vn
            << "," << (pk.dbg_pk_body >> 16) << "," << (pk.dbg_pk_body & 0xFFFFu)
            << "," << (pk.dbg_pk_stick >> 1) << "," << (pk.dbg_pk_stick & 1u)
            << "," << pk.dbg_pen << "," << pk.dbg_maxv << "," << pk.dbg_pk_omega
            << "," << pk.g_c_info.collision_count << "," << pk.sleeping_count << "\n";
        _pk.flush();
      }
      // C1 headless metrics: one CSV row per stepped frame + threshold asserts (env-gated).
      if (sim_stepped && (metrics_csv.is_open() || assert_max_deep200 >= 0.0 || assert_max_pen >= 0.0 || assert_max_maxv >= 0.0 || assert_min_miny_on))
      {
        auto const &mc = rigid_body_manager->get_sim_config_reference();
        double miny = (mc.dbg_min_y == 0xFFFFFFFFu) ? 0.0 : (double)mc.dbg_min_y / 1000.0 - 100.0;
        if (metrics_csv.is_open())
        {
          if (!metrics_hdr) { metrics_csv << "frame,solver,manifolds,sleeping,pen_mm,maxv_mm,miny_m,deep100,deep200\n"; metrics_hdr = true; }
          metrics_csv << (daxa_u64)mc.frame_count << "," << (daxa_u32)mc.solver_type
                      << "," << mc.g_c_info.collision_count << "," << mc.sleeping_count
                      << "," << mc.dbg_pen << "," << mc.dbg_maxv << "," << miny
                      << "," << mc.dbg_deep100 << "," << mc.dbg_deep200 << "\n";
          metrics_csv.flush();
        }
        if ((daxa_u64)mc.frame_count >= assert_after)
        {
          char const *which = nullptr; double val = 0.0, lim = 0.0;
          if (assert_max_deep200 >= 0.0 && (double)mc.dbg_deep200 > assert_max_deep200) { which = "deep200"; val = mc.dbg_deep200; lim = assert_max_deep200; }
          else if (assert_max_pen >= 0.0 && (double)mc.dbg_pen > assert_max_pen)         { which = "pen_mm";  val = mc.dbg_pen;     lim = assert_max_pen; }
          else if (assert_max_maxv >= 0.0 && (double)mc.dbg_maxv > assert_max_maxv)      { which = "maxv_mm"; val = mc.dbg_maxv;    lim = assert_max_maxv; }
          else if (assert_min_miny_on && miny < assert_min_miny)                          { which = "miny_m (floor escape)"; val = miny; lim = assert_min_miny; }
          if (which)
          {
            std::cerr << "[METRICS] ASSERT FAILED: " << which << "=" << val << " breached limit " << lim
                      << " at step " << (daxa_u64)mc.frame_count << " (solver=" << (daxa_u32)mc.solver_type << ")" << std::endl;
            metrics_exit_code = 2;
            break;
          }
        }
      }
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
                    << " contact_of=" << sc.dbg_contact_overflow << " node_of=" << sc.dbg_node_overflow
                    << " sleeping=" << sc.sleeping_count
                    << " avbdc=" << sc.avbd_color_count << " maxd=" << sc.avbd_max_support_depth << " avbdv=" << sc.avbd_violations
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
                    << " miss=[p" << sc.dbg_miss_present << " a" << sc.dbg_miss_absent
                    << " c" << sc.dbg_miss_corrupt << " e" << sc.dbg_miss_emptyhead << "]"
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
                    << std::hex << " ph=" << sc.dbg_poshash << " rh=" << sc.dbg_rothash
                    << " cph=" << sc.dbg_cp_poshash << " crh=" << sc.dbg_cp_rothash
                    << " c2ph=" << sc.dbg_cp2_poshash << " c2rh=" << sc.dbg_cp2_rothash
                    << " vhf=" << sc.dbg_vh_fin << " vhi=" << sc.dbg_vh_imp << std::dec
                    << " vox=" << sc.dbg_vox_interior << " wedge=" << sc.dbg_vox_wedge
                    << " pen=" << sc.dbg_pen
                    << " miny=" << (sc.dbg_min_y == 0xFFFFFFFFu ? 0.0 : (double)sc.dbg_min_y / 1000.0 - 100.0)
                    << " deep100=" << sc.dbg_deep100 << " deep200=" << sc.dbg_deep200
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
  if (metrics_csv.is_open()) metrics_csv.flush();
  return metrics_exit_code;
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
