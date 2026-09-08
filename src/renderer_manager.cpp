#include "render_pacing.hpp"
#define _CRT_SECURE_NO_WARNINGS // std::getenv (BB_RUN_SECONDS) on MSVC
#include "renderer_manager.hpp"
#include "runtime_diagnostics.hpp"
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

  snapshot.create(*task_manager, {rigid_body_manager->task_rigid_bodies,
      rigid_body_manager->task_rigid_body_entries, rigid_body_manager->task_lbvh_nodes,
      rigid_body_manager->task_islands, rigid_body_manager->task_contact_islands,
      gui_manager->task_vertex_buffer, gui_manager->task_line_vertex_buffer,
      gui_manager->task_axes_vertex_buffer, rigid_body_manager->task_sim_config});

  performance.create(gpu->device, gpu->swapchain.get_format());
  frame_timer.create(gpu->device, "Render GPU HUD timestamps");

  render_timing = std::getenv("BB_FRAME_TIMING") != nullptr;
  if (render_timing)
    render_queries = gpu->device.create_timeline_query_pool({.query_count=2,.name="ray_tracing_timing"});

  // A stable task resource retains cross-frame dependencies. Uploads are staged
  // per execution; the device buffer is reused only through ordered graph access.
  ray_tracing_config_buffer = gpu->device.create_buffer({
      .size = sizeof(RayTracingConfig),
      .name = "ray_tracing_config",
  });

  RT_pipeline = pipeline;

  // render scale (BB_RENDER_SCALE=0.25..1.0): trace at a reduced resolution and upscale.
  // Path tracing cost is ~linear in pixels: 0.75 = ~44% fewer rays, 0.5 = 75% fewer.
  // The shader derives UVs from DispatchRaysDimensions(), so it scales transparently.
  if (const char *e = std::getenv("BB_RENDER_SCALE"))
  {
    render_scale = std::clamp((f32)std::atof(e), 0.25f, 1.0f);
    std::cout << "[RENDER] BB_RENDER_SCALE=" << render_scale << std::endl;
  }
  auto scaled_extent = [this]() -> daxa::Extent3D {
    auto const ext = gpu->swapchain_get_extent();
    return daxa::Extent3D(std::max(1u, (daxa_u32)(ext.x * render_scale)),
                          std::max(1u, (daxa_u32)(ext.y * render_scale)), 1);
  };

  // the accumulation buffer matches the TRACE resolution (accumulation happens pre-upscale).
  // FORMAT: RGBA32F (RGB mean + per-pixel sample count), NOT the swapchain's 8-bit UNORM - the buffer stores LINEAR HDR radiance
  // (tonemap happens after averaging, PT batch 2), and UNORM storage CLAMPED every value >1.0
  // and quantized darks to 1/255: accumulated images came out dimmer/flatter than the live
  // frame with banding (user-reported "el buffer de acumulación no funciona bien").
  accumulation_buffer = gpu->device.create_image({
      .format = daxa::Format::R32G32B32A32_SFLOAT,
      .size = scaled_extent(),
      .usage = daxa::ImageUsageFlagBits::SHADER_STORAGE | daxa::ImageUsageFlagBits::TRANSFER_DST | daxa::ImageUsageFlagBits::TRANSFER_SRC,
      .name = "accumulation_buffer",
  });
  // offscreen trace target (1x1 dummy at scale 1.0, where the old direct-to-swapchain
  // wiring stays in effect and the blit task is not recorded)
  rt_target_image = gpu->device.create_image({
      .format = gpu->swapchain.get_format(),
      .size = render_scale < 1.0f ? scaled_extent() : daxa::Extent3D(1, 1, 1),
      .usage = daxa::ImageUsageFlagBits::SHADER_STORAGE | daxa::ImageUsageFlagBits::TRANSFER_SRC | daxa::ImageUsageFlagBits::TRANSFER_DST,
      .name = "rt_target",
  });

  daxa::InlineTaskInfo task_update_RT_config({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE,
          task_camera_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE,
           task_ray_tracing_config),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        frame_timer.begin(ti.recorder);
        auto const accumulating = status_manager->is_accumulating();
        auto const show_islands = status_manager->is_showing_islands();
        auto const show_normals = status_manager->is_showing_normals();
        auto const show_collisions = status_manager->is_showing_collisions();
        static bool const validate_rt = beat_box_diagnostics::parse_boolean("BB_RT_VALIDATE", std::getenv("BB_RT_VALIDATE"));
        auto flags = accumulating ? RayTracingFlag::RT_ACCUMULATE : RayTracingFlag::RT_NONE;
        if (validate_rt) flags |= RayTracingFlag::RT_VALIDATE;
        flags |= show_islands ? RayTracingFlag::RT_SHOW_ISLANDS : show_normals ? RayTracingFlag::RT_SHOW_NORMALS : show_collisions ? RayTracingFlag::RT_SHOW_COLLISIONS : RayTracingFlag::RT_NONE;
        // Per-execution staging allocations stay live until their GPU submit
        // completes. Task accesses order uploads after earlier shader readers.
        allocate_fill_copy(ti, camera_manager->camera_view, ti.get(task_camera_buffer));
        allocate_fill_copy(ti, RayTracingConfig{
            .flags = flags,
            .max_bounces = MAX_BOUNCES,
            .current_frame_index = status_manager->get_frame_count(),
            .frame_count = accumulating ? status_manager->get_accumulation_count() : 0,
            .light_count = scene_manager->get_light_count(),
            .instance_count = rigid_body_manager->render_config.rigid_body_count,
        }, ti.get(task_ray_tracing_config));
      },
      .name = "upload camera and ray tracing config",
  });

  auto user_callback = [this, SBT](daxa::TaskInterface ti, auto &)
  {
    auto const image_info = ti.device.image_info(ti.get(RayTracingTaskHead::AT.swapchain).id).value();
    ti.recorder.set_pipeline(*RT_pipeline->pipeline);
    ti.recorder.push_constant(RTPushConstants{.task_head = ti.attachment_shader_blob});
    bool const measure = render_timing && !render_query_pending;
    if (measure) {
      ti.recorder.reset_timestamps({.query_pool=render_queries,.start_index=0,.count=2});
      ti.recorder.write_timestamp({.query_pool=render_queries,.pipeline_stage=daxa::PipelineStageFlagBits::ALL_COMMANDS,.query_index=0});
    }
    ti.recorder.trace_rays({
        .width = image_info.size.x,
        .height = image_info.size.y,
        .depth = 1,
        .shader_binding_table = SBT,
    });
    if (measure) {
      ti.recorder.write_timestamp({.query_pool=render_queries,.pipeline_stage=daxa::PipelineStageFlagBits::ALL_COMMANDS,.query_index=1});
      render_query_pending=true;
    }
  };

  using TTask = TaskTemplate<RayTracingTaskHead::Task, decltype(user_callback)>;

  // Instantiate the task using the template class. With render scale active the "swapchain"
  // attachment is the scaled offscreen target; the upscale blit brings it to the swapchain.
  daxa::TaskImage rt_output = render_scale < 1.0f ? task_rt_target : task_swapchain_image;
  TTask task_RT(std::array{
                  daxa::attachment_view(RayTracingTaskHead::AT.camera, task_camera_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.ray_tracing_config, task_ray_tracing_config),
                  daxa::attachment_view(RayTracingTaskHead::AT.swapchain, rt_output),
                  daxa::attachment_view(RayTracingTaskHead::AT.accumulation_buffer, task_accumulation_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.tlas, accel_struct_mngr->task_tlas),
                  daxa::attachment_view(RayTracingTaskHead::AT.rigid_body_map, snapshot.buffers[RenderSnapshot::BODY_MAP]),
                  daxa::attachment_view(RayTracingTaskHead::AT.rigid_bodies,
                                        snapshot.buffers[RenderSnapshot::BODIES]),
                  daxa::attachment_view(RayTracingTaskHead::AT.aabbs, accel_struct_mngr->task_aabb_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.lbvh_nodes, snapshot.buffers[RenderSnapshot::BVH]),
                  daxa::attachment_view(RayTracingTaskHead::AT.lights, scene_manager->task_lights_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.materials, scene_manager->task_material_buffer),
                  daxa::attachment_view(RayTracingTaskHead::AT.islands, snapshot.buffers[RenderSnapshot::ISLANDS]),
                  daxa::attachment_view(RayTracingTaskHead::AT.contact_islands, snapshot.buffers[RenderSnapshot::CONTACT_ISLANDS]),
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



  std::array<daxa::TaskBuffer, 13> buffers = {
    task_camera_buffer,
    snapshot.buffers[RenderSnapshot::BODY_MAP],
    snapshot.buffers[RenderSnapshot::BODIES],
    accel_struct_mngr->task_aabb_buffer,
    snapshot.buffers[RenderSnapshot::BVH],
    snapshot.buffers[RenderSnapshot::POINTS],
    snapshot.buffers[RenderSnapshot::LINES],
    snapshot.buffers[RenderSnapshot::AXES],
    scene_manager->task_material_buffer,
    task_ray_tracing_config,
    scene_manager->task_lights_buffer,
    snapshot.buffers[RenderSnapshot::ISLANDS],
    snapshot.buffers[RenderSnapshot::CONTACT_ISLANDS]};

  std::array<daxa::TaskImage, 4> images = {task_swapchain_image, task_accumulation_buffer, task_stbn_texture, task_rt_target};

  std::array<daxa::TaskTlas, 1> tlases = {accel_struct_mngr->task_tlas};

  RT_TG = task_manager->create_task_graph(RT_TG_name, buffers, images, {}, tlases, true);

  // upscale blit: scaled trace target -> full-resolution swapchain (linear filter). Only
  // recorded when render scale is active; the GUI tasks draw AFTER at full resolution.
  daxa::InlineTaskInfo task_upscale({
      .attachments = {
          daxa::inl_attachment(daxa::TaskImageAccess::TRANSFER_READ, task_rt_target),
          daxa::inl_attachment(daxa::TaskImageAccess::TRANSFER_WRITE, task_swapchain_image),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        auto const src = ti.get(task_rt_target).id;
        auto const dst = ti.get(task_swapchain_image).id;
        auto const s = ti.device.image_info(src).value().size;
        auto const d = ti.device.image_info(dst).value().size;
        ti.recorder.blit_image_to_image({
            .src_image = src,
            .dst_image = dst,
            .src_offsets = {{{0, 0, 0}, {(daxa_i32)s.x, (daxa_i32)s.y, 1}}},
            .dst_offsets = {{{0, 0, 0}, {(daxa_i32)d.x, (daxa_i32)d.y, 1}}},
            .filter = daxa::Filter::LINEAR,
        });
      },
      .name = "upscale rt target to swapchain",
  });

  RT_TG.add_task(task_update_RT_config);
  // the frame-0 CLEAR must run BEFORE the trace: it used to run after, wiping the first
  // accumulated sample so frame 1 averaged against zeros (a half-brightness start that
  // only washed out ~1/N)
  RT_TG.add_task(task_cpy_to_accum_buffer);
  RT_TG.add_task(task_RT);
  if (render_scale < 1.0f)
  {
    RT_TG.add_task(task_upscale);
  }
  auto axes = gui_manager->gui_axes_task_info;
  axes.views[2] = daxa::attachment_view(GUIAxesTaskHead::AT.vertex_buffer, snapshot.buffers[RenderSnapshot::AXES]);
  auto lines = gui_manager->gui_line_task_info;
  lines.views[2] = daxa::attachment_view(GUILineTaskHead::AT.vertex_buffer, snapshot.buffers[RenderSnapshot::LINES]);
  auto points = gui_manager->gui_task_info;
  points.views[2] = daxa::attachment_view(GUITaskHead::AT.vertex_buffer, snapshot.buffers[RenderSnapshot::POINTS]);
  RT_TG.add_task(axes);
  RT_TG.add_task(lines);
  RT_TG.add_task(points);
  RT_TG.add_task(daxa::InlineTaskInfo{
    .attachments = {daxa::inl_attachment(daxa::TaskAccessConsts::COLOR_ATTACHMENT, task_swapchain_image)},
    .task = [this](daxa::TaskInterface const &ti) {
      auto const image = ti.get(task_swapchain_image).id;
      auto const extent = ti.device.image_info(image).value().size;
      char const *solver = get_solver() == SimSolverType::AVBD ? "AVBD" :
                           get_solver() == SimSolverType::TGS_SOFT ? "TGS" : "PGS";
      performance.draw(ti.recorder, image, extent.x, extent.y, rigid_body_manager->step_timer.metric,
                       frame_timer.metric, solver, !status_manager->is_simulating());
      frame_timer.end(ti.recorder);
    },
    .name = "Performance overlay (final render pass)",
  });

  // Daxa waits the last producer queue of shared external resources (including
  // task_tlas). TaskSubmitInfo's additional semaphore fields are unused in 3.6.
  RT_TG.submit();
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

  if (render_timing) { render_queries={}; render_query_pending=false; }
  performance.destroy();
  frame_timer.destroy();
  snapshot.destroy(gpu->device);
  gpu->device.destroy_buffer(ray_tracing_config_buffer);

  gpu->device.destroy_image(accumulation_buffer);
  gpu->device.destroy_image(rt_target_image);

  initialized = false;
}

bool RendererManager::execute()
{
  if (!initialized)
  {
    return false;
  }
  frame_timer.prepare(gpu->device);
  RT_TG.execute();
  frame_timer.submitted(gpu->device, daxa::QUEUE_MAIN);
  ++performance.rates.frames;
  return true;
}

bool RendererManager::update_resources(daxa::ImageId swapchain_image, CameraManager &cam_mngr)
{
  if (!initialized)
  {
    return false;
  }

  // set_image/set_buffer WIPE daxa's cross-execution sync state (pre_graph_queue_bits and
  // the general-layout flag) - the graph then re-initializes the image with src_access={}
  // i.e. WITHOUT waiting for the previous frame's access, and records the correct state
  // after every execution precisely so the next one can chain on it. Re-setting the SAME
  // resource every frame defeats that: the trace could overwrite rt_target while the
  // previous frame's upscale blit still reads it (tile-torn silhouettes during camera
  // motion, only with BB_RENDER_SCALE<1 - at scale 1.0 the trace writes the rotating
  // swapchain image). Only set when the backing resource actually changed (resize/reload).
  // The swapchain image rotates every frame and its sync is acquire/present semaphores.
  task_swapchain_image.set_image(swapchain_image);
  if (task_camera_buffer.id() != cam_mngr.camera_buffer) { task_camera_buffer.set_buffer(cam_mngr.camera_buffer); }
  if (task_ray_tracing_config.id() != ray_tracing_config_buffer) { task_ray_tracing_config.set_buffer(ray_tracing_config_buffer); }
  if (task_accumulation_buffer.id() != accumulation_buffer) { task_accumulation_buffer.set_image(accumulation_buffer); }
  if (task_rt_target.id() != rt_target_image) { task_rt_target.set_image(rt_target_image); }
  if (task_stbn_texture.id() != image_manager->get_spatiotemporal_blue_noise_image()) { task_stbn_texture.set_image(image_manager->get_spatiotemporal_blue_noise_image()); }

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
  if (det_steps > 0 && !std::getenv("BB_SCENE") && !std::getenv("BB_SCENE_FILE"))
    { status_manager->request_scene(3); } // default only; honor explicit test scene
  auto const run_start = std::chrono::steady_clock::now();
  // Preserve the existing one-step publication after a scene load/switch. It
  // initializes the render-facing simulation state even when starting paused.
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
  // BB_DUMP_AT_SECONDS=N: headless F12 - dump the LIVE poses once, N seconds into the run
  // (no window focus needed; SendKeys-based captures race against the user's foreground)
  double dump_at_s = -1.0;
  if (const char *e = std::getenv("BB_DUMP_AT_SECONDS")) dump_at_s = std::atof(e);
  bool dump_at_done = false;

  // BB_ASYNC_SIM=0 retains the synchronous scheduler for A/B measurements.
  // Replays default to the original schedule; explicitly enable async to test
  // identical physics steps while rendering a variable number of snapshots.
  bool const async_sim = std::getenv("BB_ASYNC_SIM")
      ? beat_box_diagnostics::parse_boolean("BB_ASYNC_SIM", std::getenv("BB_ASYNC_SIM"))
      : det_steps == 0;
  double render_hz = 60.0;
  if (auto const *e = std::getenv("BB_RENDER_HZ")) render_hz = std::clamp(std::atof(e), 1.0, 240.0);
  bool const render_fairness = !std::getenv("BB_RENDER_FAIRNESS") ||
      beat_box_diagnostics::parse_boolean("BB_RENDER_FAIRNESS", std::getenv("BB_RENDER_FAIRNESS"));
  daxa_u64 deferred_render_polls = 0;
  auto next_render = std::chrono::steady_clock::now();
  bool sim_pending = false;
  bool render_snapshot_current = false;
  daxa_u64 built_snapshot_generation = 0;
  bool built_bvh = false;
  daxa_u32 steps_since_render = 0;
  daxa_u64 pending_submit = 0;
  auto pending_start = std::chrono::steady_clock::now();
  bool force_sim_step = true;
  daxa_u64 render_frames_total = 0; // ALL render frames (stasis ones too) - the PERF frame
                                    // metric divides wall time by THIS, not by stepped
                                    // frames, or full-sleep stasis inflates frame=/deflates
                                    // the printed fps (user-reported "ya no topa 60fps"
                                    // that the present rate disproved)
  bool const frame_timing = std::getenv("BB_FRAME_TIMING") != nullptr || std::getenv("BB_PACING_TRACE") != nullptr;
  bool const fracture_timing = std::getenv("BB_RESPAWN_TIMING") != nullptr;
  daxa_u32 timed_fracture_serial = 0u;
  unsigned fracture_frame_tail = 0u;
  daxa_u32 previous_frame_steps = 0u;
  double previous_sim_phase_ms = 0.0;
  auto prepare_snapshot = [&]() {
    if (!snapshot.select(gpu->device)) return false;
    auto const &selected = snapshot.slots[snapshot.selected];
    rigid_body_manager->render_config = selected.config;
    snapshot_debug_valid = selected.debug_valid;
    bool const bvh = is_bvh_enabled();
    if (snapshot.selected_generation != built_snapshot_generation || bvh != built_bvh) {
      // Accumulate only repeated renders of the same published pose.
      status_manager->reset_accumulation_count();
      accel_struct_mngr->update_TLAS();
      built_snapshot_generation = snapshot.selected_generation;
      built_bvh = bvh;
    }
    return true;
  };
  // Attribute an interval to the work BEFORE its ending frame boundary,
  // including non-render simulation pumps. FRAME-PHASES is per-pump detail.
  struct FrameWindow {
    enum Phase { WAIT, FRONT, EVENTS, RESIZE, ACQUIRE, EARLY_RENDER, SIM,
                 EDITS, SYNC, CAPTURE_RENDER, SUBMIT, GC, COUNT };
    std::array<double, COUNT> ms = {};
    unsigned resizes = 0, controls = 0, publications = 0, steps = 0;
    void report(double wall, u64 frame, bool pending) const {
      if (wall < 1000.0 / 30.0) return;
      static char const *names[] = {"wait", "front", "events", "resize", "acquire", "early_render",
                                   "sim", "edits", "sync", "capture_render", "submit", "gc"};
      double accounted = 0;
      std::cout << "[SLOW-FRAME] frame=" << frame << " wall_ms=" << wall;
      for (unsigned i = 0; i < COUNT; ++i) { accounted += ms[i]; std::cout << ' ' << names[i] << "_ms=" << ms[i]; }
      std::cout << " unattributed_ms=" << wall - accounted << " resizes=" << resizes
                << " controls=" << controls << " publications=" << publications
                << " steps=" << steps << " pending=" << pending << std::endl;
    }
  } frame_window;
  auto elapsed_ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b-a).count(); };
  auto performance_solver = get_solver();
  bool performance_has_frame = false;
  auto reset_performance = [&] {
    ++frame_window.controls;
    frame_timer.reset(); rigid_body_manager->step_timer.reset();
    performance.frame.reset_average(); performance.rates = {};
    performance.rates.since = std::chrono::duration<double>(std::chrono::steady_clock::now() - run_start).count();
    // Keep frame history across scene changes: their stalls also count.
  };
  auto fracture_frame_clock = std::chrono::steady_clock::now();
  while (!window.should_close())
  {
    // Wake for physics independently of presentation. While a GPU step is in
    // flight, poll its completion at most every millisecond and service events.
    auto now = std::chrono::steady_clock::now();
    if (det_steps == 0 || async_sim) {
      auto wake = next_render;
      if (async_sim) {
        if (sim_pending) wake = std::min(wake, now + std::chrono::milliseconds(1));
        else if (status_manager->is_simulating()) {
          auto sim_due = sim_clock_prev + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(std::max(0.0, SIM_DT_S - sim_accum_s)));
          wake = std::min(wake, sim_due);
        }
      }
      double wait_s = std::chrono::duration<double>(wake - now).count();
      if (wait_s > 0) glfwWaitEventsTimeout(wait_s);
    }
    auto const frame_clock = std::chrono::steady_clock::now();
    bool render_due = !snapshot.allocated || (det_steps > 0 && !async_sim) || frame_clock >= next_render;
    double const previous_frame_ms = std::chrono::duration<double, std::milli>(frame_clock - fracture_frame_clock).count();
    if (render_due && snapshot.allocated && render_fairness && async_sim && det_steps == 0 &&
        sim_pending && render_hz > 1.0 / SIM_DT_S) {
      double const step_age = std::chrono::duration<double>(frame_clock - pending_start).count();
      double const retry = render_retry_delay(step_age, previous_frame_ms / 1000.0, SIM_DT_S);
      if (retry > 0) {
        render_due = false;
        ++deferred_render_polls;
        next_render = frame_clock + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(retry));
      }
    }
    frame_window.ms[FrameWindow::WAIT] += elapsed_ms(now, frame_clock);
    if (render_due) {
      if (performance_has_frame) frame_window.report(previous_frame_ms, render_frames_total, sim_pending);
      bool const resize_interval = frame_window.resizes != 0;
      frame_window = {};
      frame_timer.collect(gpu->device);
      rigid_body_manager->step_timer.collect(gpu->device);
      if (performance_has_frame) {
        if (resize_interval) performance.rates.exclude_render_interval(previous_frame_ms / 1000.0);
        else performance.frame.add(previous_frame_ms);
      }
      performance_has_frame = true;
      double const seconds = std::chrono::duration<double>(frame_clock - run_start).count();
      if (performance.rates.refresh(seconds)) {
        performance.frame.refresh(); frame_timer.metric.refresh(); rigid_body_manager->step_timer.metric.refresh();
        if (frame_timing || std::getenv("BB_HUD_TRACE")) {
          auto const &sim = rigid_body_manager->step_timer.metric;
          auto const &render = frame_timer.metric;
          std::cout << "[HUD-METRICS] seconds=" << seconds
                    << " sim_ms=" << sim.current_ms << " worst_step_ms=" << sim.worst_ms
                    << " render_ms=" << render.current_ms << " worst_render_ms=" << render.worst_ms
                    << " sim_hz=" << performance.rates.sim_hz << " render_fps=" << performance.rates.render_fps
                    << " deferred_render_polls=" << deferred_render_polls
                    << " frame_ms=" << performance.frame.current_ms << " worst_frame_ms=" << performance.frame.worst_ms
                    << std::endl;
        }
      }
      fracture_frame_clock = frame_clock;
      next_render = frame_clock + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / render_hz));
    }
    if (render_due && fracture_timing && fracture_frame_tail > 0u)
    {
      // Includes acquire/submission waits in the previous loop, without adding a GPU wait.
      std::cout << "[FRACTURE-FRAME] frame=" << render_frames_total
                << " serial=" << timed_fracture_serial << " wall_ms=" << previous_frame_ms
                << " steps=" << previous_frame_steps << " sim_phase_ms=" << previous_sim_phase_ms << std::endl;
      --fracture_frame_tail;
    }
    if (render_due) ++render_frames_total;
    if (render_due) { previous_frame_steps = 0u; previous_sim_phase_ms = 0.0; }
    if (run_limit_s > 0.0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - run_start).count() > run_limit_s)
    {
      std::cout << "[PERF] BB_RUN_SECONDS=" << run_limit_s << " elapsed -> exiting." << std::endl;
      break;
    }
    if (render_timing && render_query_pending) {
      auto q = render_queries.get_query_results(0, 2);
      if (q[1] && q[3]) {
        std::cout << "[RENDER-GPU] trace_ms=" << double(q[2]-q[0])*gpu->device.properties().limits.timestamp_period/1e6 << std::endl;
        render_query_pending = false;
      }
    }
    auto const timing_start = std::chrono::steady_clock::now();
    // Update the GUI
    gui_manager->update();
    auto const timing_gui_end = std::chrono::steady_clock::now();

    // A GUI toggle (TAB) rebuilds the ImGui overlay (+ the contact-point debug pass), hitching this
    // frame and the next; suppress the sim's REAL catch-up for those frames so the (AVBD-jittering)
    // pile doesn't advance several steps at once and visibly jerk. Same intent as the reset clamp.
    {
      bool const cur_gui = status_manager->is_gui_enabled();
      if (cur_gui != prev_gui_enabled) { gui_hitch_cooldown = 2; }
      prev_gui_enabled = cur_gui;
    }

    if (!sim_pending) {
    if (get_solver() != performance_solver) {
      performance_solver = get_solver(); reset_performance();
    }
    // reset request (key R): restart the sim from the initial scene at this frame boundary (prior
    // GPU work is already synchronized here), and clear the catch-up accumulator so it doesn't burst.
    if (status_manager->consume_reset()) {
      reset_performance();
      snapshot_debug_valid = false;
      render_snapshot_current = false;
      snapshot.invalidate();
      scene_manager->reset();
      sim_accum_s = 0.0;
      force_sim_step = true;
    }

    // scene switch request (F1-F8): rebuild from the chosen scene at this same frame boundary,
    // paused, and clear the catch-up accumulator so it doesn't burst on the first resumed step.
    if (int const requested_scene = status_manager->consume_scene(); requested_scene >= 0) {
      reset_performance();
      snapshot_debug_valid = false;
      render_snapshot_current = false;
      snapshot.invalidate();
      scene_manager->switch_scene(requested_scene);
      sim_accum_s = 0.0;
      force_sim_step = true;
    }

    // live scene dump request (F12): capture the CURRENT GPU poses to a scene file at this
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

    }

    // FRAME-PACING ANCHOR: poll events + acquire the swapchain image BEFORE the sim steps.
    // The acquire is the call that blocks at vsync cadence (FIFO); with it up front every
    // loop iteration is evenly ~16.7 ms and the fixed-timestep accumulator receives time
    // SMOOTHLY (1 step per presented frame). Measured at the old position - before the
    // blocking point, with the CPU running ahead of the GPU queue - wall time arrived in
    // ~50 ms bursts: the sim stepped 2-3 times every third frame and physics motion
    // visibly stuttered at ~20 Hz while the display presented a smooth 60 (user: "la
    // simulación se ralentiza en algunos momentos"; pace=[rf71 st71] with 31 print
    // frames was the tell). Latent flaw exposed by the render getting 2x faster.
    auto const timing_events_start = std::chrono::steady_clock::now();
    if (!window.update()) {
      frame_window.ms[FrameWindow::FRONT] += elapsed_ms(timing_start, timing_events_start);
      frame_window.ms[FrameWindow::EVENTS] += elapsed_ms(timing_events_start, std::chrono::steady_clock::now());
      continue;
    }
    auto const timing_events_end = std::chrono::steady_clock::now();
    if (window.swapchain_out_of_date)
    {
      ++frame_window.resizes;
      gpu->swapchain_resize();
      window.swapchain_out_of_date = false;
      gpu->device.destroy_image(accumulation_buffer);
      gpu->device.destroy_image(rt_target_image);
      auto const rs_ext = daxa::Extent3D(std::max(1u, (daxa_u32)(gpu->swapchain_get_extent().x * render_scale)),
                                         std::max(1u, (daxa_u32)(gpu->swapchain_get_extent().y * render_scale)), 1);
      accumulation_buffer = gpu->device.create_image({
          .format = daxa::Format::R32G32B32A32_SFLOAT, // linear HDR storage (see create())
          .size = rs_ext, // matches the TRACE resolution (accumulation is pre-upscale)
          .usage = daxa::ImageUsageFlagBits::SHADER_STORAGE | daxa::ImageUsageFlagBits::TRANSFER_DST | daxa::ImageUsageFlagBits::TRANSFER_SRC,
          .name = "accumulation_buffer",
      });
      rt_target_image = gpu->device.create_image({
          .format = gpu->swapchain.get_format(),
          .size = render_scale < 1.0f ? rs_ext : daxa::Extent3D(1, 1, 1),
          .usage = daxa::ImageUsageFlagBits::SHADER_STORAGE | daxa::ImageUsageFlagBits::TRANSFER_SRC | daxa::ImageUsageFlagBits::TRANSFER_DST,
          .name = "rt_target",
      });
      status_manager->reset_accumulation_count();
    }
    auto const timing_acquire_start = std::chrono::steady_clock::now();
    auto swapchain_image = render_due ? gpu->swapchain_acquire_next_image() : daxa::ImageId{};
    auto const timing_acquire_end = std::chrono::steady_clock::now();
    frame_window.ms[FrameWindow::FRONT] += elapsed_ms(timing_start, timing_events_start);
    frame_window.ms[FrameWindow::EVENTS] += elapsed_ms(timing_events_start, timing_events_end);
    frame_window.ms[FrameWindow::RESIZE] += elapsed_ms(timing_events_end, timing_acquire_start);
    frame_window.ms[FrameWindow::ACQUIRE] += elapsed_ms(timing_acquire_start, timing_acquire_end);
    if (render_due && swapchain_image.is_empty())
      continue;

    // Present the already-published snapshot first. A completed physics step or
    // a fracture transaction must not stand between fresh camera input and RT.
    bool const rendered_early = async_sim && render_due && render_snapshot_current && prepare_snapshot();
    auto early_render_start = std::chrono::steady_clock::now();
    if (rendered_early) {
      camera_manager->update(gpu->swapchain_get_extent());
      update_resources(swapchain_image, *camera_manager);
      execute();
      snapshot.mark_read(gpu->device);
    }
    double const early_render_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - early_render_start).count();

    // Simulate rigid bodies: fixed 60 Hz with REAL catch-up. The sim double buffers rotate
    // per STEP (begin_sim_step inside simulate()), so several steps inside one render frame
    // are legal; only the last step of the burst gets published (TLAS build below). Below
    // 60/MAX_CATCHUP_STEPS fps the sim slows down instead of spiraling.
    constexpr daxa_u32 MAX_CATCHUP_STEPS = 4u;
    auto const sim_phase_start = std::chrono::steady_clock::now();
    daxa_u32 sim_steps_this_frame = 0u;
    bool completed_simulation_snapshot = false;
    bool launch_async_step = false;
    auto completed_main_submit = gpu->device.latest_queue_submit_index(daxa::QUEUE_MAIN);
    auto completed_compute_submit = gpu->device.latest_queue_submit_index(daxa::QUEUE_COMPUTE_0);
    double sim_order_ms=0, sim_submit_ms=0, sim_wait_ms=0;
    auto run_sim_step = [&]() {
      auto const t0=std::chrono::steady_clock::now();
      completed_main_submit = gpu->device.latest_queue_submit_index(daxa::QUEUE_MAIN);
      gpu->order_simulation_after_rendering();
      auto const t1=std::chrono::steady_clock::now();
      rigid_body_manager->simulate();
      auto const t2=std::chrono::steady_clock::now();
      gpu->wait_for_simulation();
      completed_compute_submit = gpu->device.latest_queue_submit_index(daxa::QUEUE_COMPUTE_0);
      auto const t3=std::chrono::steady_clock::now();
      auto ms=[](auto a,auto b) { return std::chrono::duration<double,std::milli>(b-a).count(); };
      sim_order_ms+=ms(t0,t1); sim_submit_ms+=ms(t1,t2); sim_wait_ms+=ms(t2,t3);
      // The simulation completion boundary also covers the sampled render.
      if (render_timing && render_query_pending) {
        auto const q=render_queries.get_query_results(0,2);
        if (q[1]!=0u && q[3]!=0u) {
          std::cout << "[RENDER-GPU] trace_ms=" << double(q[2]-q[0])*gpu->device.properties().limits.timestamp_period/1e6 << std::endl;
          render_query_pending=false;
        }
      }
      // CPU submit/completion span, including any queued render dependency.
      ++performance.rates.steps;
      return ms(t1,t3);
    };
    auto const poll_start = std::chrono::steady_clock::now();
    if (sim_pending && gpu->device.oldest_pending_submit_index() > pending_submit) {
      sim_pending = false;
      ++performance.rates.steps;
      completed_simulation_snapshot = true;
      sim_steps_this_frame = 1u;
      completed_compute_submit = pending_submit;
      // No CPU render wait here. Capture uses a free snapshot slot; topology
      // edits retain their retirement boundary before reusing geometry storage.
      _sim_ms_accum += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - pending_start).count();
      ++_sim_ms_n;
    }
    double const completion_poll_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-poll_start).count();
    double pick_input_ms = 0;
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
      else if (!async_sim || completed_simulation_snapshot)
      {
        if (!async_sim) run_sim_step();
        completed_simulation_snapshot = true;
        sim_steps_this_frame = 1u;
        ++det_count;
        rigid_body_manager->read_back_sim_config(true);
        auto const &dsc = rigid_body_manager->get_sim_config_reference();
        std::cout << "[CHAIN] max=" << dsc.dbg_chain_max << " errors=" << dsc.dbg_chain_errors << " overflow=" << dsc.graph_color_overflow << std::endl;
        std::cout << "DET step=" << det_count << std::hex << " ph=" << dsc.dbg_poshash
                  << " rh=" << dsc.dbg_rothash << " cp2=" << dsc.dbg_cp2_poshash
                  << " cph=" << dsc.dbg_cp_poshash << " vhf=" << dsc.dbg_vh_fin
                  << " vhi=" << dsc.dbg_vh_imp << " chash=" << dsc.dbg_color_hash
                  << " lh=" << dsc.dbg_color_pad << " sh=" << dsc.dbg_state_hash
                  << " wh=" << dsc.dbg_state_pad
                  << std::dec << " viol=" << (dsc.avbd_violations + dsc.graph_color_violations + dsc.dbg_chain_errors) << std::endl;
        det_acc = det_acc * 0x9e3779b9u + dsc.dbg_poshash; // cumulative path hash (catches transient divergence)
        if (det_count >= (daxa_u32)det_steps)
        {
          if (det_inproc && det_pass == 1)
          {
            det_hashA = det_acc;
            std::cout << "INPROC passA acc=" << std::hex << det_acc << std::dec << " -> reset + replay" << std::endl;
            snapshot_debug_valid = false;
            render_snapshot_current = false;
            snapshot.invalidate();
            scene_manager->reset();   // reload scene_3 to the IDENTICAL initial state
            completed_simulation_snapshot = false; // reset replaced the published state
            sim_steps_this_frame = 0u; // never publish a phantom step into the reset scene
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
            if (const char *dump = std::getenv("BB_DET_DUMP"))
              scene_manager->dump_scene_live(dump);
            break;
          }
        }
      }
      if (async_sim && !sim_pending && det_inited) launch_async_step = true;
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
        auto const pick_start = std::chrono::steady_clock::now();
        auto &cam = camera_manager->camera;
        bool const left_held = glfwGetMouseButton(window.glfw_window_ptr, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        static bool prev_left = false;
        f64 const cx = window.cursor_x, cy = window.cursor_y;
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
        pick_input_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-pick_start).count();
      }
      // ONE forced step after a scene load/switch to publish the async AS through the shared-resource
      // dependency path (cures the at-rest "dented/rounded cubes"). At rest is_simulating() is false, so
      // only this runs; sim_steps_this_frame=1 makes the AS-update block below rebuild the publication.
      if (force_sim_step)
      {
        run_sim_step();
        completed_simulation_snapshot = true;
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
        auto const &ssc = rigid_body_manager->completed_config;
        if (ssc.active_rigid_body_count == 0u ||
            ssc.sleeping_count < ssc.active_rigid_body_count) { return false; }
        // a GUI toggle (TAB) changes DEBUG_INFO, and the contact-point debug buffers only
        // regenerate inside a sim step - force a couple of (identity) steps so the overlay
        // shows the CURRENT contacts instead of whatever the buffers last held
        if (gui_hitch_cooldown > 0) { return false; }
        return glfwGetMouseButton(window.glfw_window_ptr, GLFW_MOUSE_BUTTON_LEFT) != GLFW_PRESS;
      }();
      if (status_manager->is_simulating() && !stasis)
      {
        // during a GUI-toggle hitch, cap the accumulator to ONE step (no burst -> no jerk); the few
        // ms of lost real-time sync over the toggle is imperceptible and resyncs once cooldown ends.
        double const accum_cap = gui_hitch_cooldown > 0 ? SIM_DT_S : (MAX_CATCHUP_STEPS + 1.0) * SIM_DT_S;
        if (gui_hitch_cooldown > 0) { --gui_hitch_cooldown; }
        sim_accum_s = std::min(sim_accum_s + elapsed_s, accum_cap);
        if (async_sim) {
          launch_async_step = !sim_pending && sim_accum_s >= SIM_DT_S;
        }
        while (!async_sim && sim_accum_s >= SIM_DT_S && sim_steps_this_frame < MAX_CATCHUP_STEPS)
        {
          sim_accum_s -= SIM_DT_S;
          _sim_ms_accum += run_sim_step();
          completed_simulation_snapshot = true;
          _sim_ms_n++;                                                 // [PERF]
          ++sim_steps_this_frame;
          // A costly contact step must not trigger four equally costly catch-up
          // steps before presenting again. Keep fixed dt and the existing backlog
          // cap, but yield to rendering once this frame's simulation budget is spent.
          if (std::chrono::duration<double>(std::chrono::steady_clock::now() - sim_phase_start).count() >= SIM_DT_S)
            break;
        }
      }
      else
      {
        sim_accum_s = 0.0; // don't burst-step on resume
      }
    }
    auto const timing_sim_end = std::chrono::steady_clock::now();
    auto timing_edits_end = timing_sim_end;
    auto timing_sync_end = timing_sim_end;
    previous_frame_steps += sim_steps_this_frame;
    previous_sim_phase_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - sim_phase_start).count();
    bool const sim_stepped = sim_steps_this_frame > 0u;
    // (window.update() + swapchain acquire moved to the frame-pacing anchor above the sim)

    // Update the acceleration structures (only when the sim actually stepped — skipped frames
    // render the unchanged state and avoid the full-pipeline synchronize + readback)
    if(!sim_pending && (sim_stepped || status_manager->is_updating() || !render_snapshot_current)) {
      rigid_body_manager->read_back_sim_config(completed_simulation_snapshot);
      // FRACTURE: consume any impact-pass events (dedicated GPU->host bridge buffer).
      // GPU partitioning and publication return compact AS build metadata;
      // no-op (one generation comparison) when nothing fractured.
      if (sim_stepped)
      {
        if (auto const *feb = rigid_body_manager->get_fracture_events())
        {
          if (fracture_timing && feb->serial != timed_fracture_serial)
          {
            timed_fracture_serial = feb->serial;
            fracture_frame_tail = 4u; // publication frame and three following frames
          }
          scene_manager->process_fracture_events(*feb);
        }
        // Retire and spawn in one GPU edit, reusing freed slots before one AS publication.
        scene_manager->process_scene_edits(rigid_body_manager->get_sim_config_reference().dbg_min_y);
      }
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
        if (!_pk_hdr) { _pk << "frame,pk_pen_mm,pk_lambda,pk_k,pk_vn,b1,b2,cc,stick,global_pen_mm,maxv_mm,omega_mrad,manifolds,sleeping,interior_hits\n"; _pk_hdr = true; }
        _pk << (daxa_u64)pk.frame_count
            << "," << pk.dbg_pk_pen << "," << pk.dbg_pk_lambda << "," << pk.dbg_pk_k << "," << pk.dbg_pk_vn
            << "," << (pk.dbg_pk_body >> 16) << "," << (pk.dbg_pk_body & 0xFFFFu)
            << "," << (pk.dbg_pk_stick >> 1) << "," << (pk.dbg_pk_stick & 1u)
            << "," << pk.dbg_pen << "," << pk.dbg_maxv << "," << pk.dbg_pk_omega
            << "," << pk.g_c_info.collision_count << "," << pk.sleeping_count << "," << pk.dbg_vox_interior << "\n";
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
        static daxa_u64 _lrf = 0;
        // sample every 31 frames (odd) so the readback alternates between the two double-buffered
        // SimConfigs — each holds an independent dbg_ex latch; an even cadence would only ever show one.
        if ((++_cf % 31) == 0) {
          auto _t1 = std::chrono::high_resolution_clock::now();
          // divide by ALL render frames since the last print (stasis frames included), not by
          // the 31 stepped ones - stasis interleaving otherwise inflates frame=
          daxa_u64 _rf = render_frames_total - _lrf; _lrf = render_frames_total;
          double _ms = std::chrono::duration<double, std::milli>(_t1 - _t0).count() / (double)(_rf ? _rf : 1); _t0 = _t1;
          double _sim_ms = _sim_ms_n ? (_sim_ms_accum / (double)_sim_ms_n) : 0.0;   // [PERF]
          daxa_u64 _steps_n = _sim_ms_n;                                             // [PERF] steps since last print
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
                    << " frac=" << (rigid_body_manager->get_fracture_events() ? rigid_body_manager->get_fracture_events()->serial : 0u)
                    << " pen=" << sc.dbg_pen
                    << " miny=" << (sc.dbg_min_y == 0xFFFFFFFFu ? 0.0 : (double)sc.dbg_min_y / 1000.0 - 100.0)
                    << " deep100=" << sc.dbg_deep100 << " deep200=" << sc.dbg_deep200
                    << " idsum=" << (daxa_i64)sc.dbg_id_sum - (daxa_i64)((daxa_u64)sc.rigid_body_count * (sc.rigid_body_count - 1) / 2)
                    << " EX[s=" << sc.dbg_ex_stage << " b=" << sc.dbg_ex_body << " f=" << sc.dbg_ex_frame
                    << " v=" << sc.dbg_ex_vel << " y=" << sc.dbg_ex_y << " vy=" << sc.dbg_ex_vy << "]"
                    << " | pace=[rf" << _rf << " st" << _steps_n << " acc" << (daxa_u64)(sim_accum_s * 1000.0) << "ms]"
                    << " frame=" << _ms << " ms (" << (1000.0 / _ms) << " fps)"
                    << "  sim=" << _sim_ms << " ms" << std::endl; } }
      timing_edits_end = std::chrono::steady_clock::now();
      // Ordinary pose capture only needs COMPUTE completion. Scene edits may
      // publish or retire geometry on either queue; keep their existing full
      // completion boundary before invalidating old geometry snapshots.
      bool const publication_pending = !completed_simulation_snapshot ||
          status_manager->is_updating() ||
          gpu->device.latest_queue_submit_index(daxa::QUEUE_MAIN) != completed_main_submit ||
          gpu->device.latest_queue_submit_index(daxa::QUEUE_COMPUTE_0) != completed_compute_submit;
      if (publication_pending) { ++frame_window.publications; gpu->synchronize(); }
      timing_sync_end = std::chrono::steady_clock::now();
      accel_struct_mngr->collect_publication_timings();
      rigid_body_manager->release_completed_scene_uploads();
      if(status_manager->is_updating()) {
        if(!status_manager->reset_update_sim_buffer()) {
          accel_struct_mngr->update_AS_buffers();
        }
      }
      // Capture the complete publication, including debug data and CPU draw counts.
      // The next solve cannot overwrite these render-only buffers.
      auto const &sc = rigid_body_manager->get_sim_config_reference();
      SimConfig render_config = sc;
      bool const debug_valid = completed_simulation_snapshot && !publication_pending;
      if (!debug_valid) render_config.flags = SimFlag(daxa_u32(sc.flags) & ~daxa_u32(SimFlag::DEBUG_INFO));
      if (publication_pending) {
        // The lifetime boundary above has drained readers before topology reuse.
        snapshot.invalidate();
        render_snapshot_current = false;
      }
      daxa_u64 const bodies = std::min(sc.rigid_body_count, daxa_u32(MAX_RIGID_BODY_COUNT));
      bool const debug = debug_valid && (sc.flags & SimFlag::DEBUG_INFO) != 0u;
      bool const captured = snapshot.publish(gpu->device, render_config, debug_valid, {
          bodies * sizeof(RigidBody), bodies * sizeof(RigidBodyEntry),
          (bodies ? bodies * 2 - 1 : 0) * sizeof(LBVHNode),
          daxa_u64(sc.island_count) * sizeof(Island),
          daxa_u64(sc.contact_island_count) * sizeof(ContactIsland),
          debug ? daxa_u64(std::min(sc.g_c_info.collision_point_count, daxa_u32(BB_MAX_DEBUG_CONTACT_POINT_COUNT))) * sizeof(GUIVertex) : 0,
          debug ? daxa_u64(std::min(sc.g_c_info.collision_point_count * 2u, daxa_u32(BB_MAX_DEBUG_CONTACT_LINE_VERTEX_COUNT))) * sizeof(GUIVertexLine) : 0,
          debug ? bodies * 6 * sizeof(GUIVertexLine) : 0,
          sizeof(SimConfig),
      }, publication_pending ? gpu->device.latest_queue_submit_index(daxa::QUEUE_MAIN) : 0);
      if (!captured && !render_snapshot_current) throw std::runtime_error("No snapshot slot after topology retirement");
      render_snapshot_current |= captured;
    }

    // (swapchain resize + acquire moved to the frame-pacing anchor above the sim; the image
    // was acquired there and is rendered here)
    steps_since_render += sim_steps_this_frame;
    if (render_due && !rendered_early) {
      if (!prepare_snapshot()) throw std::runtime_error("No render snapshot available");
      camera_manager->update(gpu->swapchain_get_extent());
      update_resources(swapchain_image, *camera_manager);
      execute();
      snapshot.mark_read(gpu->device);
    }
    auto const timing_render_end = std::chrono::steady_clock::now();
    // The capture copy precedes this step on COMPUTE. No MAIN reader owns the
    // solver's buffers; occupied snapshot slots are skipped instead of stalling.
    if (launch_async_step && !rigid_body_manager->is_dirty() && !status_manager->has_pending_scene_request()) {
      if (det_steps == 0) sim_accum_s -= SIM_DT_S;
      pending_start = std::chrono::steady_clock::now();
      rigid_body_manager->simulate();
      pending_submit = gpu->device.latest_queue_submit_index(daxa::QUEUE_COMPUTE_0);
      sim_pending = true;
    }
    auto const timing_async_end = std::chrono::steady_clock::now();
    gpu->garbage_collector();
    frame_window.steps += sim_steps_this_frame;
    frame_window.ms[FrameWindow::EARLY_RENDER] += elapsed_ms(timing_acquire_end, sim_phase_start);
    frame_window.ms[FrameWindow::SIM] += elapsed_ms(sim_phase_start, timing_sim_end);
    frame_window.ms[FrameWindow::EDITS] += elapsed_ms(timing_sim_end, timing_edits_end);
    frame_window.ms[FrameWindow::SYNC] += elapsed_ms(timing_edits_end, timing_sync_end);
    frame_window.ms[FrameWindow::CAPTURE_RENDER] += elapsed_ms(timing_sync_end, timing_render_end);
    frame_window.ms[FrameWindow::SUBMIT] += elapsed_ms(timing_render_end, timing_async_end);
    frame_window.ms[FrameWindow::GC] += elapsed_ms(timing_async_end, std::chrono::steady_clock::now());
    if (frame_timing && render_due) {
      auto const end = std::chrono::steady_clock::now();
      auto ms = [](auto a,auto b) { return std::chrono::duration<double,std::milli>(b-a).count(); };
      std::cout << "[FRAME-PHASES] frame=" << render_frames_total
                << " steps=" << steps_since_render
                << " pending=" << sim_pending
                << " wall_ms=" << previous_frame_ms
                << " front_ms=" << ms(timing_start,sim_phase_start)
                << " gui_ms=" << ms(timing_start,timing_gui_end)
                << " scene_updates_ms=" << ms(timing_gui_end,timing_events_start)
                << " events_ms=" << ms(timing_events_start,timing_events_end)
                << " front_cpu_ms=" << ms(timing_start,timing_acquire_start)
                << " acquire_ms=" << ms(timing_acquire_start,timing_acquire_end)
                << " completion_poll_ms=" << completion_poll_ms
                << " pick_input_ms=" << pick_input_ms
                << " sim_ms=" << ms(sim_phase_start,timing_sim_end)
                << " sim_order_ms=" << sim_order_ms
                << " sim_submit_ms=" << sim_submit_ms
                << " sim_wait_ms=" << sim_wait_ms
                << " edits_ms=" << ms(timing_sim_end,timing_edits_end)
                << " sync_ms=" << ms(timing_edits_end,timing_sync_end)
                << " early_render_ms=" << early_render_ms
                << " render_ms=" << ms(timing_sync_end,timing_render_end)
                << " async_submit_ms=" << ms(timing_render_end,timing_async_end)
                << " gc_ms=" << ms(timing_async_end,end)
                << " worst_step_ms=" << rigid_body_manager->step_timer.metric.worst_ms
                << " worst_render_ms=" << frame_timer.metric.worst_ms
                << " worst_frame_ms=" << performance.frame.worst_ms
                << " total_ms=" << ms(timing_start,end) << std::endl;
    }
    if (render_due) {
      status_manager->next_frame();
      steps_since_render = 0;

    }
  }
  gpu->synchronize();
  gpu->garbage_collector();
  if (metrics_csv.is_open()) metrics_csv.flush();
  return metrics_exit_code;
}

RendererManager::~RendererManager() { gpu->synchronize(); destroy(); }

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
