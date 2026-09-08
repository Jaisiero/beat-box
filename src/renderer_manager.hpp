#pragma once
#include "defines.hpp"
#include <daxa/utils/task_graph.hpp>
#include "acceleration_structure_manager.hpp"
#include "camera_manager.hpp"
#include "rigid_body_manager.hpp"
#include "ray_tracing_pipeline.hpp"
#include "status_manager.hpp"
#include "gui_manager.hpp"
#include "image_manager.hpp"
#include "render_snapshot.hpp"
#include "performance_overlay.hpp"

BB_NAMESPACE_BEGIN

// forward-declared on purpose (review v3): scene_manager.hpp is a 1200+-line header-only manager
// (all scene definitions inline); including it here pulled it into every TU that includes this
// header (5 of 8), so ANY scene edit rebuilt nearly the whole project. Only the shared_ptr member
// and three getters (defined in renderer_manager.cpp) need the type.
struct SceneManager;

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
  // Status manager reference
  std::shared_ptr<StatusManager> status_manager;
  // GUI manager reference
  std::shared_ptr<GUIManager> gui_manager;
  // Image manager reference
  std::shared_ptr<ImageManager> image_manager;

  // Task graph information for ray tracing
  TaskGraph RT_TG;
  RenderSnapshot snapshot;
  PerformanceOverlay performance;
  GpuPerformanceTimer frame_timer;
  bool snapshot_debug_valid = false;
  daxa::TimelineQueryPool render_queries = {};
  bool render_timing = false;
  bool render_query_pending = false;
  daxa::TaskImage task_swapchain_image{{.is_swapchain_image = true, .name = "swapchain_image"}};
  daxa::TaskImage task_accumulation_buffer{{.is_swapchain_image = false, .name = "accumulation_buffer"}};
  // render scale (BB_RENDER_SCALE env, 0.25..1.0): trace into a SCALED offscreen target,
  // then blit-upscale (linear) to the swapchain. The GUI still draws at full resolution
  // AFTER the upscale. At 1.0 the target is a 1x1 dummy and the old direct wiring is used.
  daxa::TaskImage task_rt_target{{.is_swapchain_image = false, .name = "rt_target"}};
  daxa::TaskBuffer task_camera_buffer{{.buffer = {}, .name = "camera_buffer"}};
  daxa::TaskBuffer task_ray_tracing_config{{.buffer = {}, .name = "ray_tracing_config"}};
  daxa::TaskImage task_stbn_texture{{.is_swapchain_image = false, .name = "stbn_texture_task"}};

  explicit RendererManager(std::shared_ptr<GPUcontext> gpu, std::shared_ptr<TaskManager> task_manager, WindowManager& window, std::shared_ptr<CameraManager> camera_manager, std::shared_ptr<AccelerationStructureManager> accel_struct_mngr, std::shared_ptr<RigidBodyManager> rigid_body_manager, std::shared_ptr<SceneManager> scene_manager, std::shared_ptr<StatusManager> status_manager, std::shared_ptr<GUIManager> gui_manager, std::shared_ptr<ImageManager> image_manager);
  ~RendererManager();

  bool create(char const *RT_TG_name, std::shared_ptr<RayTracingPipeline> pipeline, daxa::RayTracingShaderBindingTable SBT);
  void destroy();

  bool update_resources(daxa::ImageId swapchain_image, CameraManager &cam_mngr);

  int render(); // returns process exit code (0 = ok; non-zero when a BB_ASSERT_* metric threshold fails)
  daxa_u32 get_previous_frame_index();
  daxa_u32 get_frame_index();
  daxa_u32 get_next_frame_index();
  // sim clock (per-step parity, decoupled from the render frame index; see StatusManager)
  daxa_u32 get_sim_frame_index();
  daxa_u32 get_sim_next_frame_index();
  daxa_u32 get_sim_previous_frame_index();
  void begin_sim_step();
  bool is_gui_enabled() {
    return status_manager->is_gui_enabled();
  }
  bool is_bvh_enabled() {
    return snapshot_debug_valid && status_manager->is_bvh_enabled();
  }
  daxa_u64 get_frame_count() {
    return status_manager->get_frame_count();
  }
  // bodies in renderer_manager.cpp (SceneManager is forward-declared here — see the note above)
  daxa_u32 get_rigid_body_count();
  daxa_u32 get_active_rigid_body_count();
  std::vector<ActiveRigidBody> get_active_rigid_bodies();
  SimSolverType get_solver() {
    return status_manager->get_solver();
  }

private:
  bool execute();

  daxa::BufferId ray_tracing_config_buffer;

  daxa::ImageId accumulation_buffer;
  daxa::ImageId rt_target_image = {};
  f32 render_scale = 1.0f; // BB_RENDER_SCALE, parsed once in create()
};

BB_NAMESPACE_END