#pragma once
#include "defines.hpp"
#include <daxa/utils/task_graph.hpp>
#include "acceleration_structure_manager.hpp"
#include "camera_manager.hpp"
#include "rigid_body_manager.hpp"
#include "ray_tracing_pipeline.hpp"
#include "scene_manager.hpp"
#include "status_manager.hpp"
#include "gui_manager.hpp"

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
  // Status manager reference
  std::shared_ptr<StatusManager> status_manager;
  // GUI manager reference
  std::shared_ptr<GUIManager> gui_manager;

  // Task graph information for ray tracing
  TaskGraph RT_TG;
  daxa::TaskImage task_swapchain_image{{.swapchain_image = true, .name = "swapchain_image"}};
  daxa::TaskImage task_accumulation_buffer{{.swapchain_image = false, .name = "accumulation_buffer"}};
  daxa::TaskBuffer task_camera_buffer{{.initial_buffers = {}, .name = "camera_buffer"}};
  daxa::TaskBuffer task_ray_tracing_config{{.initial_buffers = {}, .name = "ray_tracing_config"}};
  daxa::TaskBuffer task_ray_tracing_config_host{{.initial_buffers = {}, .name = "ray_tracing_config_host"}};

  explicit RendererManager(std::shared_ptr<GPUcontext> gpu, std::shared_ptr<TaskManager> task_manager, WindowManager& window, std::shared_ptr<CameraManager> camera_manager, std::shared_ptr<AccelerationStructureManager> accel_struct_mngr, std::shared_ptr<RigidBodyManager> rigid_body_manager, std::shared_ptr<SceneManager> scene_manager, std::shared_ptr<StatusManager> status_manager, std::shared_ptr<GUIManager> gui_manager);
  ~RendererManager();

  bool create(char const *RT_TG_name, std::shared_ptr<RayTracingPipeline> pipeline, daxa::RayTracingShaderBindingTable SBT);
  void destroy();

  bool update_resources(daxa::ImageId swapchain_image, CameraManager &cam_mngr);

  void render();
  daxa_u32 get_previous_frame_index();
  daxa_u32 get_frame_index();
  daxa_u32 get_next_frame_index();
  bool is_gui_enabled() {
    return status_manager->is_gui_enabled();
  }
  daxa_u64 get_frame_count() {
    return status_manager->get_frame_count();
  }
  daxa_u32 get_rigid_body_count() {
    return scene_manager->get_rigid_body_count();
  }
  SimSolverType get_solver() {
    return status_manager->get_solver();
  }

private:
  bool execute();

  daxa::BufferId ray_tracing_config_buffer[DOUBLE_BUFFERING];
  daxa::BufferId ray_tracing_config_host_buffer[DOUBLE_BUFFERING];

  daxa::ImageId accumulation_buffer;
};

BB_NAMESPACE_END