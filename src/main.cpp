
#include "defines.hpp"
#include "window_manager.hpp"
#include "gpu_context.hpp"
#include "task_manager.hpp"
#include "renderer_manager.hpp"
#include "ray_tracing_pipeline.hpp"
#include "acceleration_structure_manager.hpp"
#include "scene_manager.hpp"
#include "input_manager.hpp"
#include "camera_manager.hpp"
#include "rigid_body_manager.hpp"
#include "status_manager.hpp"


#include "shared.inl"

using namespace beatbox;

int main(int argc, char const *argv[])
{
  // Input manager
  InputManager input_manager;
  // Window
  WindowManager window("Beat Box", input_manager, 860, 640);
  // GPU context
  auto gpu = std::make_shared<GPUcontext>("RT device", "Swapchain", window);
  // Task manager (pipeline manager)
  auto task_manager = std::make_shared<TaskManager>("Pipeline Manager", gpu);
  // Camera manager
  auto camera_manager = std::make_shared<CameraManager>(gpu->device);
  auto accel_struct_mngr = std::make_shared<AccelerationStructureManager>(gpu->device, task_manager);
  // Rigid body simulator pipeline
  auto rigid_body_manager = std::make_shared<RigidBodyManager>(gpu->device, task_manager);
  // Status manager
  auto status_manager = std::make_shared<StatusManager>(gpu, accel_struct_mngr, rigid_body_manager);
  // Scene manager
  auto scene_manager = std::make_shared<SceneManager>("Scene Manager", gpu->device, accel_struct_mngr, rigid_body_manager, status_manager);

  // Primary tracing pipeline
  auto RT_pipeline = std::make_shared<RayTracingPipeline>(task_manager->create_ray_tracing(MainRayTracingPipeline{}.info), gpu->device);
  // Renderer
  auto renderer = std::make_shared<RendererManager>(gpu, task_manager, window, camera_manager, accel_struct_mngr, rigid_body_manager, scene_manager, status_manager);

  // Create camera manager
  camera_manager->create("Camera Manager");
  // Create input manager which depends on camera manager and window
  input_manager.create(camera_manager, status_manager);
  // Create task graph
  renderer->create("Ray Tracing Task Graph", RT_pipeline, RT_pipeline->build_SBT());
  // Create rigid body simulator
  rigid_body_manager->create("Rigid Body Manager", renderer, 10);
  // Create acceleration structure manager
  accel_struct_mngr->create(renderer);
  // Create status manager
  status_manager->create();
  // Create scene manager
  scene_manager->create();

  // Load scene
  if(!scene_manager->load_scene()) {
    return -1;
  }

  // Main loop
  renderer->render();

  // Cleanup
  status_manager->destroy();
  rigid_body_manager->destroy();
  accel_struct_mngr->destroy();
  input_manager.destroy();
  camera_manager->destroy();

  return 0;
}