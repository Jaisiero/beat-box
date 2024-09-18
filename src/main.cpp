
#include "defines.hpp"
#include "window.hpp"
#include "gpu_context.hpp"
#include "task_manager.hpp"
#include "ray_tracing_task_graph.hpp"
#include "ray_tracing_SBT.hpp"
#include "acceleration_structure_manager.hpp"
#include "scene_manager.hpp"
#include "input_manager.hpp"
#include "camera_manager.hpp"
#include "rigid_body_manager.hpp"

#include "shared.inl"

using namespace beatbox;

int main(int argc, char const *argv[])
{
  InputManager input_manager;
  AppWindow window("Beat Box", input_manager, 860, 640);
  GPUcontext gpu("RT device", "Swapchain", window);
  TaskManager task_manager("Pipeline Manager", gpu);
  SceneManager scene_manager("Scene Manager", gpu.device);
  std::shared_ptr<CameraManager> camera_manager = std::make_shared<CameraManager>(gpu.device);
  AccelerationStructureManager accel_struct_mngr(gpu.device, task_manager);
  RigidBodyManager rigid_body_manager(gpu.device, task_manager);

  // Primary tracing pipeline
  RayTracingPipeline RT_pipeline(task_manager.create_ray_tracing(MainRayTracingPipeline{}.info), gpu.device);
  // Renderer
  RendererManager renderer(gpu, task_manager, window, camera_manager, accel_struct_mngr, rigid_body_manager, RT_pipeline);

  // Create camera manager
  camera_manager->create("Camera Manager");
  // Create input manager which depends on camera manager and window
  input_manager.create(camera_manager);
  // Create task graph
  renderer.create("Ray Tracing Task Graph", RT_pipeline.pipeline, RT_pipeline.build_SBT());
  // Create rigid body simulator
  rigid_body_manager.create("Rigid Body Manager");
  // Create acceleration structure manager
  accel_struct_mngr.create();


  // TODO: Refactor this
  accel_struct_mngr.update_TLAS_resources(rigid_body_manager.dispatch_buffer, rigid_body_manager.sim_config);

  scene_manager.load_scene();
  // TODO: Handle error
  if (!accel_struct_mngr.build_accel_structs(scene_manager.rigid_bodies, scene_manager.aabb))
    return -1;
  accel_struct_mngr.build_AS();
  gpu.synchronize();

  // Update simulation info
  rigid_body_manager.update_dispatch_buffer(scene_manager.get_rigid_body_count());
  rigid_body_manager.update_resources(accel_struct_mngr.rigid_body_buffer, accel_struct_mngr.primitive_buffer);
  // TODO: refactor this

  // Main loop
  renderer.render();

  // Cleanup
  rigid_body_manager.destroy();
  accel_struct_mngr.destroy();
  input_manager.destroy();
  camera_manager->destroy();

  return 0;
}