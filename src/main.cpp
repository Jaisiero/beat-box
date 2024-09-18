
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
  RendererManager renderer(gpu, task_manager);
  RigidBodyManager rigid_body_manager(gpu.device, task_manager);

  RayTracingPipeline RT_pipeline(task_manager.create_ray_tracing(MainRayTracingPipeline{}.info), gpu.device);

  // TODO: refactor all this
  camera_manager->create("Camera Manager");
  input_manager.create(camera_manager);
  renderer.create("Ray Tracing Task Graph", RT_pipeline.pipeline, RT_pipeline.build_SBT());
  rigid_body_manager.create("Rigid Body Manager");
  accel_struct_mngr.create();
  accel_struct_mngr.update_TLAS_resources(rigid_body_manager.dispatch_buffer, rigid_body_manager.sim_config);

  scene_manager.load_scene();
  // TODO: Handle error
  if (!accel_struct_mngr.build_accel_structs(scene_manager.rigid_bodies, scene_manager.aabb))
    return -1;
  accel_struct_mngr.build_AS();
  gpu.synchronize();

  // Update simulation info
  rigid_body_manager.update_dispatch_buffer(scene_manager.rigid_bodies.size());
  rigid_body_manager.update_resources(accel_struct_mngr.rigid_body_buffer, accel_struct_mngr.primitive_buffer);

  while (!window.should_close())
  {
    rigid_body_manager.simulate();
    accel_struct_mngr.update();
    accel_struct_mngr.update_TLAS();

    if (!window.update())
      continue;

    if (window.swapchain_out_of_date)
    {
      gpu.swapchain_resize();
      window.swapchain_out_of_date = false;
    }

    auto handle_reload_result = [&](daxa::PipelineReloadResult reload_error, std::shared_ptr<daxa::RayTracingPipeline> RT_pipeline, RayTracingPipeline& RT_SBT, RendererManager& TG) -> void
    {
      if (auto error = daxa::get_if<daxa::PipelineReloadError>(&reload_error)) {
            std::cout << "Failed to reload " << error->message << std::endl;
      } else if (daxa::get_if<daxa::PipelineReloadSuccess>(&reload_error)) {
        TG.destroy();
        TG.create("Ray Tracing Task Graph", RT_pipeline, RT_SBT.rebuild_SBT());
        std::cout << "Successfully reloaded!" << std::endl;
      }
    };

    auto swapchain_image = gpu.swapchain_acquire_next_image();
    if (!swapchain_image.is_empty())
    {
      handle_reload_result(task_manager.reload(), RT_pipeline.pipeline, RT_pipeline, renderer);
      
      camera_manager->update(gpu.swapchain_get_extent());
      renderer.update_resources(swapchain_image, *camera_manager, accel_struct_mngr.tlas, accel_struct_mngr.rigid_body_buffer, accel_struct_mngr.primitive_buffer);
      renderer.execute();
      gpu.garbage_collector();
    }
  }

  rigid_body_manager.destroy();
  accel_struct_mngr.destroy();
  input_manager.destroy();
  camera_manager->destroy();

  return 0;
}