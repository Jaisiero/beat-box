
#include "defines.hpp"
#include "window.hpp"
#include "gpu_context.hpp"
#include "pipeline_manager.hpp"
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
  PipelineManager pipeline_manager("Pipeline Manager", gpu.device);
  SceneManager scene_manager("Scene Manager", gpu.device);
  std::shared_ptr<CameraManager> camera_manager = std::make_shared<CameraManager>(gpu.device);
  AccelerationStructureManager accel_struct_mngr(gpu.device);
  RayTracingTaskGraph loop_TG(gpu);
  RigidBodyManager rigid_body_manager(gpu.device);

  auto ray_tracing_pipeline = pipeline_manager.create_ray_tracing(MainRayTracingPipeline{}.info);
  RayTracingSBT rt_pipeline_SBT(ray_tracing_pipeline, gpu.device);

  auto RB_pipeline = pipeline_manager.create_compute(RigidBodySim{}.info);
  auto Update_AS = pipeline_manager.create_compute(UpdateAccelerationStructures{}.info);

  // TODO: refactor all this
  camera_manager->create("Camera Manager");
  input_manager.create(camera_manager);
  loop_TG.create("Ray Tracing Task Graph", RayTracingParams{ray_tracing_pipeline, rt_pipeline_SBT.build_SBT()});
  rigid_body_manager.create("Rigid Body Manager", RB_pipeline);
  accel_struct_mngr.create(Update_AS);
  accel_struct_mngr.update_TLAS_resources(rigid_body_manager.dispatch_buffer, rigid_body_manager.sim_config);

  {
    scene_manager.load_scene();
    // TODO: Handle error
    if (!accel_struct_mngr.build_accel_structs(scene_manager.rigid_bodies, scene_manager.aabb))
      return -1;
    accel_struct_mngr.build_AS();
    gpu.synchronize();
  }

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

    auto handle_reload_result = [&](daxa::PipelineReloadResult reload_error, std::shared_ptr<daxa::RayTracingPipeline> RT_pipeline, RayTracingSBT& RT_SBT, RayTracingTaskGraph& TG) -> void
    {
      if (auto error = daxa::get_if<daxa::PipelineReloadError>(&reload_error)) {
            std::cout << "Failed to reload " << error->message << std::endl;
      } else if (daxa::get_if<daxa::PipelineReloadSuccess>(&reload_error)) {
        TG.destroy();
        TG.create("Ray Tracing Task Graph", RayTracingParams{RT_pipeline, RT_SBT.rebuild_SBT()});
        std::cout << "Successfully reloaded!" << std::endl;
      }
    };

    auto swapchain_image = gpu.swapchain_acquire_next_image();
    if (!swapchain_image.is_empty())
    {
      handle_reload_result(pipeline_manager.reload(), ray_tracing_pipeline, rt_pipeline_SBT, loop_TG);
      
      camera_manager->update(gpu.swapchain_get_extent());
      loop_TG.update_resources(swapchain_image, *camera_manager, accel_struct_mngr.tlas, accel_struct_mngr.rigid_body_buffer, accel_struct_mngr.primitive_buffer);
      loop_TG.execute();
      gpu.garbage_collector();
    }
  }

  accel_struct_mngr.destroy();
  input_manager.destroy();
  camera_manager->destroy();

  return 0;
}