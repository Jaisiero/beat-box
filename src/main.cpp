
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

  camera_manager->create("Camera Manager");
  input_manager.create(camera_manager);

  auto ray_tracing_pipeline = pipeline_manager.create_ray_tracing(MainRayTracingPipeline{}.info);
  RayTracingSBT rt_pipeline_sbt(ray_tracing_pipeline, gpu.device);

  AccelerationStructureManager accel_struct_mngr(gpu.device);
  accel_struct_mngr.create();

  RayTracingTaskGraph loop_TG("RT TaskGraph", gpu, {
      .ray_tracing_pipeline = ray_tracing_pipeline,
      .shader_binding_table = rt_pipeline_sbt.build_sbt()
  });

  {
    scene_manager.load_scene();
    // TODO: Handle error
    if(!accel_struct_mngr.build_accel_structs(scene_manager.rigid_bodies, scene_manager.aabb)) return -1;
    accel_struct_mngr.build_accel_struct_execute();
    gpu.synchronize();
  }

  while (!window.should_close())
  { 
      if(!window.update()) continue;

      if (window.swapchain_out_of_date)
      {
          gpu.swapchain_resize();
          window.swapchain_out_of_date = false;
          loop_TG.SBT = rt_pipeline_sbt.rebuild_sbt();
      }

      auto swapchain_image = gpu.swapchain_acquire_next_image();
      if (!swapchain_image.is_empty())
      {
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