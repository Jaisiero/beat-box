#pragma once

#include "defines.hpp"
#include "gpu_context.hpp"
#include "acceleration_structure_manager.hpp"
#include "rigid_body_manager.hpp"

BB_NAMESPACE_BEGIN

struct StatusManager
{
  // Gpu context reference
  std::shared_ptr<GPUcontext> gpu;
  // Acceleration structure manager reference
  std::shared_ptr<AccelerationStructureManager> accel_struct_mngr;
  // Rigid body manager reference
  std::shared_ptr<RigidBodyManager> rigid_body_manager;
  // flag for initialization
  bool initialized = false;
  
  // Dispatch buffer
  daxa::BufferId dispatch_buffer;

  explicit StatusManager(std::shared_ptr<GPUcontext> gpu, std::shared_ptr<AccelerationStructureManager> accel_struct_mngr, std::shared_ptr<RigidBodyManager> rigid_body_manager) : gpu(gpu), accel_struct_mngr(accel_struct_mngr), rigid_body_manager(rigid_body_manager)
  {
  }

  ~StatusManager()
  {
  }

  bool create()
  {
    if (initialized)
    {
      return false;
    }
    
    dispatch_buffer = gpu->device.create_buffer({
        .size = sizeof(daxa_u32vec3),
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
        .name = "RB_dispatch_buffer",
    });

    *gpu->device.buffer_host_address_as<daxa_u32vec3>(dispatch_buffer).value() = daxa_u32vec3(1, 1, 1);

    // Link resources
    accel_struct_mngr->update_TLAS_resources(dispatch_buffer, rigid_body_manager->sim_config);

    rigid_body_manager->update_resources(dispatch_buffer, accel_struct_mngr->rigid_body_buffer, accel_struct_mngr->primitive_buffer);

    return initialized = true;
  }

  void destroy()
  {
    if (!initialized)
    {
      return;
    }

    // Destroy resources
    gpu->device.destroy_buffer(dispatch_buffer);

    initialized = false;
  }

  bool update_dispatch_buffer(daxa_u32 rigid_body_count)
  {
    if(!initialized)
    {
      return false;
    }

    *gpu->device.buffer_host_address_as<daxa_u32vec3>(dispatch_buffer).value() = daxa_u32vec3((rigid_body_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X, 1, 1);

    return initialized;
  }
};

BB_NAMESPACE_END