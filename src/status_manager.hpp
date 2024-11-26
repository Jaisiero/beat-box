#pragma once

#include "defines.hpp"
#include "gpu_context.hpp"
#include "acceleration_structure_manager.hpp"
#include "rigid_body_manager.hpp"

BB_NAMESPACE_BEGIN

struct StatusManager
{

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
        .size = sizeof(DispatchBuffer),
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
        .name = "RB_dispatch_buffer",
    });

    *gpu->device.buffer_host_address_as<DispatchBuffer>(dispatch_buffer).value() = DispatchBuffer(daxa_u32vec3(1u, 1u, 1u), daxa_u32vec3(1u, 1u, 1u));

    // Link resources
    accel_struct_mngr->update_TLAS_resources(dispatch_buffer);

    rigid_body_manager->update_resources(dispatch_buffer, accel_struct_mngr->primitive_buffer, accel_struct_mngr->get_points_buffer());

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

    gpu->device.buffer_host_address_as<DispatchBuffer>(dispatch_buffer).value()->dispatch = daxa_u32vec3((rigid_body_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X, 1, 1);

    return initialized;
  }

  bool next_frame()
  {
    if (!initialized)
    {
      return false;
    }

    frame_index = (frame_index + 1) % DOUBLE_BUFFERING;

    return true;
  }

  daxa_u32 get_frame_index()
  {
    return frame_index;
  }

  bool is_simulating()
  {
    return simulating;
  }

  void switch_simulating()
  {
    simulating = !simulating;
    if(!simulating)
    {
      update_sim_buffer = true;
      double_buffering_counter = 2;
    }
  }

  bool is_updating()
  {
    return update_sim_buffer;
  }

  bool reset_update_sim_buffer()
  {
    if(--double_buffering_counter == 0)
      update_sim_buffer = false;
    return !update_sim_buffer;
  }

  bool is_gui_enabled()
  {
    return gui_enabled;
  }

  void switch_gui_enabled()
  {
    gui_enabled = !gui_enabled;
    if(gui_enabled)
    {
      rigid_body_manager->set_sim_flags(SimFlag::DEBUG_INFO);
    }
    else
    {
      rigid_body_manager->clear_sim_flags(SimFlag::DEBUG_INFO);
    }
  }

  bool is_advection()
  {
    return advection;
  }

  void switch_advection()
  {
    advection = !advection;
    if(advection)
    {
      rigid_body_manager->set_sim_flags(SimFlag::ADVECTION);
    }
    else
    {
      rigid_body_manager->clear_sim_flags(SimFlag::ADVECTION);
    }
  }

private:
  // Gpu context reference
  std::shared_ptr<GPUcontext> gpu;
  // Acceleration structure manager reference
  std::shared_ptr<AccelerationStructureManager> accel_struct_mngr;
  // Rigid body manager reference
  std::shared_ptr<RigidBodyManager> rigid_body_manager;
  // flag for initialization
  bool initialized = false;
  // flag for simulation
  bool simulating = true;
  // update simulation buffer
  bool update_sim_buffer = false;
  // flag for gui
  bool gui_enabled = true;
  // flag for advection
  bool advection = true;
  // double buffering counter
  daxa_u32 double_buffering_counter = 0;
  // frame index
  daxa_u32 frame_index = 0;
  // Dispatch buffer
  daxa::BufferId dispatch_buffer;
};

BB_NAMESPACE_END