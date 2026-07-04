#pragma once

#include "defines.hpp"
#include "camera.h"

BB_NAMESPACE_BEGIN

struct CameraManager{

  // Device
  daxa::Device &device;
  // Camera buffer, DOUBLE-BUFFERED by frame parity: it is host-written every frame while
  // the PREVIOUS frame's trace may still be in flight reading it. A single buffer races
  // that read - during a camera drag some RT workgroups see the old matrix and some the
  // new one, tearing silhouettes into tiles. Same convention as every other per-frame
  // host-written buffer in the engine.
  daxa::BufferId camera_buffer[DOUBLE_BUFFERING];
  // Camera
  Camera camera;
  // boolean to check if the camera manager is initialized
  bool initialized = false;

  explicit CameraManager(daxa::Device &device) : device(device)
  {
  }

  ~CameraManager() {
    destroy();
  }
  
  bool create(char const *camera_name) {
    if(initialized) return false;

    for (daxa_u32 f = 0u; f < DOUBLE_BUFFERING; ++f)
    {
      camera_buffer[f] = device.create_buffer({
          .size = sizeof(CameraView),
          .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
          .name = std::string(camera_name) + std::to_string(f),
      });
    }

    reset_camera(camera);

    initialized = true;

    return initialized;
  }

  void destroy() {
    if(!initialized) return;

    for (daxa_u32 f = 0u; f < DOUBLE_BUFFERING; ++f) { device.destroy_buffer(camera_buffer[f]); }

    initialized = false;
  }
  
  
  // frame_index: this frame's parity - the write must go to the buffer the OTHER frame
  // is not reading (the caller binds the same parity via task_camera_buffer)
  void update(daxa::Extent2D const& extent, daxa_u32 frame_index) {
    // TODO: Update scene

    // Update camera
    camera_set_aspect(camera, extent.x, extent.y);

    // Update camera buffer
    CameraView camera_view = {
        .inv_view = get_inverse_view_matrix(camera),
        .inv_proj = get_inverse_projection_matrix(camera, true),
        .view = get_view_matrix(camera),
        .proj = get_projection_matrix(camera, true),
    };

    device.buffer_host_address_as<CameraView>(camera_buffer[frame_index % DOUBLE_BUFFERING]).value()[0] = camera_view;
  }

};

BB_NAMESPACE_END