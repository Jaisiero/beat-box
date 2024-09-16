#pragma once

#include "defines.hpp"
#include "camera.h"

BB_NAMESPACE_BEGIN

struct CameraManager{

  // Device
  daxa::Device &device;
  // Camera buffer
  daxa::BufferId camera_buffer;
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

    camera_buffer = device.create_buffer({
        .size = sizeof(CameraView),
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
        .name = camera_name,
    });

    reset_camera(camera);

    initialized = true;

    return initialized;
  }

  void destroy() {
    if(!initialized) return;

    device.destroy_buffer(camera_buffer);

    initialized = false;
  }
  
  
  void update(daxa::Extent2D const& extent) {
    // TODO: Update scene

    // Update camera
    camera_set_aspect(camera, extent.x, extent.y);

    // Update camera buffer
    CameraView camera_view = {
        .inv_view = get_inverse_view_matrix(camera),
        .inv_proj = get_inverse_projection_matrix(camera, true),
    };

    device.buffer_host_address_as<CameraView>(camera_buffer).value()[0] = camera_view;
  }

};

BB_NAMESPACE_END