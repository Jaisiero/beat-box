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
  CameraView camera_view = {};
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
    camera_view = {
        .inv_view = get_inverse_view_matrix(camera),
        .inv_proj = get_inverse_projection_matrix(camera, true),
        .view = get_view_matrix(camera),
        .proj = get_projection_matrix(camera, true),
    };

    // The render task snapshots this CPU value into its timeline-managed upload
    // allocator. Never overwrite a mapped buffer while an earlier frame reads it.
  }

};

BB_NAMESPACE_END