#pragma once

#include "defines.hpp"
#include "camera.h"

BB_NAMESPACE_BEGIN

struct SceneManager
{
  // Device
  daxa::Device &device;
  // Camera buffer
  daxa::BufferId camera_buffer;

  // TODO: Fill in scene data from file?
  std::vector<RigidBody> rigid_bodies;
  std::vector<Aabb> aabb;
  
  Camera camera;

  explicit SceneManager(char const *name, daxa::Device &device) : device(device)
  {
    camera_buffer = device.create_buffer({
        .size = sizeof(CameraView),
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "camera_buffer",
    });

    reset_camera(camera);
  }
  ~SceneManager() {
    if(!camera_buffer.is_empty())
      device.destroy_buffer(camera_buffer);
  }

  bool load_scene() {
    // TODO: temporary scene
    rigid_bodies = {{.primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0, 0.0, 1.0), .rotation = daxa_f32vec4(1.0, 0.0, 0.0, 0.0)}, {.primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0, 2.0, 1.0), .rotation = daxa_f32vec4(1.0, 0.0, 0.0, 0.0)}};
    aabb = {{.min = daxa_f32vec3(-0.5, -0.5, -0.5), .max = daxa_f32vec3(0.5, 0.5, 0.5)}, {.min = daxa_f32vec3(-0.5, -0.5, -0.5), .max = daxa_f32vec3(0.5, 0.5, 0.5)}};

    return true;
  }


  void update(daxa::Extent2D const& extent) {
    // TODO: Update scene

    // Update camera
    camera_set_aspect(camera, extent.x, extent.y);

    // Update camera buffer
    CameraView camera_view = {
        .inv_view = get_inverse_view_matrix(camera),
        .inv_proj = get_projection_matrix(camera)
    };
    
    // NOTE: Vulkan has inverted y axis in NDC
    camera_view.inv_proj.y.y *= -1;

    device.buffer_host_address_as<CameraView>(camera_buffer).value()[0] = camera_view;
  }
};

BB_NAMESPACE_END