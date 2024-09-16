#pragma once

#include "defines.hpp"
#include "camera_manager.hpp"

BB_NAMESPACE_BEGIN

struct SceneManager
{
  // Device
  daxa::Device &device;

  // TODO: Fill in scene data from file?
  std::vector<RigidBody> rigid_bodies;
  std::vector<Aabb> aabb;

  explicit SceneManager(char const *name, daxa::Device &device) : device(device)
  {

  }
  ~SceneManager() {
  }

  bool load_scene() {
    // TODO: temporary scene
    rigid_bodies = {{.primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0, 0.0, 1.0), .rotation = daxa_f32vec4(1.0, 0.0, 0.0, 0.0)}, {.primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0, 2.0, 1.0), .rotation = daxa_f32vec4(1.0, 0.0, 0.0, 0.0)}};
    aabb = {{.min = daxa_f32vec3(-0.5, -0.5, -0.5), .max = daxa_f32vec3(0.5, 0.5, 0.5)}, {.min = daxa_f32vec3(-0.5, -0.5, -0.5), .max = daxa_f32vec3(0.5, 0.5, 0.5)}};

    return true;
  }
};

BB_NAMESPACE_END