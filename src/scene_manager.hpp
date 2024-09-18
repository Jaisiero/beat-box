#pragma once

#include "defines.hpp"
#include "camera_manager.hpp"
#include "rigid_body_manager.hpp"
#include "acceleration_structure_manager.hpp"

BB_NAMESPACE_BEGIN

struct SceneManager
{
  // Device
  daxa::Device &device;
  // Acceleration structure manager reference
  AccelerationStructureManager &accel_struct_mngr;
  // Rigid body manager reference
  RigidBodyManager &rigid_body_manager;

  // TODO: Fill in scene data from file?
  std::vector<RigidBody> rigid_bodies;
  std::vector<Aabb> aabb;

  explicit SceneManager(char const *name, daxa::Device &device, AccelerationStructureManager &accel_struct_mngr, RigidBodyManager &rigid_body_manager) : device(device), accel_struct_mngr(accel_struct_mngr), rigid_body_manager(rigid_body_manager)
  {
  }
  ~SceneManager()
  {
  }

  bool load_scene()
  {
    // TODO: temporary scene
    rigid_bodies = {{.primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0, 0.0, 1.0), .rotation = daxa_f32vec4(1.0, 0.0, 0.0, 0.0), .min = daxa_f32vec3(-0.5, -0.5, -0.5), .max = daxa_f32vec3(0.5, 0.5, 0.5)}, {.primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0, 2.0, 1.0), .rotation = daxa_f32vec4(1.0, 0.0, 0.0, 0.0), .min = daxa_f32vec3(-0.5, -0.5, -0.5), .max = daxa_f32vec3(0.5, 0.5, 0.5)}};
    aabb = {{.min = daxa_f32vec3(-0.5, -0.5, -0.5), .max = daxa_f32vec3(0.5, 0.5, 0.5)}, {.min = daxa_f32vec3(-0.5, -0.5, -0.5), .max = daxa_f32vec3(0.5, 0.5, 0.5)}};

    accel_struct_mngr.update_TLAS_resources(rigid_body_manager.dispatch_buffer, rigid_body_manager.sim_config);

    // TODO: Handle error
    if (!accel_struct_mngr.build_accel_structs(rigid_bodies, aabb))
      return false;
    accel_struct_mngr.build_AS();

    // Update simulation info
    rigid_body_manager.update_dispatch_buffer(get_rigid_body_count());
    rigid_body_manager.update_resources(accel_struct_mngr.rigid_body_buffer, accel_struct_mngr.primitive_buffer);

    return true;
  }

  daxa_u32 get_rigid_body_count()
  {
    return rigid_bodies.size();
  }
};

BB_NAMESPACE_END