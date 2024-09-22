#pragma once

#include "defines.hpp"
#include "camera_manager.hpp"
#include "rigid_body_manager.hpp"
#include "acceleration_structure_manager.hpp"
#include "status_manager.hpp"

BB_NAMESPACE_BEGIN

struct SceneManager
{
  // Device
  daxa::Device &device;
  // Acceleration structure manager reference
  std::shared_ptr<AccelerationStructureManager> accel_struct_mngr;
  // Rigid body manager reference
  std::shared_ptr<RigidBodyManager> rigid_body_manager;
  // Status manager reference
  std::shared_ptr<StatusManager> status_manager;
  // Initialization flag
  bool initialized = false;

  // TODO: Fill in scene data from file?
  std::vector<RigidBody> rigid_bodies;
  std::vector<Aabb> aabb;

  explicit SceneManager(char const *name, daxa::Device &device, std::shared_ptr<AccelerationStructureManager> accel_struct_mngr, std::shared_ptr<RigidBodyManager> rigid_body_manager, std::shared_ptr<StatusManager> status_manager) : device(device), accel_struct_mngr(accel_struct_mngr), rigid_body_manager(rigid_body_manager), status_manager(status_manager)
  {
  }
  ~SceneManager()
  {
  }

  bool create()
  {
    if (initialized)
    {
      return false;
    }

    return initialized = true;
  }

  bool load_scene()
  {
    if (!initialized)
    {
      return false;
    }

    // TODO: temporary scene
    rigid_bodies = {{.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0, -5.0, 0.0), .rotation = daxa_f32vec4(0.0, 0.0, 0.0, 1.0), .min = daxa_f32vec3(-5.0, -5.0, -5.0), .max = daxa_f32vec3(5.0, 5.0, 5.0), .mass = 1.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5, .friction = 0.5}, {.flags = (RigidBodyFlag::DYNAMIC | RigidBodyFlag::KINEMATIC), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-0.5, 2.0, -0.5), .rotation = daxa_f32vec4(0.4572f, 0.0000f, -0.4572f, -0.7629f), .min = daxa_f32vec3(-0.5, -0.5, -0.5), .max = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.7, .friction = 0.3}, {.flags = (RigidBodyFlag::DYNAMIC | RigidBodyFlag::KINEMATIC), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-0.5, 5.0, -0.5), .rotation = daxa_f32vec4(0.0, 0.0, 0.0, 1.0), .min = daxa_f32vec3(-1.0, -1.0, -1.0), .max = daxa_f32vec3(1.0, 1.0, 1.0), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.7, .friction = 0.6}};


    for(auto &rigid_body : rigid_bodies)
    {
      rigid_body.inv_mass = 1.0f / rigid_body.mass;
      rigid_body.inv_inertia = cuboid_get_inverse_intertia(rigid_body.inv_mass, rigid_body.min, rigid_body.max);
    }

    aabb = {{.min = daxa_f32vec3(-5.0, -5.0, -5.0), .max = daxa_f32vec3(5.0, 5.0, 5.0)}, {.min = daxa_f32vec3(-0.5, -0.5, -0.5), .max = daxa_f32vec3(0.5, 0.5, 0.5)}, {.min = daxa_f32vec3(-1.0, -1.0, -1.0), .max = daxa_f32vec3(1.0, 1.0, 1.0)}};

    // TODO: Handle error
    if (!accel_struct_mngr->build_accel_structs(rigid_bodies, aabb))
      return false;
    accel_struct_mngr->build_AS();

    status_manager->update_dispatch_buffer(get_rigid_body_count());

    // Update simulation info
    rigid_body_manager->update_sim(get_rigid_body_count());

    return initialized;
  }

  daxa_u32 get_rigid_body_count()
  {
    return rigid_bodies.size();
  }
};

BB_NAMESPACE_END