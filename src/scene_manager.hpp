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
    rigid_bodies = {
      {.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0, -50.0, 0.0), .rotation = Quaternion(0.0, 0.0, 0.0, 1.0), .minimum = daxa_f32vec3(-50.0, -50.0, -50.0), .maximum = daxa_f32vec3(50.0, 50.0, 50.0), .mass = 0.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5, .friction = 0.5}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-0.5, 2.0, -0.5), .rotation = Quaternion(0.4572f, 0.0000f, -0.4572f, -0.7629f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.7, .friction = 0.3}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-3.0, 5.0, -1.0), .rotation = Quaternion(0.0, 0.0, 0.0, 1.0), .minimum = daxa_f32vec3(-1.0, -1.0, -1.0), .maximum = daxa_f32vec3(1.0, 1.0, 1.0), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 10), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.1, .friction = 0.6}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.3, 1.7, 0.3), .rotation = Quaternion(-0.4572f, 0.0000f, 0.4572f, -0.7629f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 2.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.3, .friction = 0.6}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-0.5, 0.25, 0.5), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.4, .friction = 0.2}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(2.5, 3.0, 2.5), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5, .friction = 0.5}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(1.5, 3.0, 2.3), .rotation = Quaternion(0.0000f, 0.0000f, 0.5000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.6, .friction = 0.7}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-2.5, 3.0, 2.5), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.7, .friction = 0.8}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-2.5, 3.25, 1.5), .rotation = Quaternion(0.5000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.8, .friction = 0.9}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(3.5, 2.0, 0.5), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.9, .friction = 0.6}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(2.5, 2.0, -0.2), .rotation = Quaternion(0.0000f, 1.0000f, 1.0000f, 0.8000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.9, .friction = 0.5}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(4.0, 2.5, -2.0), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.8, .friction = 0.5}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(5.0, 2.5, -2.7), .rotation = Quaternion(0.0000f, 1.0000f, 1.0000f, 0.8000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.7, .friction = 0.4}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(6.0, 3.5, 0.0), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.6, .friction = 0.7}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(5.0, 3.5, 0.7), .rotation = Quaternion(0.0000f, 1.0000f, 1.0000f, 0.8000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5, .friction = 0.3}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(6.5, 1.5, 0.0), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.4, .friction = 0.6}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(7.5, 1.5, 0.9), .rotation = Quaternion(0.0000f, 1.0000f, 1.0000f, 0.8000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.3, .friction = 0.9}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(6.9, 2.5, 4.5), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.2, .friction = 0.3}, 
      // {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(5.8, 3.5, 4.5), .rotation = Quaternion(0.0000f, 1.0000f, 0.0000f, -0.5000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.1, .friction = 0.3}
    };

    aabb.clear();
    aabb.reserve(rigid_bodies.size());
    for(auto &rigid_body : rigid_bodies)
    {
      rigid_body.inv_mass = rigid_body.mass == 0.0f ? 0.0f :
      1.0f / rigid_body.mass;
      rigid_body.inv_inertia = cuboid_get_inverse_intertia(rigid_body.inv_mass, rigid_body.minimum, rigid_body.maximum);
      aabb.push_back(Aabb(rigid_body.minimum, rigid_body.maximum));
    }

    status_manager->update_dispatch_buffer(get_rigid_body_count());

    // TODO: Compute queue here to push an update for all frames?
    // Update simulation info
    rigid_body_manager->update_sim(get_rigid_body_count());
    status_manager->next_frame();
    rigid_body_manager->update_sim(get_rigid_body_count());
    status_manager->next_frame();

    // TODO: Handle error
    if (!accel_struct_mngr->build_accel_structs(rigid_bodies, aabb))
      return false;
    accel_struct_mngr->build_AS();

    return initialized;
  }

  daxa_u32 get_rigid_body_count()
  {
    return rigid_bodies.size();
  }
};

BB_NAMESPACE_END