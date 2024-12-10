#pragma once

#include "defines.hpp"
#include "math.hpp"
#include "camera_manager.hpp"
#include "rigid_body_manager.hpp"
#include "acceleration_structure_manager.hpp"
#include "status_manager.hpp"
#include <random>

BB_NAMESPACE_BEGIN

struct SceneManager
{
public:
  daxa::TaskBuffer task_material_buffer{{.name = "task_material_buffer"}};
  daxa::TaskBuffer task_lights_buffer{{.name = "task_lights_buffer"}};

  explicit SceneManager(char const *name, daxa::Device &device, std::shared_ptr<AccelerationStructureManager> accel_struct_mngr, std::shared_ptr<RigidBodyManager> rigid_body_manager, std::shared_ptr<StatusManager> status_manager, std::shared_ptr<TaskManager> task_manager) : device(device), accel_struct_mngr(accel_struct_mngr), rigid_body_manager(rigid_body_manager), status_manager(status_manager), task_manager(task_manager)
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

    material_buffer = device.create_buffer({
        .size = sizeof(Material) * MAX_MATERIAL_COUNT,
        .name = "material_buffer",
    });

    lights_buffer = device.create_buffer({
        .size = sizeof(Light) * MAX_LIGHT_COUNT,
        .name = "lights_buffer",
    });

    task_material_buffer.set_buffers({.buffers = std::array{material_buffer}});
    task_lights_buffer.set_buffers({.buffers = std::array{lights_buffer}});

    record_material_upload_tasks(material_TG);
    material_TG.submit();
    material_TG.complete();

    record_light_upload_tasks(light_TG);
    light_TG.submit();
    light_TG.complete();

    return initialized = true;
  }

  void destroy()
  {
    if (!initialized)
    {
      return;
    }

    device.destroy_buffer(material_buffer);
    device.destroy_buffer(lights_buffer);

    initialized = false;
  }

  void record_material_upload_tasks(TaskGraph &M_TG)
  {
    daxa::InlineTaskInfo task_materials({
        .attachments = {
            daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_material_buffer),
        },
        .task = [this](daxa::TaskInterface const &ti)
        {
          allocate_fill_copy(ti, materials, ti.get(task_material_buffer), 0);
        },
        .name = "upload materials",
    });
    std::array<daxa::TaskBuffer, 1> buffers = {
      task_material_buffer
    };
    std::array<daxa::InlineTaskInfo, 1> tasks = {
      task_materials
    };
    M_TG = task_manager->create_task_graph("Material Upload", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, {}, {});
  }

  void record_light_upload_tasks(TaskGraph &L_TG)
  {
    daxa::InlineTaskInfo task_lights({
        .attachments = {
            daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_lights_buffer),
        },
        .task = [this](daxa::TaskInterface const &ti)
        {
          allocate_fill_copy(ti, lights, ti.get(task_lights_buffer), 0);
        },
        .name = "upload lights",
    });
    std::array<daxa::TaskBuffer, 1> buffers = {
      task_lights_buffer
    };
    std::array<daxa::InlineTaskInfo, 1> tasks = {
      task_lights
    };
    L_TG = task_manager->create_task_graph("Light Upload", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, {}, {});
  }

  // TODO: temporary scenes
  void scene_1() {
    materials = {
      {
        .albedo = daxa_f32vec3(0.1, 0.1, 0.1),
        .emission = daxa_f32vec3(0.0, 0.0, 0.0),
      },
      {
        .albedo = daxa_f32vec3(1.0, 0.0, 0.0),
        .emission = daxa_f32vec3(10.0, 10.0, 10.0),
      },
      {
        .albedo = daxa_f32vec3(0.0, 1.0, 0.0),
        .emission = daxa_f32vec3(0.0, 0.0, 0.0),
      },
      {
        .albedo = daxa_f32vec3(0.0, 0.0, 1.0),
        .emission = daxa_f32vec3(0.0, 0.0, 0.0),
      },
      {
        .albedo = daxa_f32vec3(1.0, 1.0, 0.0),
        .emission = daxa_f32vec3(0.0, 0.0, 0.0),
      },
      {
        .albedo = daxa_f32vec3(1.0, 0.0, 1.0),
        .emission = daxa_f32vec3(0.0, 0.0, 0.0),
      },
      {
        .albedo = daxa_f32vec3(0.0, 1.0, 1.0),
        .emission = daxa_f32vec3(0.0, 0.0, 0.0),
      },
    };

    rigid_bodies = {
      {.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0, -50.0, 0.0), .rotation = Quaternion(0.0, 0.0, 0.0, 1.0), .minimum = daxa_f32vec3(-50.0, -50.0, -50.0), .maximum = daxa_f32vec3(50.0, 50.0, 50.0), .mass = 0.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5, .friction = 0.5}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-0.5, 2.0, -0.5), .rotation = Quaternion(0.4572f, 0.0000f, -0.4572f, -0.7629f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.7, .friction = 0.3}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-3.0, 5.0, -1.0), .rotation = Quaternion(0.0, 0.0, 0.0, 1.0), .minimum = daxa_f32vec3(-1.0, -1.0, -1.0), .maximum = daxa_f32vec3(1.0, 1.0, 1.0), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 10), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.1, .friction = 0.6}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.3, 1.7, 0.3), .rotation = Quaternion(-0.4572f, 0.0000f, 0.4572f, -0.7629f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 2.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.3, .friction = 0.6}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-0.5, 0.25, 0.5), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.4, .friction = 0.2}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(2.5, 3.0, 2.5), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5, .friction = 0.5}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(1.5, 3.0, 2.3), .rotation = Quaternion(0.0000f, 0.0000f, 0.5000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.6, .friction = 0.7}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-2.5, 3.0, 2.5), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.7, .friction = 0.8}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-2.5, 3.25, 1.5), .rotation = Quaternion(0.5000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.8, .friction = 0.9}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(3.5, 2.0, 0.5), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.9, .friction = 0.6}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(2.5, 2.0, -0.2), .rotation = Quaternion(0.0000f, 1.0000f, 1.0000f, 0.8000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.9, .friction = 0.5}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(4.0, 2.5, -2.0), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.8, .friction = 0.5}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(5.0, 2.5, -2.7), .rotation = Quaternion(0.0000f, 1.0000f, 1.0000f, 0.8000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.7, .friction = 0.4}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(6.0, 3.5, 0.0), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.6, .friction = 0.7}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(5.0, 3.5, 0.7), .rotation = Quaternion(0.0000f, 1.0000f, 1.0000f, 0.8000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5, .friction = 0.3}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(6.5, 1.5, 0.0), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.4, .friction = 0.6}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(7.5, 1.5, 0.9), .rotation = Quaternion(0.0000f, 1.0000f, 1.0000f, 0.8000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.3, .friction = 0.9}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(6.9, 2.5, 4.5), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.2, .friction = 0.3}, 
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(5.8, 3.5, 4.5), .rotation = Quaternion(0.0000f, 1.0000f, 0.0000f, -0.5000f), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.1, .friction = 0.3}
    };
  }

  void scene_2() {
    materials = {
      {
        .albedo = daxa_f32vec3(0.1, 0.1, 0.1),
        .emission = daxa_f32vec3(0.0, 0.0, 0.0),
      },
      {
        .albedo = daxa_f32vec3(1.0, 0.0, 0.0),
        .emission = daxa_f32vec3(10.0, 10.0, 10.0),
      },
      {
        .albedo = daxa_f32vec3(0.0, 1.0, 0.0),
        .emission = daxa_f32vec3(0.0, 0.0, 0.0),
      },
      {
        .albedo = daxa_f32vec3(0.0, 0.0, 1.0),
        .emission = daxa_f32vec3(0.0, 0.0, 0.0),
      },
      {
        .albedo = daxa_f32vec3(1.0, 1.0, 0.0),
        .emission = daxa_f32vec3(0.0, 0.0, 0.0),
      },
      {
        .albedo = daxa_f32vec3(1.0, 0.0, 1.0),
        .emission = daxa_f32vec3(0.0, 0.0, 0.0),
      },
      {
        .albedo = daxa_f32vec3(0.0, 1.0, 1.0),
        .emission = daxa_f32vec3(0.0, 0.0, 0.0),
      },
    };

    auto const n_body = 500u;

    rigid_bodies.reserve(n_body);

    rigid_bodies = {
      {.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0, -50.0, 0.0), .rotation = Quaternion(0.0, 0.0, 0.0, 1.0), .minimum = daxa_f32vec3(-50.0, -50.0, -50.0), .maximum = daxa_f32vec3(50.0, 50.0, 50.0), .mass = 0.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5, .friction = 0.5}
    };
    
    std::uniform_real_distribution<float> x_distr(-15.0, 15.0); // define the range
    std::uniform_real_distribution<float> y_distr(2.0, 7.0); // define the range
    std::uniform_real_distribution<float> z_distr(-15.0, 15.0); // define the range

    for(int i = 0; i < n_body; ++i)
    {
      rigid_bodies.push_back({.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(x_distr(gen), y_distr(gen), z_distr(gen)), .rotation = Quaternion(0.0, 0.0, 0.0, 1.0), .minimum = daxa_f32vec3(-0.5, -0.5, -0.5), .maximum = daxa_f32vec3(0.5, 0.5, 0.5), .mass = 5.0, .inv_mass = 1.0, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .tmp_velocity = daxa_f32vec3(0, 0, 0), .tmp_omega = daxa_f32vec3(0, 0, 0), .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5, .friction = 0.5});
    }
  }

  bool load_scene()
  {
    if (!initialized)
    {
      return false;
    }
    
    std::random_device rd; // obtain a random number from hardware
    gen = std::mt19937(rd()); // seed the generator
  
    scene_1();
    // scene_2();

    std::uniform_int_distribution<> distr(1, materials.size()-1); // define the range

    aabb.clear();
    aabb.reserve(rigid_bodies.size());
    daxa_u32 i = 0;
    for(auto &rigid_body : rigid_bodies)
    {
      rigid_body.active_index = MAX_U32;
      rigid_body.inv_mass = rigid_body.mass == 0.0f ? 0.0f :
      1.0f / rigid_body.mass;
      rigid_body.inv_inertia = cuboid_get_inverse_intertia(rigid_body.inv_mass, rigid_body.minimum, rigid_body.maximum);
      aabb.push_back(Aabb(rigid_body.minimum, rigid_body.maximum));
      rigid_body.material_index = rigid_body.inv_mass == 0 ? 0 : distr(gen);
      if(materials.at(rigid_body.material_index).emission != daxa_f32vec3(0.0, 0.0, 0.0))
      {
        lights.push_back(Light(i));
      }
      ++i;

      if(rigid_body.flags & RigidBodyFlag::DYNAMIC)
      {
        rigid_body_map[rigid_body_active_count++] = rigid_body_count;
      }
      ++rigid_body_count;
    }

    // TODO: Compute queue here to push an update for all frames?
    // Update simulation info
    rigid_body_manager->update_sim();
    rigid_body_manager->update_active_rigid_body_list();
    status_manager->next_frame();
    rigid_body_manager->update_sim();
    rigid_body_manager->update_active_rigid_body_list();
    status_manager->next_frame();

    material_TG.execute();
    light_TG.execute();

    // TODO: Handle error
    if (!accel_struct_mngr->build_accel_structs(rigid_bodies, aabb))
      return false;
    accel_struct_mngr->build_AS();

    return initialized;
  }

  daxa_u32 get_rigid_body_count()
  {
    return rigid_body_count;
  }
  daxa_u32 get_active_rigid_body_count()
  {
    return rigid_body_active_count;
  }
  daxa_u32 get_light_count()
  {
    return lights.size();
  }

  std::vector<ActiveRigidBody> get_active_rigid_bodies() {
    std::vector<ActiveRigidBody> active_rigid_bodies;
    active_rigid_bodies.reserve(rigid_body_active_count);
    for(auto &pair : rigid_body_map)
    {
      active_rigid_bodies.push_back(ActiveRigidBody{.rigid_body_index = pair.second});
    }
    return active_rigid_bodies;
  }

private:
  // Device
  daxa::Device &device;
  // Acceleration structure manager reference
  std::shared_ptr<AccelerationStructureManager> accel_struct_mngr;
  // Rigid body manager reference
  std::shared_ptr<RigidBodyManager> rigid_body_manager;
  // Status manager reference
  std::shared_ptr<StatusManager> status_manager;
  // Task manager reference
  std::shared_ptr<TaskManager> task_manager;
  // Initialization flag
  bool initialized = false;

  daxa_u32 rigid_body_count = 0;
  daxa_u32 rigid_body_active_count = 0;
  // TODO: Fill in scene data from file?
  std::vector<RigidBody> rigid_bodies;
  std::vector<Aabb> aabb;

  // Active rigid body buffer
  daxa::BufferId active_rigid_body_buffer;
  // store active rigid body indices for key and rigid body indices for value
  std::unordered_map<daxa_u32, daxa_u32> rigid_body_map;
  // TaskGraph for active rigid body list upload
  TaskGraph ARB_TG;

  // Material vector
  std::vector<Material> materials;
  // Material buffer
  daxa::BufferId material_buffer;
  // TaskGraph for material upload
  TaskGraph material_TG;

  // Light vector
  std::vector<Light> lights;
  // Lights buffer
  daxa::BufferId lights_buffer;
  // TaskGraph for light upload
  TaskGraph light_TG;

  std::mt19937 gen;
};

BB_NAMESPACE_END