#pragma once

#include "defines.hpp"
#include "math.hpp"
#include "camera_manager.hpp"
#include "rigid_body_manager.hpp"
#include "acceleration_structure_manager.hpp"
#include "status_manager.hpp"
#include <random>
#include <cmath>

BB_NAMESPACE_BEGIN

struct SceneManager
{
public:
  daxa::TaskBuffer task_material_buffer{{.name = "task_material_buffer"}};
  daxa::TaskBuffer task_lights_buffer{{.name = "task_lights_buffer"}};

  explicit SceneManager(char const *, daxa::Device &device, std::shared_ptr<AccelerationStructureManager> accel_struct_mngr, std::shared_ptr<RigidBodyManager> rigid_body_manager, std::shared_ptr<StatusManager> status_manager, std::shared_ptr<TaskManager> task_manager) : device(device), accel_struct_mngr(accel_struct_mngr), rigid_body_manager(rigid_body_manager), status_manager(status_manager), task_manager(task_manager)
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

    task_material_buffer.set_buffer(material_buffer);
    task_lights_buffer.set_buffer(lights_buffer);

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
        .albedo = daxa_f32vec3(0.1f, 0.1f, 0.1f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 0.0f, 0.0f),
        .emission = daxa_f32vec3(10.0f, 10.0f, 10.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 1.0f, 0.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 0.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 1.0f, 0.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 0.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 1.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 0.5f, 0.5f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.5f, 1.0f, 0.5f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.5f, 0.5f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.5f, 1.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 0.5f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 1.0f, 0.5f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
    };

    rigid_bodies = {
      {.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, -50.0f, 0.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-50.0f, -50.0f, -50.0f), .maximum = daxa_f32vec3(50.0f, 50.0f, 50.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.5f, .friction = 0.5f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-0.5f, 2.0f, -0.5f), .rotation = Quaternion(0.4572f, 0.0000f, -0.4572f, -0.7629f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.7f, .friction = 0.3f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-3.0f, 5.0f, -1.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-1.0f, -1.0f, -1.0f), .maximum = daxa_f32vec3(1.0f, 1.0f, 1.0f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 10),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.1f, .friction = 0.6f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.3f, 1.7f, 0.3f), .rotation = Quaternion(-0.4572f, 0.0000f, 0.4572f, -0.7629f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 2.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.3f, .friction = 0.6f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-0.5f, 0.25f, 0.5f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.4f, .friction = 0.2f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(2.5f, 3.0f, 2.5f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5f, .friction = 0.5f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(1.5f, 3.0f, 2.3f), .rotation = Quaternion(0.0000f, 0.0000f, 0.5000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.6f, .friction = 0.7f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-2.5f, 3.0f, 2.5f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.7f, .friction = 0.8f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-2.5f, 3.25f, 1.5f), .rotation = Quaternion(0.5000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.8f, .friction = 0.9f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(3.5f, 2.0f, 0.5f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.9f, .friction = 0.6f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(2.5f, 2.0f, -0.2f), .rotation = Quaternion(0.0000f, 1.0000f, 1.0000f, 0.8000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.9f, .friction = 0.5f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(4.0f, 2.5f, -2.0f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.8f, .friction = 0.5f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(5.0f, 2.5f, -2.7f), .rotation = Quaternion(0.0000f, 1.0000f, 1.0000f, 0.8000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.7f, .friction = 0.4f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(6.0f, 3.5f, 0.0f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.6f, .friction = 0.7f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(5.0f, 3.5f, 0.7f), .rotation = Quaternion(0.0000f, 1.0000f, 1.0000f, 0.8000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5f, .friction = 0.3f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(6.5f, 1.5f, 0.0f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.4f, .friction = 0.6f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(7.5f, 1.5f, 0.9f), .rotation = Quaternion(0.0000f, 1.0000f, 1.0000f, 0.8000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.3f, .friction = 0.9f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(6.9f, 2.5f, 4.5f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.2f, .friction = 0.3f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(5.8f, 3.5f, 4.5f), .rotation = Quaternion(0.0000f, 1.0000f, 0.0000f, -0.5000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.1f, .friction = 0.3f}
    };
  }

  void scene_2() {
    materials = {
      {
        .albedo = daxa_f32vec3(0.1f, 0.1f, 0.1f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 0.0f, 0.0f),
        .emission = daxa_f32vec3(10.0f, 10.0f, 10.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 1.0f, 0.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 0.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 1.0f, 0.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 0.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 1.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 0.5f, 0.5f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.5f, 1.0f, 0.5f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.5f, 0.5f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.5f, 1.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 0.5f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 1.0f, 0.5f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
    };

    // auto const n_body = 250u;
    // auto const n_body = 500u;
    // FIXME: slow
    auto const n_body = 1000u;

    rigid_bodies.reserve(n_body);

    rigid_bodies = {
      {.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, -50.0f, 0.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-50.0f, -50.0f, -50.0f), .maximum = daxa_f32vec3(50.0f, 50.0f, 50.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.5f, .friction = 0.5f}
    };

    std::uniform_real_distribution<float> x_distr(-15.0f, 15.0f); // define the range
    std::uniform_real_distribution<float> y_distr(2.0f, 7.0f); // define the range
    std::uniform_real_distribution<float> z_distr(-15.0f, 15.0f); // define the range

    for(int i = 0; i < n_body; ++i)
    {
      rigid_bodies.push_back({.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(x_distr(gen), y_distr(gen), z_distr(gen)), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5f, .friction = 0.5f});
    }
  }

  void scene_3() {
    materials = {
      {
        .albedo = daxa_f32vec3(0.1f, 0.1f, 0.1f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 0.0f, 0.0f),
        .emission = daxa_f32vec3(10.0f, 10.0f, 10.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 1.0f, 0.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 0.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 1.0f, 0.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 0.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 1.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 0.5f, 0.5f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.5f, 1.0f, 0.5f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.5f, 0.5f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.5f, 1.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 0.5f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 1.0f, 0.5f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },

    };

    // Box-pillar stability test: three towers of increasing height (5 / 8 / 12). Long
    // constraint chains are the classic solver-convergence stress: soft/iterative solvers
    // bulge and topple tall stacks, a converging block solver keeps them rigid. Towers
    // spawn with a 5mm settle gap per level; one color per tower (green/yellow/cyan).
    // View note: the default camera looks along +z, so the 12-tower (+x) is screen-LEFT.
    rigid_bodies = {
      {.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, -50.0f, 0.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-50.0f, -50.0f, -50.0f), .maximum = daxa_f32vec3(50.0f, 50.0f, 50.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 0.5f}
    };

    // static emissive panel high above (same lighting pattern as scene_4)
    rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, 22.0f, 14.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-5.0f, -0.2f, -5.0f), .maximum = daxa_f32vec3(5.0f, 0.2f, 5.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 0.5f});
    rigid_bodies.back().material_index = 1u; // emissive

    auto push_tower = [&](daxa_f32 x, daxa_f32 z, int height, daxa_u32 mat) {
      for (int i = 0; i < height; ++i)
      {
        rigid_bodies.push_back({.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(x, 0.5f + i * 1.005f, z), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 0.2f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.0f, .friction = 0.5f});
        rigid_bodies.back().material_index = mat;
      }
    };
    push_tower(-6.0f, 14.0f,  5, 2u); // green, screen-right
    push_tower( 0.0f, 14.0f,  8, 4u); // yellow, center
    push_tower( 6.0f, 14.0f, 12, 6u); // cyan, screen-left
  }

  // Sticking / static-friction test: two static ramps (friction 1.0) with cube pairs of
  // increasing friction parked on them. The effective contact friction is
  // sqrt(mu_cube * mu_ramp), to be compared against tan(theta):
  //   gentle ramp 15 deg (tan = 0.268): mu_eff 0.2 SLIDES | 0.5 sticks | 1.0 sticks (+ tower)
  //   steep ramp  30 deg (tan = 0.577): mu_eff 0.2 SLIDES | 0.5 SLIDES | 1.0 sticks
  // Color code: magenta = mu_eff 0.2, yellow = 0.5, green = 1.0. The 2-cube tower on the
  // gentle ramp's green pair only stands with true static friction. Without sticking
  // anchors (AVBD) the "stuck" pairs creep slowly downhill instead of pinning in place.
  // Validated against AVBD: every outcome matches plain Coulomb theory exactly (a probe
  // run measured resting lam_t = m g sin(theta) to 4 digits and re-stick from a 1 m/s
  // kick). NOTE for screenshot readers: the default camera looks along +z, so world +x
  // appears on the LEFT of the screen - the steep ramp (dark red, +x) is screen-left,
  // the gentle ramp (gray, -x) is screen-right.
  void scene_4() {
    materials = {
      {
        .albedo = daxa_f32vec3(0.1f, 0.1f, 0.1f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 1.0f, 1.0f),
        .emission = daxa_f32vec3(10.0f, 10.0f, 10.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 1.0f, 0.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 0.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 1.0f, 0.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        // bluer magenta: resists the warm sky tint that washes (1,0,1) toward orange
        .albedo = daxa_f32vec3(0.55f, 0.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 1.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        // dark red-brown: the STEEP ramp, so the two ramps are unambiguous on screen
        .albedo = daxa_f32vec3(0.30f, 0.08f, 0.06f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
    };

    // floor (same slab as the other scenes: top surface at y = 0)
    rigid_bodies = {
      {.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, -50.0f, 0.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-50.0f, -50.0f, -50.0f), .maximum = daxa_f32vec3(50.0f, 50.0f, 50.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 1.0f}
    };

    // static emissive panel high above the ramps: stable lighting, out of the default
    // camera frame and high enough not to bloom out the ramp tops
    rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, 20.0f, 14.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-5.0f, -0.2f, -5.0f), .maximum = daxa_f32vec3(5.0f, 0.2f, 5.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 1.0f});
    rigid_bodies.back().material_index = 1u; // emissive

    auto const deg = 3.14159265f / 180.0f;
    daxa_f32 const th_a = 15.0f * deg; // gentle ramp, downhill -x (tilt about +Z)
    daxa_f32 const th_b = 30.0f * deg; // steep ramp, downhill +x (tilt about -Z)
    Quaternion const q_a = Quaternion(0.0f, 0.0f, std::sin(th_a * 0.5f), std::cos(th_a * 0.5f));
    Quaternion const q_b = Quaternion(0.0f, 0.0f, -std::sin(th_b * 0.5f), std::cos(th_b * 0.5f));
    // ramp local axes in world space (x_l = up-slope-ish, y_l = surface normal)
    daxa_f32vec3 const xl_a = daxa_f32vec3(std::cos(th_a), std::sin(th_a), 0.0f);
    daxa_f32vec3 const yl_a = daxa_f32vec3(-std::sin(th_a), std::cos(th_a), 0.0f);
    daxa_f32vec3 const xl_b = daxa_f32vec3(std::cos(th_b), -std::sin(th_b), 0.0f);
    daxa_f32vec3 const yl_b = daxa_f32vec3(std::sin(th_b), std::cos(th_b), 0.0f);
    // ramp centers chosen so each foot ends just above the floor
    daxa_f32vec3 const ramp_a_c = daxa_f32vec3(-7.0f, 2.3f, 14.0f);
    daxa_f32vec3 const ramp_b_c = daxa_f32vec3(7.0f, 3.9f, 14.0f);

    auto push_ramp = [&](daxa_f32vec3 c, Quaternion q) {
      rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = c, .rotation = q, .minimum = daxa_f32vec3(-7.0f, -0.4f, -4.5f), .maximum = daxa_f32vec3(7.0f, 0.4f, 4.5f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 1.0f});
    };
    push_ramp(ramp_a_c, q_a);
    push_ramp(ramp_b_c, q_b);
    rigid_bodies.back().material_index = 7u; // steep ramp in dark red (screen-left)

    // one cube flush on a ramp surface: u = along-slope offset, z = world z, lift = ramp
    // half-height + cube half-size + 2cm settle gap (or stacked tower levels)
    auto push_cube = [&](daxa_f32vec3 c, daxa_f32vec3 xl, daxa_f32vec3 yl, Quaternion q,
                         daxa_f32 u, daxa_f32 z, daxa_f32 mu, daxa_u32 mat, daxa_f32 lift) {
      daxa_f32vec3 p = daxa_f32vec3(c.x + u * xl.x + lift * yl.x,
                                    c.y + u * xl.y + lift * yl.y,
                                    z);
      rigid_bodies.push_back({.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = p, .rotation = q, .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 0.2f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.0f, .friction = mu});
      rigid_bodies.back().material_index = mat;
    };
    daxa_f32 const lift0 = 0.4f + 0.5f + 0.005f; // 5mm settle gap (2cm rocked the tower on spawn)
    // friction pairs (mu_cube -> mu_eff against the friction-1 ramps): 0.04 -> 0.2 magenta,
    // 0.25 -> 0.5 yellow, 1.0 -> 1.0 green. Each row owns a DISJOINT z band so sliding
    // rows pass beside the sticking rows (sharing a z lane lets an upslope slider bowl
    // the stickers downhill) and nothing hides behind anything from the default camera.
    // gentle ramp (downhill -x): magenta slides, yellow and green stick
    push_cube(ramp_a_c, xl_a, yl_a, q_a, 3.5f, 16.2f, 0.04f, 5u, lift0);
    push_cube(ramp_a_c, xl_a, yl_a, q_a, 3.5f, 17.8f, 0.04f, 5u, lift0);
    push_cube(ramp_a_c, xl_a, yl_a, q_a, 1.75f, 13.4f, 0.25f, 4u, lift0);
    push_cube(ramp_a_c, xl_a, yl_a, q_a, 1.75f, 15.0f, 0.25f, 4u, lift0);
    push_cube(ramp_a_c, xl_a, yl_a, q_a, 0.0f, 10.6f, 1.0f, 2u, lift0);
    push_cube(ramp_a_c, xl_a, yl_a, q_a, 0.0f, 12.2f, 1.0f, 2u, lift0);
    // 1 extra cube stacked on the gentle ramp's first green cube (a 2-high tower on the
    // slope: tip lever 1.0*tan15 = 0.27 < 0.5 half-footprint, held by static friction;
    // 3-high was marginal, 0.40, and toppled from the spawn settle)
    push_cube(ramp_a_c, xl_a, yl_a, q_a, 0.0f, 10.6f, 1.0f, 2u, lift0 + 1.005f);
    // steep ramp (downhill +x, up-slope is -x_l): magenta AND yellow slide, green sticks
    push_cube(ramp_b_c, xl_b, yl_b, q_b, -3.5f, 16.2f, 0.04f, 5u, lift0);
    push_cube(ramp_b_c, xl_b, yl_b, q_b, -3.5f, 17.8f, 0.04f, 5u, lift0);
    push_cube(ramp_b_c, xl_b, yl_b, q_b, -1.75f, 13.4f, 0.25f, 4u, lift0);
    push_cube(ramp_b_c, xl_b, yl_b, q_b, -1.75f, 15.0f, 0.25f, 4u, lift0);
    push_cube(ramp_b_c, xl_b, yl_b, q_b, 0.0f, 10.6f, 1.0f, 2u, lift0);
    push_cube(ramp_b_c, xl_b, yl_b, q_b, 0.0f, 12.2f, 1.0f, 2u, lift0);
  }

  bool load_scene()
  {
    if (!initialized)
    {
      return false;
    }

    std::random_device rd; // obtain a random number from hardware
    gen = std::mt19937(rd()); // seed the generator

    // scene_1();
    // scene_2();
    scene_3();
    // scene_4();

    std::uniform_int_distribution<> distr(1, static_cast<int>(materials.size() - 1)); // define the range

    aabb.clear();
    aabb.reserve(rigid_bodies.size());
    for(auto &rigid_body : rigid_bodies)
    {
      rigid_body.id = id_generator;
      rigid_body.island_index = MAX_U32;
      rigid_body.manifold_node_index = MAX_U32;
      rigid_body.active_index = MAX_U32;
      rigid_body.inv_mass = rigid_body.mass == 0.0f ? 0.0f :
      1.0f / rigid_body.mass;
      rigid_body.inv_inertia = cuboid_get_inverse_intertia(rigid_body.inv_mass, rigid_body.minimum, rigid_body.maximum);
      aabb.push_back(Aabb(rigid_body.minimum, rigid_body.maximum));
      // scenes may pre-assign a material (e.g. scene_4 color-codes friction); 0 = unset
      if (rigid_body.material_index == 0u)
      {
        rigid_body.material_index = rigid_body.inv_mass == 0 ? 0u : static_cast<daxa_u32>(distr(gen));
      }
      if(materials.at(rigid_body.material_index).emission != daxa_f32vec3(0.0f, 0.0f, 0.0f))
      {
        lights.push_back(Light(rigid_body.id));
      }

      if(rigid_body.flags & RigidBodyFlag::DYNAMIC)
      {
        rigid_body_map[rigid_body_active_count++] = rigid_body.id;
      }
      ++id_generator;
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
    if (!accel_struct_mngr->build_accel_structs(rigid_bodies, aabb)) {
      std::cerr << "ERROR: Failed to build acceleration structures in scene_manager!" << std::endl;
      return false;
    }
    accel_struct_mngr->build_AS();

    // Rebuild the TLAS instances via the GPU update shader so the INITIAL (pre-simulation)
    // render uses the same instance-transform convention as the runtime path. The CPU-side
    // get_instance_transform() produces a transposed rotation relative to the intersection
    // shader's world_to_object(), which makes rotated cubes render with clipped/beveled
    // corners. Running the GPU instance update once here makes the static frame correct.
    accel_struct_mngr->update_TLAS();

    std::cout << "SUCCESS: Scene loaded successfully with " << rigid_body_count << " rigid bodies!" << std::endl;
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
    return static_cast<daxa_u32>(lights.size());
  }

  std::vector<ActiveRigidBody> get_active_rigid_bodies() {
    std::vector<ActiveRigidBody> active_rigid_bodies;
    active_rigid_bodies.reserve(rigid_body_active_count);
    for(auto &pair : rigid_body_map)
    {
      active_rigid_bodies.push_back(ActiveRigidBody{.rigid_body_id = pair.second});
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


  daxa_u32 id_generator = 0;
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
