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

    auto const n_body = 50u;

    rigid_bodies.reserve(n_body);

    rigid_bodies = {
      {.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, -50.0f, 0.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-50.0f, -50.0f, -50.0f), .maximum = daxa_f32vec3(50.0f, 50.0f, 50.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.5f, .friction = 0.5f}
    };

    // stack of boxes over main box
    // maybe start with 8 of them

    for(int i = 0; i < 7; ++i)
    {
      rigid_bodies.push_back({.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-0.5f, 0.5f + i, -0.5f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5f, .friction = 0.5f});
    }

    for(int i = 0; i < 7; ++i)
    {
      rigid_bodies.push_back({.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-2.5f, 0.5f + i, -0.5f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5f, .friction = 0.5f});
    }

    for(int i = 0; i < 7; ++i)
    {
      rigid_bodies.push_back({.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(1.5f, 0.5f + i, -0.5f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5f, .friction = 0.5f});
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

    // scene_1();
    scene_2();
    // scene_3();

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
      rigid_body.material_index = rigid_body.inv_mass == 0 ? 0 : distr(gen);
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
