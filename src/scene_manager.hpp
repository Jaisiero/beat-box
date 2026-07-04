#pragma once

#include "defines.hpp"
#include "math.hpp"
#include "camera_manager.hpp"
#include "rigid_body_manager.hpp"
#include "acceleration_structure_manager.hpp"
#include "status_manager.hpp"
#include <random>
#include <cmath>
#include <cstdlib> // std::getenv / std::atoi (BB_SCENE headless scene selection)
#include <fstream> // BB_SCENE_FILE / BB_SCENE_DUMP data-driven scenes (F3)
#include <sstream>
#include <string>
#include <map>       // fracture: component census
#include <set>       // fracture: per-batch body dedup
#include <algorithm> // fracture: component ordering

BB_NAMESPACE_BEGIN

// getenv wrapper that silences MSVC C4996 for a read-only env lookup (this header is included by TUs
// that don't #define _CRT_SECURE_NO_WARNINGS).
inline char const *bb_getenv(char const *key)
{
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
  return std::getenv(key);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

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
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(1.5f, 3.0f, 2.3f), .rotation = Quaternion(0.0000f, 0.0000f, 0.4472136f, 0.8944272f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.6f, .friction = 0.7f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-2.5f, 3.0f, 2.5f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.7f, .friction = 0.8f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-2.5f, 3.25f, 1.5f), .rotation = Quaternion(0.4472136f, 0.0000f, 0.0000f, 0.8944272f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.8f, .friction = 0.9f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(3.5f, 2.0f, 0.5f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.9f, .friction = 0.6f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(2.5f, 2.0f, -0.2f), .rotation = Quaternion(0.0000f, 0.6154575f, 0.6154575f, 0.4923660f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.9f, .friction = 0.5f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(4.0f, 2.5f, -2.0f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.8f, .friction = 0.5f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(5.0f, 2.5f, -2.7f), .rotation = Quaternion(0.0000f, 0.6154575f, 0.6154575f, 0.4923660f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.7f, .friction = 0.4f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(6.0f, 3.5f, 0.0f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.6f, .friction = 0.7f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(5.0f, 3.5f, 0.7f), .rotation = Quaternion(0.0000f, 0.6154575f, 0.6154575f, 0.4923660f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.5f, .friction = 0.3f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(6.5f, 1.5f, 0.0f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.4f, .friction = 0.6f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(7.5f, 1.5f, 0.9f), .rotation = Quaternion(0.0000f, 0.6154575f, 0.6154575f, 0.4923660f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.3f, .friction = 0.9f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(6.9f, 2.5f, 4.5f), .rotation = Quaternion(0.0000f, 0.0000f, 0.0000f, 1.0000f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.2f, .friction = 0.3f},
      {.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(5.8f, 3.5f, 4.5f), .rotation = Quaternion(0.0000f, 0.8944272f, 0.0000f, -0.4472136f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 1.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.1f, .friction = 0.3f}
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
    // spawn with a 1mm settle gap per level (gentler landing than 5mm); one color per
    // tower (green/yellow/cyan). NOTE: from the pitched default camera, off-center
    // vertical towers PROJECT slightly tilted (perspective keystone) - the center tower
    // renders perfectly vertical, which is the reference for judging real lean.
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
        rigid_bodies.push_back({.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(x, 0.5f + i * 1.001f, z), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 0.2f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.0f, .friction = 0.5f});
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

  // ---- voxel collision shapes (Teardown-style concave bodies) ----
  // Builds the occupancy bitmask, the surface-voxel list and the mass properties from a
  // solid(x,y,z) predicate over a small grid. The BODY origin is the center of mass; the
  // shape stores grid_origin = corner of voxel (0,0,0) in the body frame. Returns the
  // 1-based id for RigidBody::shape_index (0 = no voxel shape).
  struct VoxelShapeBuild
  {
    daxa_u32 shape_id; // 1-based
    daxa_f32 mass;
    daxa_f32mat3x3 inv_inertia;
    daxa_f32vec3 minimum;
    daxa_f32vec3 maximum;
    daxa_u32 primitive_count;
  };

  template <typename F>
  VoxelShapeBuild build_voxel_shape(glm::uvec3 dims, f32 vs, f32 density, F &&solid)
  {
    auto idx = [&](u32 x, u32 y, u32 z) { return x + y * dims.x + z * dims.x * dims.y; };
    auto is_solid = [&](i32 x, i32 y, i32 z) -> bool {
      if (x < 0 || y < 0 || z < 0 || x >= (i32)dims.x || y >= (i32)dims.y || z >= (i32)dims.z) return false;
      return solid((u32)x, (u32)y, (u32)z);
    };

    // center of mass (uniform density) in grid space
    glm::dvec3 com(0.0);
    u32 count = 0;
    for (u32 z = 0; z < dims.z; ++z)
      for (u32 y = 0; y < dims.y; ++y)
        for (u32 x = 0; x < dims.x; ++x)
          if (solid(x, y, z)) { com += glm::dvec3(x + 0.5, y + 0.5, z + 0.5); ++count; }
    com = com / (f64)count * (f64)vs;
    f32 const voxel_mass = density * vs * vs * vs;
    f32 const mass = voxel_mass * (f32)count;
    glm::vec3 const grid_origin = -glm::vec3(com); // body frame: CoM at the origin

    // occupancy bits + surface list + inertia about the CoM
    u32 const bit_count = dims.x * dims.y * dims.z;
    u32 const occ_offset = (u32)voxel_occ_cpu.size();
    voxel_occ_cpu.resize(occ_offset + (bit_count + 31u) / 32u, 0u);
    // surface ENTRIES are built ON THE GPU (build_voxel_pools_gpu, canonical cell order);
    // the CPU fill below runs only as the BB_SDF_VERIFY oracle. The pool slice is reserved
    // at the worst case (every solid cell on the surface) so offsets never depend on the env.
    static bool const sdf_verify_oracle = bb_getenv("BB_SDF_VERIFY") != nullptr;
    u32 const surf_offset = (u32)voxel_surf_cpu.size();
    voxel_surf_cpu.resize(surf_offset + count, 0u);
    u32 surf_n = 0;
    glm::mat3 inertia(0.0f);
    std::vector<Aabb> prims;
    prims.reserve(count);
    for (u32 z = 0; z < dims.z; ++z)
      for (u32 y = 0; y < dims.y; ++y)
        for (u32 x = 0; x < dims.x; ++x)
        {
          if (!solid(x, y, z)) continue;
          u32 const bit = idx(x, y, z);
          voxel_occ_cpu[occ_offset + bit / 32u] |= 1u << (bit % 32u);
          glm::vec3 const center = grid_origin + glm::vec3(x + 0.5f, y + 0.5f, z + 0.5f) * vs;
          // parallel-axis voxel inertia: point term + own-cube term
          f32 const r2 = glm::dot(center, center);
          inertia += voxel_mass * (glm::mat3(r2) - glm::outerProduct(center, center));
          inertia += glm::mat3(voxel_mass * vs * vs / 6.0f);
          // BLAS AABBs are built ON THE GPU (entry_voxel_prims_build via the AS post-upload
          // hook); the CPU fill is only the BB_SDF_VERIFY oracle
          if (sdf_verify_oracle)
          {
            prims.push_back(Aabb(daxa_f32vec3(center.x - 0.5f * vs, center.y - 0.5f * vs, center.z - 0.5f * vs),
                                 daxa_f32vec3(center.x + 0.5f * vs, center.y + 0.5f * vs, center.z + 0.5f * vs)));
          }
          // surface voxel: any of the 6 neighbors empty; normal_code = first empty direction.
          // count always (record field, trivial); the ENTRY only under the oracle env
          i32 const nx[6][3] = {{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
          for (u32 d = 0; d < 6; ++d)
          {
            if (!is_solid((i32)x + nx[d][0], (i32)y + nx[d][1], (i32)z + nx[d][2]))
            {
              if (sdf_verify_oracle) { voxel_surf_cpu[surf_offset + surf_n] = x | (y << 8) | (z << 16) | (d << 24); }
              ++surf_n;
              break;
            }
          }
        }

    // NODE signed-distance field: the REAL build runs ON THE GPU (build_voxel_pools_gpu,
    // exact separable EDT from the occupancy bitmask - the destructibility rebuild path).
    // This CPU brute force is kept ONLY as the BB_SDF_VERIFY oracle and is skipped
    // otherwise (GPU-first directive: CPU algorithms are temporary/debug scaffolding).
    // The pool slice is always RESERVED (sdf_offset accounting must not depend on the env).
    u32 const sdf_offset = (u32)voxel_sdf_cpu.size();
    glm::uvec3 const ndims = dims + glm::uvec3(1);
    voxel_sdf_cpu.resize(sdf_offset + ndims.x * ndims.y * ndims.z, 0.0f);
    auto point_to_cell = [&](glm::vec3 p, u32 cx, u32 cy, u32 cz) -> f32 {
      glm::vec3 const lo((f32)cx, (f32)cy, (f32)cz);
      glm::vec3 const d = glm::max(glm::max(lo - p, p - (lo + glm::vec3(1.0f))), glm::vec3(0.0f));
      return glm::length(d);
    };
    if (sdf_verify_oracle)
    for (u32 nz = 0; nz < ndims.z; ++nz)
      for (u32 ny = 0; ny < ndims.y; ++ny)
        for (u32 nx_ = 0; nx_ < ndims.x; ++nx_)
        {
          glm::vec3 const p((f32)nx_, (f32)ny, (f32)nz); // grid units
          f32 d_solid = 1e30f, d_empty = 1e30f;
          for (u32 z = 0; z < dims.z; ++z)
            for (u32 y = 0; y < dims.y; ++y)
              for (u32 x = 0; x < dims.x; ++x)
              {
                f32 const dc = point_to_cell(p, x, y, z);
                if (solid(x, y, z)) { d_solid = std::min(d_solid, dc); }
                else                { d_empty = std::min(d_empty, dc); }
              }
          // outside the grid box is all empty: distance from an interior point to the box hull
          f32 const d_out = std::min({p.x, p.y, p.z, (f32)dims.x - p.x, (f32)dims.y - p.y, (f32)dims.z - p.z});
          d_empty = std::min(d_empty, std::max(d_out, 0.0f));
          f32 const sd = d_solid > 0.0f ? d_solid : -d_empty; // on-surface nodes: both 0
          voxel_sdf_cpu[sdf_offset + nx_ + ny * ndims.x + nz * ndims.x * ndims.y] = sd * vs;
        }

    if (voxel_occ_cpu.size() > BB_MAX_VOXEL_OCC_U32S || voxel_surf_cpu.size() > BB_MAX_VOXEL_SURF_COUNT ||
        voxel_sdf_cpu.size() > BB_MAX_VOXEL_SDF_F32S || voxel_shape_cpu.size() >= BB_MAX_VOXEL_SHAPE_COUNT)
    {
      std::cerr << "ERROR: voxel shape pools exceeded!" << std::endl;
    }

    voxel_shape_cpu.push_back(VoxelShape{
        .dims = daxa_u32vec3(dims.x, dims.y, dims.z),
        .voxel_size = vs,
        .grid_origin = daxa_f32vec3(grid_origin.x, grid_origin.y, grid_origin.z),
        .occ_offset = occ_offset,
        .surf_offset = surf_offset,
        .surf_count = surf_n,
        .sdf_offset = sdf_offset,
    });
    shape_private.push_back(false); // load shapes are shared between bodies (fracture clones them)
    if (!sdf_verify_oracle)
    {
      // placeholder range with the right SIZE: the aabb vector's voxel ranges are
      // overwritten on the GPU before any BLAS build reads them
      prims.resize(count, Aabb(daxa_f32vec3(0.0f, 0.0f, 0.0f), daxa_f32vec3(0.0f, 0.0f, 0.0f)));
    }
    voxel_shape_prims.push_back(std::move(prims));
    // CPU authoring mass properties, recorded as the GPU inertia-reduce's verify twin
    // (unit voxel mass; the kernel is the future fracture path's fragment-mass primitive)
    voxel_derived_cpu.push_back(VoxelShapeDerived{
        .count = count,
        .com = daxa_f32vec3((f32)com.x, (f32)com.y, (f32)com.z),
        .unit_inertia = daxa_mat3_from_glm_mat3(inertia * (1.0f / voxel_mass)),
    });

    return VoxelShapeBuild{
        .shape_id = (u32)voxel_shape_cpu.size(), // 1-based
        .mass = mass,
        .inv_inertia = daxa_mat3_from_glm_mat3(glm::inverse(inertia)),
        .minimum = daxa_f32vec3(grid_origin.x, grid_origin.y, grid_origin.z),
        .maximum = daxa_f32vec3(grid_origin.x + dims.x * vs, grid_origin.y + dims.y * vs, grid_origin.z + dims.z * vs),
        .primitive_count = (u32)voxel_shape_prims.back().size(),
    };
  }

  // AS post-upload hook: fills every voxel body's AABB range in the primitive scratch ON
  // THE GPU (entry_voxel_prims_build) between the host upload and the BLAS build. Both
  // build_accel_structs call sites (load + reset) must pass it - the CPU aabb entries for
  // voxel ranges are zeros unless BB_SDF_VERIFY authored the oracle values.
  std::function<void(daxa::BufferId)> voxel_prims_hook()
  {
    return [this](daxa::BufferId prims_buffer) {
      rigid_body_manager->build_voxel_prims_gpu(voxel_shape_cpu, voxel_prim_sites, prims_buffer, aabb);
    };
  }

  void push_voxel_body(VoxelShapeBuild const &s, daxa_f32vec3 pos, Quaternion rot, daxa_u32 mat, f32 friction,
                       f32 fracture_impulse = 0.0f)
  {
    rigid_bodies.push_back({.flags = (RigidBodyFlag::DYNAMIC | RigidBodyFlag::GRAVITY), .primitive_count = s.primitive_count, .primitive_offset = 0, .shape_index = s.shape_id, .position = pos, .rotation = rot, .minimum = s.minimum, .maximum = s.maximum, .mass = s.mass, .inv_mass = 1.0f / s.mass, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .inv_inertia = s.inv_inertia, .restitution = 0.0f, .friction = friction, .fracture_impulse = fracture_impulse});
    rigid_bodies.back().material_index = mat;
  }

  // Voxel-shape showcase: concave bodies (L / cross / hollow frame) dropping into a pile,
  // plus a static post under one frame (the V3 "threading" money shot). V0 INTERIM physics:
  // bodies collide as their bounding boxes (the SAT path) until the voxel narrow phase
  // lands, so visual interpenetration of the concave parts is expected for now.
  void scene_5() {
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
        .albedo = daxa_f32vec3(0.55f, 0.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 1.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.30f, 0.08f, 0.06f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
    };

    // floor + lighting panel (same pattern as scene_3/4)
    rigid_bodies = {
      {.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, -50.0f, 0.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-50.0f, -50.0f, -50.0f), .maximum = daxa_f32vec3(50.0f, 50.0f, 50.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 0.6f}
    };
    rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, 22.0f, 14.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-5.0f, -0.2f, -5.0f), .maximum = daxa_f32vec3(5.0f, 0.2f, 5.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 0.6f});
    rigid_bodies.back().material_index = 1u; // emissive

    // static post for the frame-threading showcase (dark red, world +x = screen-left)
    rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(8.0f, 4.0f, 14.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-0.35f, -4.0f, -0.35f), .maximum = daxa_f32vec3(0.35f, 4.0f, 0.35f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 0.6f});
    rigid_bodies.back().material_index = 7u;

    f32 const vs = 0.5f;
    f32 const density = 2.0f;
    // L: 6x2x2 base + 2x4x2 column (40 voxels, 3x3x1 world units)
    auto l_shape = build_voxel_shape(glm::uvec3(6, 6, 2), vs, density,
        [](u32 x, u32 y, u32) { return y < 2 || x < 2; });
    // cross / plus (40 voxels)
    auto cross_shape = build_voxel_shape(glm::uvec3(6, 6, 2), vs, density,
        [](u32 x, u32 y, u32) { return (x >= 2 && x < 4) || (y >= 2 && y < 4); });
    // hollow frame: 8x8x2 closed ring with a 4x4 hole (2x2 world units - the 0.7-wide
    // post threads with clearance). The hole predicate is for an 8x8 grid: with the old
    // 6x6 dims it silently produced an OPEN corner (two walls), not a ring
    auto frame_shape = build_voxel_shape(glm::uvec3(8, 8, 2), vs, density,
        [](u32 x, u32 y, u32) { return !(x >= 2 && x < 6 && y >= 2 && y < 6); });

    Quaternion const q_id = Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    Quaternion const q_x90 = Quaternion(0.7071f, 0.0f, 0.0f, 0.7071f); // hole axis vertical

    // small pile of concave bodies (greens = L, yellow = cross, violet = frame)
    push_voxel_body(l_shape,     daxa_f32vec3(-6.0f, 2.5f, 13.0f), q_id, 2u, 0.6f);
    push_voxel_body(l_shape,     daxa_f32vec3(-5.4f, 5.5f, 14.6f), q_x90, 2u, 0.6f);
    push_voxel_body(l_shape,     daxa_f32vec3(-2.0f, 8.0f, 13.8f), q_id, 2u, 0.6f);
    push_voxel_body(cross_shape, daxa_f32vec3(-3.5f, 2.5f, 14.8f), q_id, 4u, 0.6f);
    push_voxel_body(cross_shape, daxa_f32vec3(-4.5f, 11.0f, 14.0f), q_x90, 4u, 0.6f);
    push_voxel_body(cross_shape, daxa_f32vec3(0.5f, 4.0f, 14.2f), q_id, 4u, 0.6f);
    push_voxel_body(frame_shape, daxa_f32vec3(-1.0f, 12.5f, 14.4f), q_id, 5u, 0.6f);
    push_voxel_body(frame_shape, daxa_f32vec3(2.5f, 7.0f, 13.4f), q_x90, 5u, 0.6f);
    // the showcase frame: spawns flat above the post (threads onto it once the voxel
    // narrow phase exists; rests on the post's bounding box until then)
    push_voxel_body(frame_shape, daxa_f32vec3(8.0f, 11.0f, 14.0f), q_x90, 5u, 0.6f);
  }

  // DETERMINISTIC voxel jitter probe: three isolated test cases, tiny settle gaps, no
  // randomness - run-to-run comparable, and spatially separated so the accumulator
  // render (sharp = asleep, blurry = dancing) attributes jitter to a specific case.
  //   A (x ~ -6): one L flat on the floor              -> must sleep almost instantly
  //   B (x ~  0): one L leaning ~30 deg on a block     -> tilted-contact stability
  //   C (x ~ +6): cross + two Ls stacked into a tangle -> multi-direction contacts
  void scene_6() {
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
        .albedo = daxa_f32vec3(0.55f, 0.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 1.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.30f, 0.08f, 0.06f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
    };

    rigid_bodies = {
      {.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, -50.0f, 0.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-50.0f, -50.0f, -50.0f), .maximum = daxa_f32vec3(50.0f, 50.0f, 50.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 0.6f}
    };
    rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, 22.0f, 14.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-5.0f, -0.2f, -5.0f), .maximum = daxa_f32vec3(5.0f, 0.2f, 5.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 0.6f});
    rigid_bodies.back().material_index = 1u; // emissive

    // case B's static block (dark red)
    rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.8f, 1.0f, 14.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-0.5f, -1.0f, -2.0f), .maximum = daxa_f32vec3(0.5f, 1.0f, 2.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 0.6f});
    rigid_bodies.back().material_index = 7u;

    f32 const vs = 0.5f;
    f32 const density = 2.0f;
    auto l_shape = build_voxel_shape(glm::uvec3(6, 6, 2), vs, density,
        [](u32 x, u32 y, u32) { return y < 2 || x < 2; });
    auto cross_shape = build_voxel_shape(glm::uvec3(6, 6, 2), vs, density,
        [](u32 x, u32 y, u32) { return (x >= 2 && x < 4) || (y >= 2 && y < 4); });

    Quaternion const q_id = Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    Quaternion const q_x90 = Quaternion(0.7071f, 0.0f, 0.0f, 0.7071f);   // lying flat
    Quaternion const q_lean = Quaternion(0.0f, 0.0f, 0.2588f, 0.9659f);  // 30 deg about Z
    Quaternion const q_tilt = Quaternion(0.2164f, 0.0f, 0.0f, 0.9763f);  // 25 deg about X

    // A: L lying flat on the floor (green)
    push_voxel_body(l_shape, daxa_f32vec3(-6.0f, 0.51f, 14.0f), q_x90, 2u, 0.6f);
    // B: L spawned tilted 30 deg next to the block; falls a few mm and leans on it (yellow)
    push_voxel_body(l_shape, daxa_f32vec3(-0.9f, 1.65f, 14.0f), q_lean, 4u, 0.6f);
    // C: tangle - cross flat on the floor, an L tilted 25 deg resting across it, and a
    //    second L leaning onto the first (violet/green/yellow)
    push_voxel_body(cross_shape, daxa_f32vec3(6.0f, 0.51f, 14.0f), q_x90, 5u, 0.6f);
    push_voxel_body(l_shape, daxa_f32vec3(6.2f, 1.45f, 14.3f), q_tilt, 2u, 0.6f);
    push_voxel_body(l_shape, daxa_f32vec3(7.3f, 1.30f, 13.8f), q_lean, 4u, 0.6f);
    // D: STACK - L flat dropped centered onto a flat cross (the user-reported unstable
    //    configuration: a body resting entirely on another's small top faces)
    push_voxel_body(cross_shape, daxa_f32vec3(-12.0f, 0.51f, 14.0f), q_x90, 5u, 0.6f);
    push_voxel_body(l_shape, daxa_f32vec3(-12.0f, 1.56f, 14.0f), q_x90, 2u, 0.6f);
    // E: CANTILEVERED stack - L dropped OFF-CENTER onto a flat cross so it overhangs
    //    (marginal support near the patch edge, the tangle-like case)
    push_voxel_body(cross_shape, daxa_f32vec3(-18.0f, 0.51f, 14.0f), q_x90, 5u, 0.6f);
    push_voxel_body(l_shape, daxa_f32vec3(-17.2f, 1.56f, 14.6f), q_x90, 4u, 0.6f);
  }

  // scene_7: BOX POOL - a narrow, TALL pit filled by a deep stack of unit cubes.
  // Depth (a ~8-cube column once settled) maximizes per-contact load and the length of
  // the warm-start convergence chain: THE stress test for stacked contacts. Walls
  // confine the crust so nothing rolls away. Success criterion: EVERY cube asleep
  // (sleeping=432, fresh=0, maxv=0), red contact points only.
  void scene_7() {
    materials = {
      {
        .albedo = daxa_f32vec3(0.1f, 0.1f, 0.1f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(1.0f, 1.0f, 1.0f),
        .emission = daxa_f32vec3(30.0f, 30.0f, 30.0f),
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
        .albedo = daxa_f32vec3(0.55f, 0.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.0f, 1.0f, 1.0f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
      {
        .albedo = daxa_f32vec3(0.30f, 0.08f, 0.06f),
        .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f),
      },
    };

    // floor + lighting panel. The panel sits ABOVE the spawn column (cubes rain down
    // from up to y~43 and would otherwise bounce off it); brighter to compensate.
    rigid_bodies = {
      {.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, -50.0f, 0.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-50.0f, -50.0f, -50.0f), .maximum = daxa_f32vec3(50.0f, 50.0f, 50.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 0.6f}
    };
    rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, 50.0f, 14.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-8.0f, -0.2f, -8.0f), .maximum = daxa_f32vec3(8.0f, 0.2f, 8.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 0.6f});
    rigid_bodies.back().material_index = 1u; // emissive

    // the pit: interior 8x8 world units, walls 10 high (dark red statics) - low enough
    // for the default camera (y=16) to look over the rim, tall enough that the ~7-deep
    // settled heap stays confined. Static-static wall/floor pairs are skipped by the
    // narrow phase, overlapping corners are free.
    auto push_wall = [&](daxa_f32vec3 pos, daxa_f32vec3 half) {
      rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = pos, .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-half.x, -half.y, -half.z), .maximum = half, .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 0.6f});
      rigid_bodies.back().material_index = 7u;
    };
    push_wall(daxa_f32vec3( 4.25f, 5.0f, 14.0f), daxa_f32vec3(0.25f, 5.0f, 4.5f));
    push_wall(daxa_f32vec3(-4.25f, 5.0f, 14.0f), daxa_f32vec3(0.25f, 5.0f, 4.5f));
    push_wall(daxa_f32vec3(0.0f, 5.0f, 18.25f), daxa_f32vec3(4.5f, 5.0f, 0.25f));
    push_wall(daxa_f32vec3(0.0f, 5.0f, 9.75f), daxa_f32vec3(4.5f, 5.0f, 0.25f));

    // 12 waves x 6x6 unit cubes (432) RAINING into the pit: vertical spacing 2.8 from
    // y=12 up to ~43, so the waves arrive staggered in time and pile onto the already
    // settled heap. Deterministic index-hashed jitter AND TILT (no RNG, exactly
    // reproducible): the tilts (4-13 deg about a hashed horizontal bearing) make the
    // cubes tumble into a disordered heap instead of landing as a lattice.
    for (u32 iy = 0; iy < 12; ++iy) {
      for (u32 iz = 0; iz < 6; ++iz) {
        for (u32 ix = 0; ix < 6; ++ix) {
          u32 const h1 = ix * 3u + iy * 5u + iz * 7u;
          u32 const h2 = ix * 7u + iy * 3u + iz * 5u;
          u32 const h3 = ix * 5u + iy * 7u + iz * 3u;
          f32 const jx = (f32(h1 % 11u) - 5.0f) * 0.05f;  // +-0.25
          f32 const jz = (f32(h2 % 11u) - 5.0f) * 0.05f;  // +-0.25
          f32 const jy = (f32(h3 % 11u) - 5.0f) * 0.20f;  // +-1.0: staggers each wave
          f32 const phi = f32(h3 % 16u) * 0.3927f;        // tilt axis bearing
          f32 const ang = 0.07f + f32(h1 % 7u) * 0.025f;  // 4-13 deg
          f32 const sh = std::sin(0.5f * ang);
          Quaternion const q_tilt = Quaternion(std::cos(phi) * sh, 0.0f, std::sin(phi) * sh, std::cos(0.5f * ang));
          rigid_bodies.push_back({.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(-3.0f + 1.2f * ix + jx, 12.0f + 2.8f * iy + jy, 11.0f + 1.2f * iz + jz), .rotation = q_tilt, .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 0.2f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.0f, .friction = 0.6f});
        }
      }
    }
  }

  // scene_8: SINGLE-CUBE free-fall test. One dynamic unit cube dropped from y=6 onto the floor
  // (top at y=0), no neighbours, no walls. With only one dynamic body the pocket_trace.csv columns
  // become a clean per-step fall log: maxv_mm = the cube's speed, and global_pen_mm goes nonzero
  // the frame the cube touches the floor (solver-agnostic) = the exact time-to-fall to compare
  // AVBD vs TGS. y=6 keeps the peak speed (~10.4 m/s) under the 12 m/s anti-punch-through clamp,
  // so it is pure unclamped free fall. (Temporary measurement scene.)
  void scene_8() {
    materials = {
      { .albedo = daxa_f32vec3(0.1f, 0.1f, 0.1f),  .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) },
      { .albedo = daxa_f32vec3(1.0f, 1.0f, 1.0f),  .emission = daxa_f32vec3(30.0f, 30.0f, 30.0f) },
      { .albedo = daxa_f32vec3(0.0f, 1.0f, 0.0f),  .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) },
    };
    // floor (top at y=0)
    rigid_bodies = {
      {.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, -50.0f, 0.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-50.0f, -50.0f, -50.0f), .maximum = daxa_f32vec3(50.0f, 50.0f, 50.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 0.6f}
    };
    // emissive light panel above
    rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, 22.0f, 14.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-5.0f, -0.2f, -5.0f), .maximum = daxa_f32vec3(5.0f, 0.2f, 5.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(0)), .restitution = 0.0f, .friction = 0.6f});
    rigid_bodies.back().material_index = 1u; // emissive
    // THE single dynamic cube: dropped straight down from y=6 (no rotation, no initial velocity)
    rigid_bodies.push_back({.flags = (RigidBodyFlag::DYNAMIC|RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, 6.0f, 14.0f), .rotation = Quaternion(0.0f, 0.0f, 0.0f, 1.0f), .minimum = daxa_f32vec3(-0.5f, -0.5f, -0.5f), .maximum = daxa_f32vec3(0.5f, 0.5f, 0.5f), .mass = 5.0f, .inv_mass = 0.2f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0),  .inv_inertia = daxa_mat3_from_glm_mat3(glm::mat3(1)), .restitution = 0.0f, .friction = 0.6f});
    rigid_bodies.back().material_index = 2u; // green
  }

  // FRACTURE SHOWCASE: three voxel structures with material strength coded by color, each
  // bombarded by a heavy projectile. Left green = weak (shatters), middle yellow = medium
  // (cracks), right violet = strong (withstands the same hit). Projectiles are staggered in
  // height so the impacts cascade in sequence. Strengths were CALIBRATED from the measured
  // per-contact stopping-impulse demand at these masses/heights (see [FRACTURE] logs).
  void scene_9() {
    materials = {
      { .albedo = daxa_f32vec3(0.1f, 0.1f, 0.1f),   .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) },  // 0 floor
      { .albedo = daxa_f32vec3(1.0f, 1.0f, 1.0f),   .emission = daxa_f32vec3(18.0f, 18.0f, 18.0f) }, // 1 light
      { .albedo = daxa_f32vec3(0.15f, 0.95f, 0.15f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) },  // 2 weak green
      { .albedo = daxa_f32vec3(0.98f, 0.85f, 0.10f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) },  // 3 medium yellow
      { .albedo = daxa_f32vec3(0.60f, 0.10f, 0.95f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) },  // 4 strong violet
      { .albedo = daxa_f32vec3(0.55f, 0.57f, 0.60f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) },  // 5 projectile steel
      { .albedo = daxa_f32vec3(0.30f, 0.08f, 0.06f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) },  // 6 static dark red
    };

    auto const Q = Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    auto const I0 = daxa_mat3_from_glm_mat3(glm::mat3(0));
    auto const I1 = daxa_mat3_from_glm_mat3(glm::mat3(1));
    // floor (top at y=0) + wide emissive panel overhead
    rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, -50.0f, 0.0f), .rotation = Q, .minimum = daxa_f32vec3(-50.0f, -50.0f, -50.0f), .maximum = daxa_f32vec3(50.0f, 50.0f, 50.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .inv_inertia = I0, .restitution = 0.0f, .friction = 0.7f});
    auto light = RigidBody{.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, 24.0f, 14.0f), .rotation = Q, .minimum = daxa_f32vec3(-11.0f, -0.2f, -6.0f), .maximum = daxa_f32vec3(11.0f, 0.2f, 6.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .inv_inertia = I0, .restitution = 0.0f, .friction = 0.6f};
    light.material_index = 1u;
    rigid_bodies.push_back(light);

    f32 const vs = 0.5f;
    f32 const density = 2.0f;
    // thin BEAM (14 x 2 x 3 = 84 voxels, 7 x 1 x 1.5 world units). Each beam is DROPPED flat
    // onto a small static ANVIL centered under it. A fracture fires with high demand only
    // when the voxel body ITSELF slams an immovable object (something hitting a free-standing
    // beam just makes it yield - measured demand ~5); the anvil concentrates the whole ~12
    // m/s stop at the beam's center, and the crater punches clean through the thin (1-unit)
    // beam → it SNAPS into a left and a right half that fall off the anvil (a real
    // connected-components split). com at the geometric center → half-height 0.5.
    auto beam = build_voxel_shape(glm::uvec3(14, 2, 3), vs, density,
        [](u32, u32, u32) { return true; });

    // three columns along x. strengths CALIBRATED to the measured anvil-drop demand: weak
    // snaps and the halves re-shatter, medium snaps once, strong takes the hit and holds.
    struct Col { f32 x; daxa_u32 mat; f32 strength; f32 drop_y; };
    Col const cols[3] = {
        {-9.0f, 2u, 18.0f,   9.0f},  // weak green
        { 0.0f, 3u, 70.0f,  14.0f},  // medium yellow
        { 9.0f, 4u, 250.0f, 19.0f},  // strong violet
    };
    for (auto const &c : cols)
    {
      // static anvil (dark red) poking up under the beam center — concentrates the impact
      rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(c.x, 0.75f, 14.0f), .rotation = Q, .minimum = daxa_f32vec3(-0.7f, -0.75f, -0.9f), .maximum = daxa_f32vec3(0.7f, 0.75f, 0.9f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .inv_inertia = I0, .restitution = 0.0f, .friction = 0.7f});
      rigid_bodies.back().material_index = 6u;
      // beam dropped flat from a STAGGERED height (impacts land one at a time — a fracture
      // respawn does a global AS+sim refresh that would disturb another impact in flight).
      // All drops clear the ~12 m/s terminal clamp, so impact hardness is EQUAL across
      // columns and the only variable is strength.
      push_voxel_body(beam, daxa_f32vec3(c.x, c.drop_y, 14.0f), Q, c.mat, 0.7f, c.strength);
    }
  }

  // F3: data-driven scene from a text file (BB_SCENE_FILE=path). One dynamic cube per line:
  //   px py pz [half_extent] [mass] [restitution] [friction]   (# comments and blank lines ignored)
  // A floor + an emissive light panel are added automatically; the common load_scene() post-pass fills
  // in inv_mass / inv_inertia / aabb / random materials. Lets you iterate on repro scenes with no rebuild.
  void scene_from_file(std::string const &path)
  {
    materials = {
      {.albedo = daxa_f32vec3(0.1f, 0.1f, 0.1f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f)}, // 0 static/floor
      {.albedo = daxa_f32vec3(1.0f, 1.0f, 1.0f), .emission = daxa_f32vec3(10.0f, 10.0f, 10.0f)}, // 1 light
      {.albedo = daxa_f32vec3(1.0f, 0.0f, 0.0f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f)},
      {.albedo = daxa_f32vec3(0.0f, 1.0f, 0.0f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f)},
      {.albedo = daxa_f32vec3(0.0f, 0.0f, 1.0f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f)},
      {.albedo = daxa_f32vec3(1.0f, 1.0f, 0.0f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f)},
      {.albedo = daxa_f32vec3(0.0f, 1.0f, 1.0f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f)},
      {.albedo = daxa_f32vec3(1.0f, 0.0f, 1.0f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f)},
    };
    auto const Q = Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    auto const I0 = daxa_mat3_from_glm_mat3(glm::mat3(0));
    auto const I1 = daxa_mat3_from_glm_mat3(glm::mat3(1));
    // floor (static; top at y=0)
    rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, -50.0f, 0.0f), .rotation = Q, .minimum = daxa_f32vec3(-50.0f, -50.0f, -50.0f), .maximum = daxa_f32vec3(50.0f, 50.0f, 50.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .inv_inertia = I0, .restitution = 0.0f, .friction = 0.6f});
    // emissive light panel (static; material index 1 -> becomes a light in the post-pass)
    auto light = RigidBody{.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, 22.0f, 0.0f), .rotation = Q, .minimum = daxa_f32vec3(-5.0f, -0.2f, -5.0f), .maximum = daxa_f32vec3(5.0f, 0.2f, 5.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .inv_inertia = I0, .restitution = 0.0f, .friction = 0.6f};
    light.material_index = 1u;
    rigid_bodies.push_back(light);
    // dynamic cubes from the file
    std::ifstream in(path);
    if (!in) { std::cerr << "BB_SCENE_FILE: cannot open '" << path << "'" << std::endl; return; }
    // Second line format (the thin-feature wedge investigation — voxel repros as text files):
    //   vox <l|cross|frame> px py pz [qx qy qz qw]   -> a concave voxel piece (scene_5 shape set)
    bool shapes_built = false;
    VoxelShapeBuild vox_l{}, vox_cross{}, vox_frame{};
    auto ensure_shapes = [&]() {
      if (shapes_built) { return; }
      f32 const vvs = 0.5f; f32 const vdensity = 2.0f;
      vox_l = build_voxel_shape(glm::uvec3(6, 6, 2), vvs, vdensity,
          [](u32 x, u32 y, u32) { return y < 2 || x < 2; });
      vox_cross = build_voxel_shape(glm::uvec3(6, 6, 2), vvs, vdensity,
          [](u32 x, u32 y, u32) { return (x >= 2 && x < 4) || (y >= 2 && y < 4); });
      vox_frame = build_voxel_shape(glm::uvec3(8, 8, 2), vvs, vdensity,
          [](u32 x, u32 y, u32) { return !(x >= 2 && x < 6 && y >= 2 && y < 6); });
      shapes_built = true;
    };
    daxa_u32 n = 0u;
    std::string line;
    while (std::getline(in, line))
    {
      auto const s = line.find_first_not_of(" \t\r\n");
      if (s == std::string::npos || line[s] == '#') { continue; }
      std::istringstream ss(line);
      if (line.compare(s, 4, "vox ") == 0)
      {
        std::string kw, shape;
        float px, py, pz, qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f, qt, strength = 0.0f;
        ss >> kw >> shape;
        if (!(ss >> px >> py >> pz)) { continue; }
        if (ss >> qt) { qx = qt; if (ss >> qt) qy = qt; if (ss >> qt) qz = qt; if (ss >> qt) qw = qt; }
        // optional 5th trailing value: fracture strength (impact impulse in kg*m/s; 0 = unbreakable)
        if (ss >> qt) { strength = qt; }
        Quaternion q = Quaternion(qx, qy, qz, qw).normalize(); // hand-typed quats: keep |q|==1
        ensure_shapes();
        VoxelShapeBuild const *vsb = shape == "l" ? &vox_l : shape == "cross" ? &vox_cross
                                   : shape == "frame" ? &vox_frame : nullptr;
        if (vsb == nullptr) { std::cerr << "BB_SCENE_FILE: unknown vox shape '" << shape << "'" << std::endl; continue; }
        // palette above: 3=green (l), 5=yellow (cross), 7=magenta (frame) — the scene_5 look
        daxa_u32 const vmat = shape == "l" ? 3u : shape == "cross" ? 5u : 7u;
        if (strength > 0.0f) { std::cout << "[SCENE] vox '" << shape << "' fracture strength " << strength << std::endl; }
        push_voxel_body(*vsb, daxa_f32vec3(px, py, pz), q, vmat, 0.6f, strength);
        ++n;
        continue;
      }
      float px, py, pz; float h = 0.5f, m = 5.0f, e = 0.0f, fr = 0.6f, tmp;
      if (!(ss >> px >> py >> pz)) { continue; }
      if (ss >> tmp) h = tmp;  if (ss >> tmp) m = tmp;  if (ss >> tmp) e = tmp;  if (ss >> tmp) fr = tmp;
      // optional rotation (F9 live dumps append it): px py pz h m e fr qx qy qz qw
      Quaternion cq = Q;
      float cqx, cqy, cqz, cqw;
      if (ss >> cqx >> cqy >> cqz >> cqw) { cq = Quaternion(cqx, cqy, cqz, cqw).normalize(); }
      rigid_bodies.push_back({.flags = (RigidBodyFlag::DYNAMIC | RigidBodyFlag::GRAVITY), .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(px, py, pz), .rotation = cq, .minimum = daxa_f32vec3(-h, -h, -h), .maximum = daxa_f32vec3(h, h, h), .mass = m, .inv_mass = (m == 0.0f ? 0.0f : 1.0f / m), .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .inv_inertia = I1, .restitution = e, .friction = fr});
      ++n;
    }
    std::cout << "[SCENE] BB_SCENE_FILE loaded " << n << " bodies from '" << path << "'" << std::endl;
  }

  // F3: dump the current scene's dynamic cubes in the BB_SCENE_FILE format (edit + reload without a rebuild).
  void dump_scene(std::string const &path)
  {
    std::ofstream out(path, std::ios::trunc);
    if (!out) { std::cerr << "BB_SCENE_DUMP: cannot open '" << path << "'" << std::endl; return; }
    out << "# BB_SCENE_FILE: px py pz [half_extent] [mass] [restitution] [friction] (a floor + light are auto-added)\n";
    daxa_u32 n = 0u;
    for (auto const &b : rigid_bodies)
    {
      if ((b.flags & RigidBodyFlag::DYNAMIC) == RigidBodyFlag::NONE) { continue; }
      float const h = (b.maximum.x - b.minimum.x) * 0.5f;
      out << b.position.x << " " << b.position.y << " " << b.position.z << " " << h << " " << b.mass << " " << b.restitution << " " << b.friction << "\n";
      ++n;
    }
    std::cout << "[SCENE] BB_SCENE_DUMP wrote " << n << " bodies to '" << path << "'" << std::endl;
  }

  // F9: dump the LIVE GPU poses (current-parity rigid body buffer) in BB_SCENE_FILE format.
  // Voxel bodies dump as `vox <name> pos quat` (shape_index 1/2/3 = l/cross/frame, the same
  // order scene_5 and the file loader build them); cubes dump with their rotation appended.
  // This is the capture half of the repro loop: see the bad configuration -> F9 -> load the
  // file headless with BB_SCENE_FILE and debug the exact state. CPU-side by design (debug
  // tooling; the sim itself stays GPU-resident).
  void dump_scene_live(std::string const &path)
  {
    daxa_u32 const count = rigid_body_count;
    if (count == 0u) { std::cerr << "F9 dump: no bodies" << std::endl; return; }
    daxa::BufferId src = accel_struct_mngr->get_rigid_body_buffer();
    if (src.is_empty()) { std::cerr << "F9 dump: rigid body buffer not ready" << std::endl; return; }
    auto const size = static_cast<daxa::usize>(count) * sizeof(RigidBody);
    daxa::BufferId staging = device.create_buffer({
        .size = size,
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "scene_dump_staging",
    });
    {
      auto rec = device.create_command_recorder({});
      rec.copy_buffer_to_buffer({.src_buffer = src, .dst_buffer = staging, .size = size});
      auto cmds = rec.complete_current_commands();
      device.submit_commands({.command_lists = std::array{cmds}});
      device.wait_idle();
    }
    RigidBody const *live = device.buffer_host_address_as<RigidBody>(staging).value();
    std::ofstream out(path, std::ios::trunc);
    if (!out)
    {
      std::cerr << "F9 dump: cannot open '" << path << "'" << std::endl;
      device.destroy_buffer(staging);
      return;
    }
    out << "# F9 live dump: px py pz [h m e fr qx qy qz qw] / vox <shape> px py pz qx qy qz qw\n";
    daxa_u32 n = 0u;
    for (daxa_u32 i = 0u; i < count; ++i)
    {
      RigidBody const &b = live[i];
      if ((b.flags & RigidBodyFlag::DYNAMIC) == RigidBodyFlag::NONE) { continue; }
      if (b.shape_index != 0u)
      {
        char const *name = b.shape_index == 1u ? "l" : b.shape_index == 2u ? "cross" : "frame";
        out << "vox " << name << " " << b.position.x << " " << b.position.y << " " << b.position.z
            << " " << b.rotation.v.x << " " << b.rotation.v.y << " " << b.rotation.v.z << " " << b.rotation.w << "\n";
      }
      else
      {
        float const h = (b.maximum.x - b.minimum.x) * 0.5f;
        out << b.position.x << " " << b.position.y << " " << b.position.z << " " << h << " " << b.mass
            << " " << b.restitution << " " << b.friction
            << " " << b.rotation.v.x << " " << b.rotation.v.y << " " << b.rotation.v.z << " " << b.rotation.w << "\n";
      }
      ++n;
    }
    device.destroy_buffer(staging);
    std::cout << "[SCENE] F9 live dump wrote " << n << " bodies to '" << path << "'" << std::endl;
  }

  // ================================ FRACTURE (MVP) ================================
  // Host ORCHESTRATOR for GPU fracture. The impact pass latches events into SimConfig's
  // persistent serial ring; this consumes them. All geometry/analysis is GPU (carve +
  // connected components + the pool rebuild chain); the host only moves validated numbers
  // (slot allocation, RigidBody records) and re-enters the battle-tested load/reset upload
  // path - a "mini-reload" with a full AS rebuild (a few ms at fracture instants; the
  // update_sim warm-start wipe is the known reset-path behavior, visually masked by the
  // fracture itself). Slot recycling is deliberately NOT here: pools are append-only with
  // loud overflow guards (phase 2 = free-lists).

  // by value: Quaternion::conjugate() is not const-qualified (shared shader/C++ struct)
  static daxa_f32vec3 quat_rotate(Quaternion q, daxa_f32vec3 const &v)
  {
    return (q * Quaternion(v, 0.0f) * q.conjugate()).v;
  }
  static daxa_f32vec3 quat_rotate_inv(Quaternion q, daxa_f32vec3 const &v)
  {
    return (q.conjugate() * Quaternion(v, 0.0f) * q).v;
  }

  // post-derived fixup bookkeeping: fragment records need the GPU-reduced com before they
  // can be finalized; the parent shifts to its new com. Parent kinematics are CAPTURED at
  // apply time (later events in the same batch must not see half-fixed values).
  struct FragFix
  {
    daxa_u32 body;       // host body index (== persistent id)
    daxa_f32 voxel_mass; // parent's per-voxel mass (invariant under fracture)
    glm::vec3 com_old;   // parent's pre-fracture com (from the grid min corner, world scale)
    daxa_f32vec3 parent_pos;
    Quaternion parent_rot;
    daxa_f32vec3 parent_vel;
    daxa_f32vec3 parent_omega;
  };

  // Pull the LIVE GPU body states into the host vector (the host holds the SPAWN state by
  // design - see reset()). Without this, the fracture respawn would teleport the whole
  // scene back to its initial placement. Same mechanism as the F9 live dump.
  bool sync_live_bodies()
  {
    daxa_u32 const count = rigid_body_count;
    if (count == 0u) { return false; }
    daxa::BufferId src = accel_struct_mngr->get_rigid_body_buffer();
    if (src.is_empty()) { return false; }
    auto const size = static_cast<daxa::usize>(count) * sizeof(RigidBody);
    daxa::BufferId staging = device.create_buffer({
        .size = size,
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "fracture_live_sync_staging",
    });
    {
      auto rec = device.create_command_recorder({});
      rec.copy_buffer_to_buffer({.src_buffer = src, .dst_buffer = staging, .size = size});
      auto cmds = rec.complete_current_commands();
      device.submit_commands({.command_lists = std::array{cmds}});
      device.wait_idle();
    }
    RigidBody const *live = device.buffer_host_address_as<RigidBody>(staging).value();
    for (daxa_u32 i = 0u; i < count; ++i)
    {
      RigidBody const &lb = live[i]; // GPU rows are sort-ordered; map by persistent id
      if (lb.id >= rigid_bodies.size()) { continue; }
      auto &hb = rigid_bodies[lb.id];
      hb.position = lb.position;
      hb.rotation = lb.rotation;
      hb.velocity = lb.velocity;
      hb.omega = lb.omega;
      hb.prev_velocity = lb.prev_velocity;
      hb.prev_omega = lb.prev_omega;
      hb.flags = lb.flags;
      hb.sleep_timer = lb.sleep_timer;
      // solver scratch: reset like reset() does (the respawn starts a fresh warm world)
      hb.island_index = MAX_U32;
      hb.manifold_node_index = MAX_U32;
      hb.active_index = MAX_U32;
    }
    device.destroy_buffer(staging);
    return true;
  }

  // one event: clone-if-shared, GPU carve + component labels, split components into bodies
  bool apply_fracture(FractureEvent const &ev, std::vector<FragFix> &fixes)
  {
    if (ev.body_id >= rigid_bodies.size()) { return false; }
    auto &body = rigid_bodies[ev.body_id];
    if (body.shape_index == 0u || (body.flags & RigidBodyFlag::DYNAMIC) == RigidBodyFlag::NONE) { return false; }
    daxa_u32 shape_i = body.shape_index - 1u;
    daxa_u32vec3 const dims = voxel_shape_cpu[shape_i].dims;
    daxa_u32 const cells = dims.x * dims.y * dims.z;
    daxa_u32 const words = (cells + 31u) / 32u;
    daxa_u32 const nodes = (dims.x + 1u) * (dims.y + 1u) * (dims.z + 1u);
    auto pools_full = [&]() {
      return voxel_shape_cpu.size() >= BB_MAX_VOXEL_SHAPE_COUNT ||
             voxel_occ_cpu.size() + words > BB_MAX_VOXEL_OCC_U32S ||
             voxel_surf_cpu.size() + cells > BB_MAX_VOXEL_SURF_COUNT ||
             voxel_sdf_cpu.size() + nodes > BB_MAX_VOXEL_SDF_F32S;
    };

    // CLONE-ON-FIRST-FRACTURE: load shapes are SHARED between bodies (scene_5 reuses one L
    // for three pieces) - carving a shared slice would dent every sibling. Fracture-born
    // shapes are single-owner (shape_private) and carve in place.
    if (!shape_private[shape_i])
    {
      if (pools_full())
      {
        std::cerr << "FRACTURE: shape pools full, event dropped (body " << ev.body_id << ")" << std::endl;
        return false;
      }
      VoxelShape ns = voxel_shape_cpu[shape_i];
      ns.occ_offset = (u32)voxel_occ_cpu.size();
      ns.surf_offset = (u32)voxel_surf_cpu.size();
      ns.sdf_offset = (u32)voxel_sdf_cpu.size();
      std::vector<daxa_u32> occ_copy(voxel_occ_cpu.begin() + voxel_shape_cpu[shape_i].occ_offset,
                                     voxel_occ_cpu.begin() + voxel_shape_cpu[shape_i].occ_offset + words);
      voxel_occ_cpu.insert(voxel_occ_cpu.end(), occ_copy.begin(), occ_copy.end());
      voxel_surf_cpu.resize(voxel_surf_cpu.size() + cells, 0u);
      voxel_sdf_cpu.resize(voxel_sdf_cpu.size() + nodes, 0.0f);
      voxel_shape_cpu.push_back(ns);
      shape_private.push_back(true);
      voxel_shape_prims.push_back(std::vector<Aabb>(voxel_shape_prims[shape_i].size(),
                                                    Aabb(daxa_f32vec3(0, 0, 0), daxa_f32vec3(0, 0, 0))));
      body.shape_index = (u32)voxel_shape_cpu.size(); // 1-based
      shape_i = (u32)voxel_shape_cpu.size() - 1u;
      // the GPU pool doesn't have the clone yet: re-upload occupancy + records so the
      // carve kernel sees it (small pools; fracture-rate one-off)
      std::memcpy(device.buffer_host_address_as<daxa_u32>(rigid_body_manager->get_voxel_occupancy_buffer()).value(),
                  voxel_occ_cpu.data(), voxel_occ_cpu.size() * sizeof(daxa_u32));
      rigid_body_manager->upload_voxel_shapes(voxel_shape_cpu);
    }
    VoxelShape const shape = voxel_shape_cpu[shape_i]; // stable copy for this event

    // world -> grid-space carve center
    daxa_f32vec3 const rel = daxa_f32vec3(ev.position.x - body.position.x,
                                          ev.position.y - body.position.y,
                                          ev.position.z - body.position.z);
    daxa_f32vec3 const lp = quat_rotate_inv(body.rotation, rel);
    f32 const vs = shape.voxel_size;
    daxa_f32vec3 const grid_c = daxa_f32vec3((lp.x - shape.grid_origin.x) / vs,
                                             (lp.y - shape.grid_origin.y) / vs,
                                             (lp.z - shape.grid_origin.z) / vs);
    // impulse-scaled crater: a harder hit breaks more (radius in cells; clamped so a big
    // overshoot chips instead of pulverizing - the pulverization clamp refuses total loss)
    f32 const carve_r = std::clamp(1.0f + 0.6f * (ev.impulse / body.fracture_impulse), 1.6f, 2.6f);

    // capture per-voxel mass + parent kinematics BEFORE anything changes
    daxa_u32 const old_count = (daxa_u32)voxel_shape_prims[shape_i].size();
    if (old_count == 0u) { return false; }
    FragFix const parent_ctx{ev.body_id,
                             body.mass / (f32)old_count,
                             glm::vec3(-shape.grid_origin.x, -shape.grid_origin.y, -shape.grid_origin.z),
                             body.position, body.rotation, body.velocity, body.omega};

    // GPU: carve + connected-component labels; tiny readbacks for the bookkeeping
    std::vector<daxa_u32> occ_words, labels;
    rigid_body_manager->carve_and_label(shape, grid_c, carve_r, occ_words, labels);

    // component census (labels are min cell indices -> deterministic identities)
    std::map<daxa_u32, daxa_u32> comp_counts;
    for (daxa_u32 c = 0u; c < cells; ++c)
    {
      if (labels[c] != MAX_U32) { ++comp_counts[labels[c]]; }
    }
    if (comp_counts.empty())
    {
      // full pulverization: clamp (MVP keeps bodies alive). The host slice is untouched,
      // so the respawn upload restores the GPU slice = the carve is refused.
      std::cerr << "FRACTURE: body " << ev.body_id << " would pulverize entirely; carve refused" << std::endl;
      return true; // respawn anyway to restore the GPU slice
    }
    std::vector<std::pair<daxa_u32, daxa_u32>> comps(comp_counts.begin(), comp_counts.end());
    std::sort(comps.begin(), comps.end(), [](auto const &a, auto const &b) {
      return a.second != b.second ? a.second > b.second : a.first < b.first;
    });

    auto write_component = [&](daxa_u32 occ_offset, daxa_u32 label) {
      for (daxa_u32 w = 0u; w < words; ++w) { voxel_occ_cpu[occ_offset + w] = 0u; }
      for (daxa_u32 c = 0u; c < cells; ++c)
      {
        if (labels[c] == label) { voxel_occ_cpu[occ_offset + c / 32u] |= 1u << (c % 32u); }
      }
    };

    // largest component keeps this body/shape
    write_component(shape.occ_offset, comps[0].first);
    voxel_shape_prims[shape_i].assign(comps[0].second, Aabb(daxa_f32vec3(0, 0, 0), daxa_f32vec3(0, 0, 0)));
    body.primitive_count = comps[0].second;
    fixes.push_back(parent_ctx);

    // the rest become fragment shapes + bodies (same dims: no re-indexing, tiny slices)
    for (size_t k = 1; k < comps.size(); ++k)
    {
      if (pools_full() || rigid_bodies.size() >= MAX_RIGID_BODY_COUNT)
      {
        std::cerr << "FRACTURE: pools/bodies full, fragment dropped" << std::endl;
        break;
      }
      VoxelShape fs = shape;
      fs.occ_offset = (u32)voxel_occ_cpu.size();
      fs.surf_offset = (u32)voxel_surf_cpu.size();
      fs.sdf_offset = (u32)voxel_sdf_cpu.size();
      voxel_occ_cpu.resize(voxel_occ_cpu.size() + words, 0u);
      voxel_surf_cpu.resize(voxel_surf_cpu.size() + cells, 0u);
      voxel_sdf_cpu.resize(voxel_sdf_cpu.size() + nodes, 0.0f);
      write_component(fs.occ_offset, comps[k].first);
      voxel_shape_cpu.push_back(fs);
      shape_private.push_back(true);
      voxel_shape_prims.push_back(std::vector<Aabb>(comps[k].second, Aabb(daxa_f32vec3(0, 0, 0), daxa_f32vec3(0, 0, 0))));

      RigidBody frag = body; // inherit material/friction/restitution/strength/pose
      frag.id = id_generator++;
      frag.shape_index = (u32)voxel_shape_cpu.size();
      frag.primitive_count = comps[k].second;
      frag.primitive_offset = 0u;
      frag.island_index = MAX_U32;
      frag.manifold_node_index = MAX_U32;
      frag.active_index = MAX_U32;
      frag.sleep_timer = 0u;
      frag.flags = RigidBodyFlag::DYNAMIC | RigidBodyFlag::GRAVITY;
      // mass/inertia/position/velocity are PROVISIONAL until the derived readback fixup
      rigid_bodies.push_back(frag);
      rigid_body_map[rigid_body_active_count++] = frag.id;
      ++rigid_body_count;
      FragFix ff = parent_ctx;
      ff.body = frag.id;
      fixes.push_back(ff);
    }
    return true;
  }

  // the mini-reload: pools up, GPU rebuild chain, derived readback -> final records, AS + sim refresh
  void respawn_after_fracture(std::vector<FragFix> const &fixes)
  {
    // 1. pools upload (the host vectors are authoritative again)
    std::memcpy(device.buffer_host_address_as<VoxelShape>(rigid_body_manager->get_voxel_shapes_buffer()).value(),
                voxel_shape_cpu.data(), voxel_shape_cpu.size() * sizeof(VoxelShape));
    std::memcpy(device.buffer_host_address_as<daxa_u32>(rigid_body_manager->get_voxel_occupancy_buffer()).value(),
                voxel_occ_cpu.data(), voxel_occ_cpu.size() * sizeof(daxa_u32));
    std::memcpy(device.buffer_host_address_as<daxa_u32>(rigid_body_manager->get_voxel_surface_buffer()).value(),
                voxel_surf_cpu.data(), voxel_surf_cpu.size() * sizeof(daxa_u32));
    std::memcpy(device.buffer_host_address_as<daxa_f32>(rigid_body_manager->get_voxel_sdf_buffer()).value(),
                voxel_sdf_cpu.data(), voxel_sdf_cpu.size() * sizeof(daxa_f32));
    // 2. GPU rebuild chain (SDF + surface + inertia for every shape - tiny at these sizes)
    rigid_body_manager->build_voxel_pools_gpu(voxel_shape_cpu, voxel_sdf_cpu, voxel_surf_cpu, voxel_derived_cpu);
    // 3. the GPU mass-property reduce is the AUTHORITY for the affected records
    std::vector<VoxelShapeDerived> derived;
    rigid_body_manager->read_voxel_derived((daxa_u32)voxel_shape_cpu.size(), derived);
    for (auto const &fx : fixes)
    {
      auto &b = rigid_bodies[fx.body];
      daxa_u32 const si = b.shape_index - 1u;
      VoxelShapeDerived const &d = derived[si];
      if (d.count == 0u) { continue; } // pulverization-clamp path: record untouched
      b.mass = fx.voxel_mass * (f32)d.count;
      b.inv_mass = 1.0f / b.mass;
      glm::mat3 I;
      I[0] = glm::vec3(d.unit_inertia.x.x, d.unit_inertia.x.y, d.unit_inertia.x.z);
      I[1] = glm::vec3(d.unit_inertia.y.x, d.unit_inertia.y.y, d.unit_inertia.y.z);
      I[2] = glm::vec3(d.unit_inertia.z.x, d.unit_inertia.z.y, d.unit_inertia.z.z);
      b.inv_inertia = daxa_mat3_from_glm_mat3(glm::inverse(I * fx.voxel_mass));
      // shape frame: com back at the body origin
      glm::vec3 const com_new(d.com.x, d.com.y, d.com.z);
      VoxelShape &sh = voxel_shape_cpu[si];
      sh.grid_origin = daxa_f32vec3(-com_new.x, -com_new.y, -com_new.z);
      b.minimum = sh.grid_origin;
      b.maximum = daxa_f32vec3(sh.grid_origin.x + sh.dims.x * sh.voxel_size,
                               sh.grid_origin.y + sh.dims.y * sh.voxel_size,
                               sh.grid_origin.z + sh.dims.z * sh.voxel_size);
      // kinematics from the CAPTURED parent frame: same world voxels, new com ->
      // pos' = pos + R*(com_new - com_old); v' = v + omega x (pos' - pos)
      glm::vec3 const shift = com_new - fx.com_old;
      daxa_f32vec3 const ws = quat_rotate(fx.parent_rot, daxa_f32vec3(shift.x, shift.y, shift.z));
      b.rotation = fx.parent_rot;
      b.position = daxa_f32vec3(fx.parent_pos.x + ws.x, fx.parent_pos.y + ws.y, fx.parent_pos.z + ws.z);
      glm::vec3 const w(fx.parent_omega.x, fx.parent_omega.y, fx.parent_omega.z);
      glm::vec3 const dv = glm::cross(w, glm::vec3(ws.x, ws.y, ws.z));
      b.velocity = daxa_f32vec3(fx.parent_vel.x + dv.x, fx.parent_vel.y + dv.y, fx.parent_vel.z + dv.z);
      b.omega = fx.parent_omega;
      b.prev_velocity = b.velocity;
      b.prev_omega = b.omega;
      // a fracture wakes what it touches
      b.flags = RigidBodyFlag(daxa_u32(b.flags) & ~daxa_u32(RigidBodyFlag::SLEEPING));
      b.sleep_timer = 0u;
    }
    // 4. records with fixed grid_origins back to the GPU (the prims pass reads them)
    rigid_body_manager->upload_voxel_shapes(voxel_shape_cpu);
    // 5. primitive lists in body order (same layout rule as load)
    aabb.clear();
    voxel_prim_sites.clear();
    for (auto &rb : rigid_bodies)
    {
      if (rb.shape_index == 0u)
      {
        aabb.push_back(Aabb(rb.minimum, rb.maximum));
      }
      else
      {
        voxel_prim_sites.emplace_back(rb.shape_index - 1u, (u32)aabb.size());
        auto const &prims = voxel_shape_prims.at(rb.shape_index - 1u);
        aabb.insert(aabb.end(), prims.begin(), prims.end());
      }
    }
    // 6. full AS rebuild + sim refresh (the reset() tail, minus the pause)
    accel_struct_mngr->reset_for_reload();
    if (!accel_struct_mngr->build_accel_structs(rigid_bodies, aabb, voxel_prims_hook()))
    {
      std::cerr << "FRACTURE: AS rebuild failed!" << std::endl;
      return;
    }
    accel_struct_mngr->build_AS();
    rigid_body_manager->update_sim();
    rigid_body_manager->update_active_rigid_body_list();
    status_manager->next_frame();
    rigid_body_manager->update_sim();
    rigid_body_manager->update_active_rigid_body_list();
    status_manager->next_frame();
    accel_struct_mngr->update_TLAS();
    std::cout << "[FRACTURE] respawn: " << rigid_body_count << " bodies, "
              << voxel_shape_cpu.size() << " shapes" << std::endl;
  }

  // entry point, called from the render loop after each stepped frame (event source = the
  // dedicated GPU->host bridge buffer, NOT SimConfig - see FractureEventBuffer)
  void process_fracture_events(FractureEventBuffer const &fb)
  {
    daxa_u32 const serial = fb.serial;
    if (serial == fracture_serial_seen) { return; }
    daxa_u32 n = serial - fracture_serial_seen;
    if (n > BB_MAX_FRACTURE_EVENTS)
    {
      std::cerr << "FRACTURE: " << (n - BB_MAX_FRACTURE_EVENTS) << " events dropped (ring overflow)" << std::endl;
      n = BB_MAX_FRACTURE_EVENTS;
    }
    fracture_serial_seen = serial;
    // live GPU state first: the respawn re-uploads the whole host vector
    if (!sync_live_bodies()) { return; }
    bool any = false;
    std::vector<FragFix> fixes;
    std::set<daxa_u32> done; // one carve per body per batch (an impact spams manifold rows)
    for (daxa_u32 s = serial - n; s != serial; ++s)
    {
      FractureEvent const &ev = fb.events[s % BB_MAX_FRACTURE_EVENTS];
      if (done.count(ev.body_id) != 0u) { continue; }
      done.insert(ev.body_id);
      std::cout << "[FRACTURE] body " << ev.body_id << " impulse " << ev.impulse << std::endl;
      any = apply_fracture(ev, fixes) || any;
    }
    if (any) { respawn_after_fracture(fixes); }
  }

  bool load_scene()
  {
    if (!initialized)
    {
      return false;
    }

    // Headless scene selection: BB_SCENE=N picks the launch scene ONCE (startup only, so F1-F8
    // switch_scene() still works). Lets a headless run / A-B measure any scene, not just the default.
    {
      static bool bb_scene_applied = false;
      if (!bb_scene_applied)
      {
        bb_scene_applied = true;
        // std::getenv is flagged C4996 ('unsafe') by MSVC; the .cpp files including this HEADER don't
        // all #define _CRT_SECURE_NO_WARNINGS (only main.cpp/renderer_manager.cpp do), so suppress it
        // locally — a read-only env lookup is safe.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
        if (const char *s = std::getenv("BB_SCENE")) current_scene = std::atoi(s);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
      }
    }

    // FIXED SEED (determinism): scene spawns must be identical run to run so that
    // same-machine replays, golden-trace regression and solver A/Bs compare like with
    // like (landing variance was the dominant measurement noise of the whole solver
    // campaign). Change the constant to explore different pilings.
    gen = std::mt19937(0xBEA7B0C5u);

    // Scene dispatch by current_scene (set at startup default + switched at runtime via F1-F8).
    // scene_1 .. scene_8 are compile-time builders; switch_scene() resets the host state so a
    // different builder can repopulate the shared vectors.
    // F3: BB_SCENE_FILE=path loads a data-driven scene from a text file instead of a compile-time
    // builder (no rebuild needed to iterate on a repro scene). F1-F8 still switch the built-in scenes.
    if (char const *scene_file = bb_getenv("BB_SCENE_FILE"))
    {
      scene_from_file(scene_file);
    }
    else
    {
      switch (current_scene)
      {
        case 1: scene_1(); break;
        case 2: scene_2(); break;
        case 3: scene_3(); break;
        case 4: scene_4(); break;
        case 5: scene_5(); break; // V3 showcase: frame threads onto the post + mixed concave pile
        case 6: scene_6(); break; // deterministic stability probe (rests + stacks; fresh/pen 0/single-digit)
        case 7: scene_7(); break; // box pool: 432 cubes rain into the pit (all must settle and sleep)
        case 8: scene_8(); break; // single-cube free-fall A/B (AVBD vs TGS time-to-floor)
        case 9: scene_9(); break; // FRACTURE showcase: strength-coded towers + projectiles
        default: scene_6(); break;
      }
    }
    // F3: BB_SCENE_DUMP=path writes the current scene's dynamic cubes in the BB_SCENE_FILE format.
    if (char const *dump = bb_getenv("BB_SCENE_DUMP")) { dump_scene(dump); }

    // random materials for bodies with material_index 0 (unset). Start at 2: index 1 is
    // the emissive light-panel material, and randomly emissive bodies read as glaring
    // white "lamp cubes" in every pile render
    std::uniform_int_distribution<> distr(2, static_cast<int>(materials.size() - 1));

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
      if (rigid_body.shape_index == 0u)
      {
        // legacy OBB body: cuboid inertia + a single box primitive
        rigid_body.inv_inertia = cuboid_get_inverse_intertia(rigid_body.inv_mass, rigid_body.minimum, rigid_body.maximum);
        aabb.push_back(Aabb(rigid_body.minimum, rigid_body.maximum));
      }
      else
      {
        // voxel body: inertia from the voxel sum (set by build_voxel_shape) and one AABB
        // primitive per solid voxel (the BLAS builder consumes primitive_count in body order).
        // Record the body's range so the GPU prims pass can fill it (the CPU entries are
        // zeros unless BB_SDF_VERIFY authored the oracle values).
        voxel_prim_sites.emplace_back(rigid_body.shape_index - 1u, (u32)aabb.size());
        auto const &prims = voxel_shape_prims.at(rigid_body.shape_index - 1u);
        aabb.insert(aabb.end(), prims.begin(), prims.end());
      }
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

    // upload the voxel shape pools (static for the scene's lifetime; host-writable buffers
    // owned by the rigid body manager, read by the narrow phase)
    if (!voxel_shape_cpu.empty())
    {
      std::memcpy(device.buffer_host_address_as<VoxelShape>(rigid_body_manager->get_voxel_shapes_buffer()).value(),
                  voxel_shape_cpu.data(), voxel_shape_cpu.size() * sizeof(VoxelShape));
      std::memcpy(device.buffer_host_address_as<daxa_u32>(rigid_body_manager->get_voxel_occupancy_buffer()).value(),
                  voxel_occ_cpu.data(), voxel_occ_cpu.size() * sizeof(daxa_u32));
      std::memcpy(device.buffer_host_address_as<daxa_u32>(rigid_body_manager->get_voxel_surface_buffer()).value(),
                  voxel_surf_cpu.data(), voxel_surf_cpu.size() * sizeof(daxa_u32));
      std::memcpy(device.buffer_host_address_as<daxa_f32>(rigid_body_manager->get_voxel_sdf_buffer()).value(),
                  voxel_sdf_cpu.data(), voxel_sdf_cpu.size() * sizeof(daxa_f32));
      // GPU-first: the node SDF, the surface-voxel list and the mass-property reduce are
      // (re)built ON THE GPU from the occupancy bitmask - the CPU values uploaded above
      // are only the BB_SDF_VERIFY oracles (the GPU results overwrite them, including each
      // shape's surf_count). This is the path future runtime shape edits (destruction) re-run.
      rigid_body_manager->build_voxel_pools_gpu(voxel_shape_cpu, voxel_sdf_cpu, voxel_surf_cpu, voxel_derived_cpu);
    }

    // TODO: Handle error
    if (!accel_struct_mngr->build_accel_structs(rigid_bodies, aabb, voxel_prims_hook())) {
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

  // Restart the simulation from the initial scene state. The host `rigid_bodies` vector is never
  // written by the sim (the GPU holds the live state), so it still holds the initial placement;
  // re-uploading it + resetting the SimConfig (counts/manifolds -> 0) restarts cleanly. Same body
  // ids, no RNG reseed -> deterministic restart of the SAME scene.
  bool reset()
  {
    if (!initialized)
    {
      return false;
    }
    // belt-and-suspenders: clear the per-body dynamic scratch the solver writes on the GPU side
    for (auto &rb : rigid_bodies)
    {
      rb.island_index = MAX_U32;
      rb.manifold_node_index = MAX_U32;
      rb.active_index = MAX_U32;
      rb.velocity = daxa_f32vec3(0, 0, 0);
      rb.omega = daxa_f32vec3(0, 0, 0);
      rb.sleep_timer = 0u;
    }
    // zero the incremental upload counters first, or the 2nd reset would append (864->1296 > max) and fail
    accel_struct_mngr->reset_for_reload();
    if (!accel_struct_mngr->build_accel_structs(rigid_bodies, aabb, voxel_prims_hook()))
    {
      std::cerr << "ERROR: reset() failed to re-upload rigid bodies!" << std::endl;
      return false;
    }
    accel_struct_mngr->build_AS();
    // reset both double-buffer parities of the SimConfig (collision/manifold/island counts -> 0)
    rigid_body_manager->update_sim();
    rigid_body_manager->update_active_rigid_body_list();
    status_manager->next_frame();
    rigid_body_manager->update_sim();
    rigid_body_manager->update_active_rigid_body_list();
    status_manager->next_frame();
    accel_struct_mngr->update_TLAS();
    status_manager->stop_simulating(); // reset also pauses (like Space): restart fresh, paused
    std::cout << "RESET: simulation restarted from the initial scene state (paused)." << std::endl;
    return true;
  }

  // Switch to a different scene at runtime (F1-F8): tear down the host-side scene state, rebuild
  // from scene_N(), and pause (like reset). The scene_N() builders push_back into the shared
  // vectors and assume them empty + load_scene() accumulates ids/counts/lights, so everything that
  // accumulates must be cleared here before re-running load_scene(). n is 1..8.
  bool switch_scene(int n)
  {
    if (!initialized)
    {
      return false;
    }
    if (n < 1 || n > 9)
    {
      std::cerr << "SCENE: ignoring out-of-range scene " << n << std::endl;
      return false;
    }
    if (n == current_scene)
    {
      return reset(); // same scene -> just restart it (cheaper, keeps ids)
    }
    // tear down everything load_scene()/the scene builders accumulate (materials are reassigned
    // wholesale by each scene_N, so they need no clear)
    rigid_bodies.clear();
    aabb.clear();
    lights.clear();
    rigid_body_map.clear();
    voxel_shape_cpu.clear();
    voxel_occ_cpu.clear();
    voxel_surf_cpu.clear();
    voxel_sdf_cpu.clear();
    voxel_shape_prims.clear();
    voxel_prim_sites.clear();
    voxel_derived_cpu.clear();
    shape_private.clear();
    fracture_serial_seen = 0;
    rigid_body_manager->reset_fracture_events(); // stale serials from the old scene reference dead ids
    id_generator = 0;
    rigid_body_count = 0;
    rigid_body_active_count = 0;
    // zero the incremental AS upload counters, or build_accel_structs would append onto the old
    // scene's buffers (the same hazard reset() guards against with reset_for_reload)
    accel_struct_mngr->reset_for_reload();
    current_scene = n;
    if (!load_scene())
    {
      std::cerr << "SCENE: failed to load scene " << n << std::endl;
      return false;
    }
    status_manager->stop_simulating(); // load the new scene paused (like reset)
    std::cout << "SCENE: switched to scene_" << n << " (paused)." << std::endl;
    return true;
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
  // active scene (1..8); load_scene() dispatches on it, switch_scene() (F1-F8) changes it. Default
  // is scene_7 (the 432-cube rain pool) — the pre-switch hardcoded boot scene, which the harness
  // and benchmarks assume. Change this initializer to pick a different launch scene.
  int current_scene = 7;
  // TODO: Fill in scene data from file?
  std::vector<RigidBody> rigid_bodies;
  std::vector<Aabb> aabb;
  // voxel collision shape pools (built by build_voxel_shape, uploaded once at load)
  std::vector<VoxelShape> voxel_shape_cpu;
  std::vector<daxa_u32> voxel_occ_cpu;
  std::vector<daxa_u32> voxel_surf_cpu;
  std::vector<daxa_f32> voxel_sdf_cpu; // node SDF, (dims+1)^3 f32s per shape
  std::vector<std::vector<Aabb>> voxel_shape_prims; // BLAS primitives per shape (body frame)
  std::vector<std::pair<daxa_u32, daxa_u32>> voxel_prim_sites; // per voxel body: (shape idx, first Aabb in `aabb`)
  std::vector<VoxelShapeDerived> voxel_derived_cpu; // CPU mass properties (GPU reduce verify twin)
  // FRACTURE bookkeeping: single-owner flag per shape (load shapes are shared between
  // bodies -> clone-on-first-fracture; fracture-born shapes carve in place) + the host's
  // last-consumed event serial (the GPU ring is never cleared)
  std::vector<bool> shape_private;
  daxa_u32 fracture_serial_seen = 0;

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
