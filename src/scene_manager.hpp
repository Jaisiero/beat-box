#pragma once

#include "defines.hpp"
#include "math.hpp"
#include "fragment_finalization_reference.hpp"
#include "gpu_pool_allocator.inl"
#include "pool_storage.hpp"
#include "free_list_pool.hpp" // FRACTURE memory pools (shared with the AS manager)
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
#include <array>     // fracture: per-component bbox
#include <chrono>    // respawn timing probe

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
    // reserve the surf slice at `bit_count` (total cells), NOT `count` (solid voxels): the
    // fracture path reserves `cells` per shape, so keeping load consistent means every shape
    // frees the same size it reserved (the free-list accounting + retire_shape depend on it).
    u32 const surf_offset = (u32)voxel_surf_cpu.size();
    voxel_surf_cpu.resize(surf_offset + bit_count, 0u);
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
  // Host-only reference inputs for the optional physical-property oracle.
  struct FragmentPublicationReference {
    daxa_u32 body, parent_id;
    daxa_f32vec3 crop_off;
  };
  std::function<void(daxa::BufferId, daxa::BufferId, daxa::BufferId)> voxel_prims_hook(
      std::span<FragmentPublicationReference const> finalizations = {},
      bool preserve_live = false, daxa_u32 live_count = 0u)
  {
    return [this, preserve_live, live_count, inputs = std::vector<FragmentPublicationReference>(finalizations.begin(), finalizations.end())]
        (daxa::BufferId prims_buffer, daxa::BufferId body_buffer, daxa::BufferId instance_buffer) mutable {
      rigid_body_manager->build_voxel_prims_gpu(voxel_shape_cpu, voxel_prim_sites, prims_buffer, aabb,
          preserve_live ? body_buffer : daxa::BufferId{}, instance_buffer, live_count);
      // Mapped final bodies are read only by the explicit verification oracle.
      // Production keeps pose, inertia, shape origins and surface data on GPU.
      bool const verify = bb_getenv("BB_FRAGMENT_VERIFY") != nullptr;
      bool const census_verify = bb_getenv("BB_CENSUS_VERIFY") != nullptr;
      if (!inputs.empty() && (verify || census_verify))
      {
        auto const *bodies = device.buffer_host_address_as<RigidBody>(body_buffer).value();
        auto const *shapes = device.buffer_host_address_as<VoxelShape>(rigid_body_manager->get_voxel_shapes_buffer()).value();
        std::vector<VoxelShapeDerived> derived;
        std::vector<FractureParentContext> contexts;
        if (verify) {
          rigid_body_manager->read_voxel_derived(static_cast<daxa_u32>(voxel_shape_cpu.size()), derived);
          contexts = rigid_body_manager->read_fracture_contexts();
        }
        for (auto const &pc : inputs)
        {
          auto const si = bodies[pc.body].shape_index-1u;
          if (verify) {
            auto const &context = contexts[pc.parent_id];
            auto const &parent = context.parent;
            FragmentFinalizePushConstants reference{
                .voxel_mass = parent.mass/float(parent.primitive_count),
                .com_old = {-context.shape.grid_origin.x,-context.shape.grid_origin.y,-context.shape.grid_origin.z},
                .parent_pos = parent.position, .parent_rot = parent.rotation,
                .parent_vel = parent.velocity, .parent_omega = parent.omega, .crop_off = pc.crop_off};
            auto expected = bodies[pc.body];
            auto shape = shapes[si];
            fragment_finalize_reference(expected,shape,derived[si],reference);
            fragment_verify_finalization(expected,shape,bodies[pc.body],shapes[si],pc.body);
          }
          if (census_verify && voxel_shape_cpu[si].surf_count != shapes[si].surf_count) {
            std::cerr << "[CENSUS-VERIFY] surface count FAILED shape=" << si << std::endl;
            std::abort();
          }
        }
      }
    };
  }

  void push_voxel_body(VoxelShapeBuild const &s, daxa_f32vec3 pos, Quaternion rot, daxa_u32 mat, f32 friction,
                       f32 fracture_impulse = 0.0f, daxa_u32 fracture_material = 0u)
  {
    rigid_bodies.push_back({.flags = (RigidBodyFlag::DYNAMIC | RigidBodyFlag::GRAVITY), .primitive_count = s.primitive_count, .primitive_offset = 0, .shape_index = s.shape_id, .position = pos, .rotation = rot, .minimum = s.minimum, .maximum = s.maximum, .mass = s.mass, .inv_mass = 1.0f / s.mass, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .inv_inertia = s.inv_inertia, .restitution = 0.0f, .friction = friction, .fracture_impulse = fracture_impulse, .fracture_material = fracture_material});
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

  // Interactive demolition: connected voxel frames carry their own geometry as one
  // rigid body until impact fracture separates them. This is geometric connectivity,
  // not a stress/bond solver between independently authored rigid bodies.
  void scene_9() {
    materials = {
      { .albedo = daxa_f32vec3(0.1f, 0.1f, 0.1f),   .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) },  // 0 floor
      { .albedo = daxa_f32vec3(1.0f, 1.0f, 1.0f),   .emission = daxa_f32vec3(18.0f, 18.0f, 18.0f) }, // 1 light
      { .albedo = daxa_f32vec3(0.42f, 0.22f, 0.09f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) },  // 2 dark timber
      { .albedo = daxa_f32vec3(0.65f, 0.40f, 0.18f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) },  // 3 light timber
      { .albedo = daxa_f32vec3(0.55f, 0.57f, 0.60f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) },  // 4 projectile steel
    };

    auto const Q = Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    auto const I0 = daxa_mat3_from_glm_mat3(glm::mat3(0));
    // floor (top at y=0) + wide emissive panel overhead
    rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, -50.0f, 0.0f), .rotation = Q, .minimum = daxa_f32vec3(-50.0f, -50.0f, -50.0f), .maximum = daxa_f32vec3(50.0f, 50.0f, 50.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .inv_inertia = I0, .restitution = 0.0f, .friction = 0.7f});
    auto light = RigidBody{.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, 24.0f, 14.0f), .rotation = Q, .minimum = daxa_f32vec3(-11.0f, -0.2f, -6.0f), .maximum = daxa_f32vec3(11.0f, 0.2f, 6.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .inv_inertia = I0, .restitution = 0.0f, .friction = 0.6f};
    light.material_index = 1u;
    rigid_bodies.push_back(light);

    // Two freestanding timber frames. Posts and crossbars belong to ONE connected
    // voxel shape, so the crossbars do not fall before the first hit.
    auto frame = build_voxel_shape(glm::uvec3(24, 16, 4), 0.25f, 12.0f,
        [](u32 x, u32 y, u32) {
          return x < 3u || x >= 21u || y < 2u || (y >= 7u && y < 9u) || y >= 14u;
        });
    push_voxel_body(frame, daxa_f32vec3(0.0f, 2.001f, 14.0f), Q, 3u, 0.8f, 30.0f, 1u);
    push_voxel_body(frame, daxa_f32vec3(3.0f, 2.001f, 18.0f), Q, 2u, 0.8f, 30.0f, 1u);

    // Reusable, unbreakable steel club: grab its end with LEFT, swing, then release.
    // Its offset anchor produces torque through the mouse joint's effective mass.
    auto club = build_voxel_shape(glm::uvec3(12, 3, 3), 0.25f, 40.0f,
        [](u32, u32, u32) { return true; });
    push_voxel_body(club, daxa_f32vec3(-6.0f, 0.376f, 12.0f), Q, 4u, 0.7f);
    std::cout << "[F9] Left hold: grab/swing; release: throw; right: camera; Space: run; R: rebuild."
              << std::endl;
  }

  // F7 pool geometry with F9 wood fracture strength; static walls never fracture.
  void scene_10() {
    // Reuse F7's exact pool, rain positions and rotations. Static floor/walls
    // remain ordinary OBBs with zero fracture strength.
    scene_7();
    auto originals = std::move(rigid_bodies);
    rigid_bodies.clear();
    rigid_bodies.reserve(originals.size());
    auto cube = build_voxel_shape(glm::uvec3(4,4,4), 0.25f, 5.0f,
        [](u32,u32,u32) { return true; });
    u32 color = 0;
    for (auto const &body : originals)
    {
      if ((body.flags & RigidBodyFlag::DYNAMIC) == RigidBodyFlag::NONE)
        rigid_bodies.push_back(body);
      else
        push_voxel_body(cube, body.position, body.rotation, 2u + color++ % 5u,
                        body.friction, 30.0f, 1u); // F9 wood/impact strength
    }
    std::cout << "[F10] 432 breakable voxel boxes; static unbreakable pool walls."
              << std::endl;
  }

  // FRACTURE SOAK TEST (scene_11): an exhaustive free-list stress. A runtime spawner rains
  // breakable voxel bodies of MANY shapes (different pool footprints -> the free-list must
  // split/coalesce varied sizes) onto anvils; each shatters (Voronoi), the fragments settle,
  // and a high kill plane culls them -> a long, near-capacity alloc/free churn. Run headless
  // with BB_POOL_VERIFY=1 (self-check after every fracture/cull) + BB_KILL_Y (cull height) +
  // BB_FRACTURE_SPAWN_STEPS (spawn cadence). The 5-minute soak is just this scene left running.
  void scene_11() {
    materials = {
      { .albedo = daxa_f32vec3(0.1f, 0.1f, 0.1f),    .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) },
      { .albedo = daxa_f32vec3(1.0f, 1.0f, 1.0f),    .emission = daxa_f32vec3(20.0f, 20.0f, 20.0f) },
      { .albedo = daxa_f32vec3(0.20f, 0.85f, 0.30f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) }, // 2
      { .albedo = daxa_f32vec3(0.95f, 0.75f, 0.10f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) }, // 3
      { .albedo = daxa_f32vec3(0.60f, 0.15f, 0.90f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) }, // 4
      { .albedo = daxa_f32vec3(0.90f, 0.35f, 0.15f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) }, // 5
      { .albedo = daxa_f32vec3(0.30f, 0.08f, 0.06f), .emission = daxa_f32vec3(0.0f, 0.0f, 0.0f) }, // 6 anvil
    };
    auto const Q = Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    auto const I0 = daxa_mat3_from_glm_mat3(glm::mat3(0));
    rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, -50.0f, 0.0f), .rotation = Q, .minimum = daxa_f32vec3(-50.0f, -50.0f, -50.0f), .maximum = daxa_f32vec3(50.0f, 50.0f, 50.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .inv_inertia = I0, .restitution = 0.0f, .friction = 0.7f});
    auto light = RigidBody{.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(0.0f, 26.0f, 14.0f), .rotation = Q, .minimum = daxa_f32vec3(-14.0f, -0.2f, -8.0f), .maximum = daxa_f32vec3(14.0f, 0.2f, 8.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .inv_inertia = I0, .restitution = 0.0f, .friction = 0.6f};
    light.material_index = 1u;
    rigid_bodies.push_back(light);

    f32 const vs = 0.5f, den = 2.0f;
    // five source shapes with DELIBERATELY different pool footprints (nodes/words/cells) so
    // the free-list sees a wide spread of allocation sizes:
    spawn_shapes_.clear(); spawn_strengths_.clear(); spawn_mats_.clear();
    auto add_shape = [&](VoxelShapeBuild b, f32 strength, daxa_u32 mat) {
      spawn_shapes_.push_back(b); spawn_strengths_.push_back(strength); spawn_mats_.push_back(mat);
    };
    add_shape(build_voxel_shape(glm::uvec3(3, 3, 3), vs, den, [](u32,u32,u32){ return true; }), 25.0f, 2u); // cube
    add_shape(build_voxel_shape(glm::uvec3(6, 6, 2), vs, den, [](u32 x,u32 y,u32){ return y<2||x<2; }), 40.0f, 3u); // L
    add_shape(build_voxel_shape(glm::uvec3(8, 8, 2), vs, den, [](u32 x,u32 y,u32){ return !(x>=2&&x<6&&y>=2&&y<6); }), 55.0f, 4u); // frame
    add_shape(build_voxel_shape(glm::uvec3(10, 4, 4), vs, den, [](u32,u32,u32){ return true; }), 70.0f, 5u); // block (wood grain=x)
    add_shape(build_voxel_shape(glm::uvec3(12, 2, 3), vs, den, [](u32,u32,u32){ return true; }), 45.0f, 3u); // slab

    // a row of anvils to land on (concentrate the impact -> reliable fracture)
    for (f32 ax : {-9.0f, -3.0f, 3.0f, 9.0f})
    {
      rigid_bodies.push_back({.flags = RigidBodyFlag::NONE, .primitive_count = 1, .primitive_offset = 0, .position = daxa_f32vec3(ax, 0.9f, 14.0f), .rotation = Q, .minimum = daxa_f32vec3(-0.8f, -0.9f, -1.0f), .maximum = daxa_f32vec3(0.8f, 0.9f, 1.0f), .mass = 0.0f, .inv_mass = 0.0f, .velocity = daxa_f32vec3(0, 0, 0), .omega = daxa_f32vec3(0, 0, 0), .inv_inertia = I0, .restitution = 0.0f, .friction = 0.7f});
      rigid_bodies.back().material_index = 6u;
    }
    spawner_on_ = true;
    spawn_step_ = 0;
  }

  // Immutable scene templates are uploaded once. Runtime selection, random state,
  // body slots and kinematics belong to the GPU scene-edit controller.
  RigidBody spawn_template(daxa_u32 si) const
  {
    auto const &shape=spawn_shapes_[si];
    return {.flags=RigidBodyFlag::DYNAMIC | RigidBodyFlag::GRAVITY,
        .material_index=spawn_mats_[si],.primitive_count=shape.primitive_count,
        .shape_index=shape.shape_id,.rotation=Quaternion(0,0,0,1),
        .minimum=shape.minimum,.maximum=shape.maximum,.mass=shape.mass,.inv_mass=1.0f/shape.mass,
        .inv_inertia=shape.inv_inertia,.friction=0.6f,
        .fracture_impulse=spawn_strengths_[si],.fracture_material=si==3u ? 1u : 0u};
  }
  static Quaternion Q_id() { return Quaternion(0.0f, 0.0f, 0.0f, 1.0f); }

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
    VoxelShapeBuild vox_l{}, vox_cross{}, vox_frame{}, vox_timber{};
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
        if (shape == "timber" && vox_timber.shape_id == 0u)
          vox_timber = build_voxel_shape(glm::uvec3(24,16,4), 0.25f, 12.0f,
              [](u32 x,u32 y,u32) { return x<3u || x>=21u || y<2u || (y>=7u && y<9u) || y>=14u; });
        VoxelShapeBuild const *vsb = shape == "l" ? &vox_l : shape == "cross" ? &vox_cross
                                   : shape == "frame" ? &vox_frame : shape == "timber" ? &vox_timber : nullptr;
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
      // optional rotation (F12 live dumps append it): px py pz h m e fr qx qy qz qw
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

  // F12: dump the LIVE GPU poses (current-parity rigid body buffer) in BB_SCENE_FILE format.
  // Voxel bodies dump as `vox <name> pos quat` (shape_index 1/2/3 = l/cross/frame, the same
  // order scene_5 and the file loader build them); cubes dump with their rotation appended.
  // This is the capture half of the repro loop: see the bad configuration -> F12 -> load the
  // file headless with BB_SCENE_FILE and debug the exact state. CPU-side by design (debug
  // tooling; the sim itself stays GPU-resident).
  // On-demand forensic capture. Unlike the legacy scene text, this preserves actual
  // fragment occupancy and the solver's stored contact points. Never runs per frame.
  void dump_contact_geometry(std::string const &path)
  {
    auto const &sc = rigid_body_manager->get_sim_config_reference();
    u32 const nb = sc.rigid_body_count;
    u32 const nm = std::min(sc.g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
    if (nb == 0u) return;
    bool const avbd = sc.solver_type == SimSolverType::AVBD;
    // Finish both queues before copying simulation buffers or reading host pools.
    device.wait_idle();
    auto read = [&]<typename T>(daxa::BufferId source, u32 count) {
      std::vector<T> result;
      if (count == 0u) return result;
      auto const bytes = daxa::usize(count) * sizeof(T);
      auto staging = device.create_buffer({.size = bytes, .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM, .name = "contact_capture"});
      auto rec = device.create_command_recorder({.queue_type = daxa::QueueType::COMPUTE});
      rec.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE, .dst_access = daxa::AccessConsts::TRANSFER_READ});
      rec.copy_buffer_to_buffer({.src_buffer = source, .dst_buffer = staging, .size = bytes});
      rec.pipeline_barrier({.src_access = daxa::AccessConsts::TRANSFER_WRITE, .dst_access = daxa::AccessConsts::HOST_READ});
      auto cmds = rec.complete_current_commands();
      device.submit_commands({.queue = daxa::QUEUE_COMPUTE_0, .command_lists = std::array{cmds}});
      device.wait_on_submit({.queue = daxa::QUEUE_COMPUTE_0, .queue_submit_index = device.latest_queue_submit_index(daxa::QUEUE_COMPUTE_0)});
      auto const *data = device.buffer_host_address_as<T>(staging).value();
      result.assign(data, data + count);
      device.destroy_buffer(staging);
      return result;
    };
    auto bodies = read.template operator()<RigidBody>(rigid_body_manager->task_rigid_bodies.id(), nb);
    auto contacts = read.template operator()<Manifold>(rigid_body_manager->task_collisions.id(), nm);
    auto states = read.template operator()<AvbdBodyState>(rigid_body_manager->task_avbd_state.id(), avbd ? nb : 0u);
    auto gpu_shapes = read.template operator()<VoxelShape>(rigid_body_manager->get_voxel_shapes_buffer(), u32(voxel_shape_cpu.size()));
    auto gpu_surface = read.template operator()<daxa_u32>(rigid_body_manager->get_voxel_surface_buffer(), BB_MAX_VOXEL_SURF_COUNT);
    std::ofstream out(path, std::ios::trunc);
    if (!out) { std::cerr << "Contact capture: cannot open " << path << std::endl; return; }
    out.precision(9);
    auto v = [&](auto const &p) { out << '[' << p.x << ',' << p.y << ',' << p.z << ']'; };
    auto q = [&](auto const &r) { out << '[' << r.v.x << ',' << r.v.y << ',' << r.v.z << ',' << r.w << ']'; };
    out << "{\"version\":1,\"solver\":" << u32(sc.solver_type) << ",\"frame\":" << sc.frame_count << ",\"bodies\":[";
    for (u32 i = 0; i < nb; ++i) {
      auto const &b = bodies[i];
      if (i) out << ',';
      out << "{\"id\":" << b.id << ",\"shape\":" << b.shape_index << ",\"flags\":" << u32(b.flags) << ",\"position\":"; v(b.position);
      out << ",\"rotation\":"; q(b.rotation);
      out << ",\"velocity\":"; v(b.velocity);
      out << ",\"omega\":"; v(b.omega);
      out << ",\"minimum\":"; v(b.minimum); out << ",\"maximum\":"; v(b.maximum);
      bool const awake = (b.flags & RigidBodyFlag::DYNAMIC) != RigidBodyFlag::NONE && (b.flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE;
      out << ",\"start_valid\":" << (avbd && awake ? "true" : "false");
      if (avbd && awake) { out << ",\"start_position\":"; v(states[i].pos_start); out << ",\"start_rotation\":"; q(states[i].rot_start); }
      out << '}';
    }
    auto gpu_occupancy = read.template operator()<daxa_u32>(rigid_body_manager->get_voxel_occupancy_buffer(), BB_MAX_VOXEL_OCC_U32S);
    out << "],\"shapes\":[";
    for (u32 i = 0; i < voxel_shape_cpu.size(); ++i) {
      auto sh = gpu_shapes[i]; if (i) out << ',';
      if (voxel_shape_cpu[i].dims.x==0u) { sh.dims={0,0,0};sh.surf_count=0; }
      out << "{\"dims\":"; v(sh.dims); out << ",\"origin\":"; v(sh.grid_origin);
      out << ",\"voxel_size\":" << sh.voxel_size << ",\"occupancy\":[";
      u32 const words = (sh.dims.x * sh.dims.y * sh.dims.z + 31u) / 32u;
      for (u32 j = 0; j < words; ++j) { if (j) out << ','; out << gpu_occupancy[sh.occ_offset+j]; }
      out << "],\"surface_count\":" << sh.surf_count << ",\"gpu_surface_count\":" << gpu_shapes[i].surf_count << ",\"surface\":[";
      for (u32 j = 0; j < sh.surf_count && sh.surf_offset + j < gpu_surface.size(); ++j) {
        if (j) out << ',';
        out << gpu_surface[gpu_shapes[i].surf_offset+j];
      }
      out << "]}";
    }
    out << "],\"manifolds\":[";
    for (u32 i = 0; i < nm; ++i) {
      auto const &m = contacts[i]; if (i) out << ',';
      out << "{\"a\":" << m.obb1_index << ",\"b\":" << m.obb2_index << ",\"key\":" << m.key << ",\"normal\":"; v(m.normal);
      out << ",\"contacts\":[";
      for (u32 j = 0; j < std::min(u32(std::max(m.contact_count,0)), MAX_CONTACT_POINT_COUNT); ++j) {
        auto const &c = m.contacts[j]; if (j) out << ',';
        out << "{\"position\":"; v(c.position); out << ",\"penetration\":" << c.penetration << ",\"features\":[" << c.fp.in_reference << ',' << c.fp.out_reference << ',' << c.fp.in_incident << ',' << c.fp.out_incident << "]}";
      }
      out << "]}";
    }
    out << "]}\n";
    std::cout << "[CONTACT-CAPTURE] " << path << " bodies=" << nb << " manifolds=" << nm << std::endl;
  }

  void dump_scene_live(std::string const &path)
  {
    daxa_u32 const count = rigid_body_count;
    if (count == 0u) { std::cerr << "F12 dump: no bodies" << std::endl; return; }
    daxa::BufferId src = accel_struct_mngr->get_rigid_body_buffer();
    if (src.is_empty()) { std::cerr << "F12 dump: rigid body buffer not ready" << std::endl; return; }
    auto const size = static_cast<daxa::usize>(count) * sizeof(RigidBody);
    daxa::BufferId staging = device.create_buffer({
        .size = size,
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "scene_dump_staging",
    });
    {
      device.wait_idle(); // scene edits and host readback require a completed publication
      auto rec = device.create_command_recorder({.queue_type = daxa::QueueType::COMPUTE});
      rec.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE,
                            .dst_access = daxa::AccessConsts::TRANSFER_READ});
      rec.copy_buffer_to_buffer({.src_buffer = src, .dst_buffer = staging, .size = size});
      rec.pipeline_barrier({.src_access = daxa::AccessConsts::TRANSFER_WRITE,
                            .dst_access = daxa::AccessConsts::HOST_READ});
      auto cmds = rec.complete_current_commands();
      device.submit_commands({.queue = daxa::QUEUE_COMPUTE_0, .command_lists = std::array{cmds}});
      device.wait_idle();
    }
    RigidBody const *live = device.buffer_host_address_as<RigidBody>(staging).value();
    std::ofstream out(path, std::ios::trunc);
    if (!out)
    {
      std::cerr << "F12 dump: cannot open '" << path << "'" << std::endl;
      device.destroy_buffer(staging);
      return;
    }
    out.precision(9); // round-trip float32 poses for independent geometry checks
    out << "# F12 live dump: px py pz [h m e fr qx qy qz qw] / vox <shape> px py pz qx qy qz qw\n";
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
    dump_contact_geometry(path + ".contacts.json");
    std::cout << "[SCENE] F12 live dump wrote " << n << " bodies to '" << path << "'" << std::endl;
  }

  // Runtime fracture orchestration: GPU owns topology, allocation and physical
  // bodies. The host mirrors compact child metadata to record AS builds.
  // Reference inputs below are used only by explicit verification.
  struct FragFix
  {
    daxa_u32 body, parent_id;
    glm::vec3 crop_off{0.0f};
  };

  // sync the pool high-water marks to the post-load vector sizes ONCE (the load path is a
  // plain bump append; the allocators take over only for the runtime fracture path). Done
  // lazily on the first fracture so it captures every load/switch_scene rebuild.
  void sync_pools_if_needed()
  {
    if (pools_synced_) { return; }
    occ_pool_.reset();  occ_pool_.high_water = (daxa_u32)voxel_occ_cpu.size();  occ_pool_.live_bytes = occ_pool_.high_water;
    surf_pool_.reset(); surf_pool_.high_water = (daxa_u32)voxel_surf_cpu.size(); surf_pool_.live_bytes = surf_pool_.high_water;
    sdf_pool_.reset();  sdf_pool_.high_water = (daxa_u32)voxel_sdf_cpu.size();  sdf_pool_.live_bytes = sdf_pool_.high_water;
    free_shape_slots_.clear();
    std::vector<RigidBody> spawn_templates;
    for (daxa_u32 i=0u;i<spawn_shapes_.size();++i) spawn_templates.push_back(spawn_template(i));
    rigid_body_manager->initialize_fracture_allocator({occ_pool_.high_water,sdf_pool_.high_water,
        surf_pool_.high_water,0u,static_cast<daxa_u32>(voxel_shape_cpu.size()),static_cast<daxa_u32>(rigid_bodies.size())},spawn_templates);
    pools_synced_ = true;
  }

  // FULL POOL VERIFICATION (BB_POOL_VERIFY): the per-pool self-check (coalescing + byte
  // accounting) PLUS a cross-check that every LIVE shape's slice is in bounds and mutually
  // DISJOINT and disjoint from every free hole - the worst bug (two shapes aliasing memory)
  // lives here. Called after each fracture/cull; aborts loudly at the first violation so a
  // corruption is caught at the exact operation instead of surfacing as garbage geometry.
  void verify_pools(char const *where)
  {
    auto fail = [&](std::string const &msg) {
      std::cerr << "[POOL-VERIFY] FAIL @ " << where << ": " << msg << std::endl;
      std::abort();
    };
    for (auto const &pr : {std::pair<FreeListPool *, char const *>{&occ_pool_, "occ"},
                           {&surf_pool_, "surf"}, {&sdf_pool_, "sdf"}})
    {
      std::string const e = pr.first->verify_free();
      if (!e.empty()) { fail(std::string(pr.second) + ": " + e); }
    }
    // gather each pool's live intervals from the live shapes (skip dims=0 dead slots)
    std::vector<std::pair<daxa_u32, daxa_u32>> occ_iv, surf_iv, sdf_iv;
    for (daxa_u32 si = 0u; si < voxel_shape_cpu.size(); ++si)
    {
      VoxelShape const &s = voxel_shape_cpu[si];
      if (s.dims.x == 0u) { continue; }
      daxa_u32 const cells = s.dims.x * s.dims.y * s.dims.z;
      occ_iv.emplace_back(s.occ_offset, (cells + 31u) / 32u);
      surf_iv.emplace_back(s.surf_offset, cells); // fracture shapes reserved `cells` for surf
      sdf_iv.emplace_back(s.sdf_offset, (s.dims.x + 1u) * (s.dims.y + 1u) * (s.dims.z + 1u));
    }
    auto check = [&](std::vector<std::pair<daxa_u32, daxa_u32>> &iv, FreeListPool &p, char const *name) {
      std::sort(iv.begin(), iv.end());
      daxa_u32 live_sum = 0u;
      for (size_t i = 0; i < iv.size(); ++i)
      {
        if (iv[i].first + iv[i].second > p.high_water) { fail(std::string(name) + ": live slice past high_water at " + std::to_string(iv[i].first)); }
        if (i > 0 && iv[i].first < iv[i - 1].first + iv[i - 1].second) { fail(std::string(name) + ": OVERLAPPING live slices near " + std::to_string(iv[i].first)); }
        // a live slice must not intersect any free hole
        auto it = p.free_ranges.lower_bound(iv[i].first);
        if (it != p.free_ranges.begin()) { auto pv = std::prev(it); if (pv->first + pv->second > iv[i].first) { fail(std::string(name) + ": live slice intersects a free hole at " + std::to_string(iv[i].first)); } }
        if (it != p.free_ranges.end() && it->first < iv[i].first + iv[i].second) { fail(std::string(name) + ": live slice intersects a free hole at " + std::to_string(iv[i].first)); }
        live_sum += iv[i].second;
      }
      if (live_sum != p.live_bytes) { fail(std::string(name) + ": live-slice sum " + std::to_string(live_sum) + " != pool live_bytes " + std::to_string(p.live_bytes)); }
    };
    auto gpu_pools=rigid_body_manager->read_fracture_pools();
    std::array<FreeListPool*,3> cpu_pools{&occ_pool_,&sdf_pool_,&surf_pool_};
    for (size_t i=0;i<cpu_pools.size();++i) {
      auto const &cpu=*cpu_pools[i];auto &gpu=gpu_pools[i];
      if (!gpu.valid() || cpu.high_water!=gpu.high_water || cpu.live_bytes!=gpu.live_units || cpu.free_ranges.size()!=gpu.range_count)
        fail("GPU allocator accounting mismatch");
      size_t j=0;for (auto const &[offset,size]:cpu.free_ranges) {
        if (gpu.ranges[j].offset!=offset || gpu.ranges[j].size!=size) fail("GPU allocator range mismatch");
        ++j;
      }
    }
    auto check_slots=[&](GpuFreeList &pool,size_t count,auto const &free_slots) {
      if (!pool.valid() || pool.live_units!=count-free_slots.size()) fail("GPU slot accounting mismatch");
      for (daxa_u32 slot=0u;slot<count;++slot) {
        bool live=slot<pool.high_water;
        for (daxa_u32 r=0u;r<pool.range_count;++r)
          if (slot>=pool.ranges[r].offset && slot<pool.ranges[r].offset+pool.ranges[r].size) live=false;
        bool expected=std::find(free_slots.begin(),free_slots.end(),slot)==free_slots.end();
        if (live!=expected) fail("GPU slot ownership mismatch");
      }
    };
    check_slots(gpu_pools[4],voxel_shape_cpu.size(),free_shape_slots_);
    check_slots(gpu_pools[5],rigid_bodies.size(),free_body_slots_);
    check(occ_iv, occ_pool_, "occ");
    check(surf_iv, surf_pool_, "surf");
    check(sdf_iv, sdf_pool_, "sdf");
  }
  bool pool_verify_on()
  {
    static bool const on = bb_getenv("BB_POOL_VERIFY") != nullptr;
    return on;
  }

  // free a shape's byte-pool slices + its index back to the free lists (the slot's vectors
  // are left in place; a future alloc_shape_slot reuses the index and overwrites them).
  void retire_shape(daxa_u32 shape_i)
  {
    if (shape_i >= voxel_shape_cpu.size()) { return; }
    VoxelShape const &s = voxel_shape_cpu[shape_i];
    daxa_u32 const cells = s.dims.x * s.dims.y * s.dims.z;
    daxa_u32 const words = (cells + 31u) / 32u;
    daxa_u32 const nodes = (s.dims.x + 1u) * (s.dims.y + 1u) * (s.dims.z + 1u);
    if (pool_verify_on()) {
      occ_pool_.free(s.occ_offset, words);
      sdf_pool_.free(s.sdf_offset, nodes);
      surf_pool_.free(s.surf_offset, cells);
    }
    // sentinel: a freed slot's stale record must NOT be re-processed by the GPU pool rebuild
    // (its offsets may already belong to a reused slot -> corruption). dims=0 marks it dead;
    // build_voxel_pools_gpu skips it, and GPU admission overwrites its descriptor on reuse.
    voxel_shape_cpu[shape_i].dims = daxa_u32vec3(0, 0, 0);
    free_shape_slots_.push_back(shape_i);
    voxel_shape_prims[shape_i].clear();
  }

  // Retire a body (index == persistent id): free its voxel shape back to the pools and
  // TOMBSTONE the row - an inert static box parked far below the world, its index pushed to
  // free_body_slots_ for reuse by the next fragment. The row stays in place (contiguity +
  // the id==index invariant the fracture path relies on); respawn rebuilds the active set
  // from scratch, so a tombstone (flags NONE) is simply excluded. The transient BLAS a
  // tombstone still carries is reclaimed only in phase 2 (incremental AS); phase 1 reclaims
  // the SHAPE pools (the resource that actually exhausts under heavy shattering).
  void retire_body(daxa_u32 index)
  {
    if (index >= rigid_bodies.size()) { return; }
    RigidBody &b = rigid_bodies[index];
    if (b.shape_index != 0u)
    {
      // ONLY free single-owner shapes. A body may still reference a SHARED template (a load
      // shape, e.g. a spawner source, before it ever fractured) - freeing that would corrupt
      // every sibling + future spawn that uses it. Clones/fragments are shape_private=true.
      daxa_u32 const si = b.shape_index - 1u;
      if (si < shape_private.size() && shape_private[si]) { retire_shape(si); }
    }
    b.flags = RigidBodyFlag::NONE;
    b.shape_index = 0u;
    b.primitive_count = 1u;
    b.primitive_offset = 0u;
    b.mass = 0.0f;
    b.inv_mass = 0.0f;
    b.velocity = daxa_f32vec3(0, 0, 0);
    b.omega = daxa_f32vec3(0, 0, 0);
    b.position = daxa_f32vec3(0.0f, -1000.0f, 0.0f);
    b.minimum = daxa_f32vec3(-0.001f, -0.001f, -0.001f);
    b.maximum = daxa_f32vec3(0.001f, 0.001f, 0.001f);
    b.material_index = 0u;
    b.island_index = MAX_U32;
    b.manifold_node_index = MAX_U32;
    b.active_index = MAX_U32;
    b.sleep_timer = 0u;
    free_body_slots_.push_back(index);
  }

  // rebuild the host active-body bookkeeping (id set + counts) from the live rigid_bodies
  // vector - called by respawn so retirement/spawn never touch the active list incrementally
  // (which had id/key-collision hazards). Tombstones and statics (flags without DYNAMIC) are
  // naturally excluded. rigid_body_map is a set of active ids (its keys are unused).
  void rebuild_active_bookkeeping()
  {
    rigid_body_count = (daxa_u32)rigid_bodies.size();
    rigid_body_map.clear();
    rigid_body_active_count = 0u;
    for (auto const &b : rigid_bodies)
    {
      if ((b.flags & RigidBodyFlag::DYNAMIC) != RigidBodyFlag::NONE)
      {
        rigid_body_map[rigid_body_active_count++] = b.id;
      }
    }
  }

  // Debug-only independent reference for newly cropped shapes. Runtime packing
  // reserves these slices but deliberately does not author their derived data.
  void refresh_fracture_sdf_reference(daxa_u32 si)
  {
    auto const &sh=voxel_shape_cpu[si];
    auto d=sh.dims;
    auto solid=[&](int x,int y,int z) {
      if (x<0 || y<0 || z<0 || x>=int(d.x) || y>=int(d.y) || z>=int(d.z)) return false;
      u32 bit=u32(x)+u32(y)*d.x+u32(z)*d.x*d.y;
      return (voxel_occ_cpu[sh.occ_offset+bit/32u] & (1u<<(bit%32u)))!=0u;
    };
    std::vector<glm::vec3> occupied,empty;
    glm::dvec3 sum(0.0);
    u32 surface_count=0u;
    int const neighbors[6][3]={{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
    for (u32 z=0;z<d.z;++z) for (u32 y=0;y<d.y;++y) for (u32 x=0;x<d.x;++x) {
      glm::vec3 cell(x,y,z);
      if (!solid(x,y,z)) { empty.push_back(cell);continue; }
      occupied.push_back(cell);sum+=glm::dvec3(cell)+glm::dvec3(0.5);
      for (u32 n=0;n<6u;++n)
        if (!solid(int(x)+neighbors[n][0],int(y)+neighbors[n][1],int(z)+neighbors[n][2])) {
          voxel_surf_cpu[sh.surf_offset+surface_count++]=x|(y<<8u)|(z<<16u)|(n<<24u);
          break;
        }
    }
    if (surface_count!=sh.surf_count || occupied.empty()) {
      std::cerr << "[SDF-REFERENCE] invalid cropped occupancy" << std::endl;std::abort();
    }
    glm::vec3 com(sum/double(occupied.size())*double(sh.voxel_size));
    glm::mat3 inertia(0.0f);
    for (auto cell:occupied) {
      auto c=(cell+glm::vec3(0.5f))*sh.voxel_size-com;
      inertia+=glm::mat3(glm::dot(c,c))-glm::outerProduct(c,c);
      inertia+=glm::mat3(sh.voxel_size*sh.voxel_size/6.0f);
    }
    voxel_derived_cpu[si]={.count=u32(occupied.size()),
      .com=daxa_f32vec3(com.x,com.y,com.z),.unit_inertia=daxa_mat3_from_glm_mat3(inertia)};
    auto distance=[](glm::vec3 p,glm::vec3 cell) {
      return glm::length(glm::max(glm::max(cell-p,p-cell-glm::vec3(1.0f)),glm::vec3(0.0f)));
    };
    for (u32 z=0;z<=d.z;++z) for (u32 y=0;y<=d.y;++y) for (u32 x=0;x<=d.x;++x) {
      glm::vec3 p(x,y,z);
      float ds=1e30f,de=float(std::min({x,y,z,d.x-x,d.y-y,d.z-z}));
      for (auto cell:occupied) ds=std::min(ds,distance(p,cell));
      for (auto cell:empty) de=std::min(de,distance(p,cell));
      voxel_sdf_cpu[sh.sdf_offset+x+y*(d.x+1u)+z*(d.x+1u)*(d.y+1u)]=(ds>0.0f ? ds : -de)*sh.voxel_size;
    }
  }

  // One event: read-only GPU partition + component labels, then private fragment grids.
  bool apply_fracture(FractureEventSummary const &ev, std::vector<FragFix> &fixes,
                      bool partitioned = false, std::span<FractureBatchChild const> batch = {})
  {
    if (ev.body_id >= rigid_bodies.size()) { return false; }
    auto &body = rigid_bodies[ev.body_id];
    if (body.shape_index == 0u || (body.flags & RigidBodyFlag::DYNAMIC) == RigidBodyFlag::NONE) { return false; }
    daxa_u32 shape_i = body.shape_index - 1u;
    daxa_u32vec3 const dims = voxel_shape_cpu[shape_i].dims;
    daxa_u32 const cells = dims.x * dims.y * dims.z;
    // Partitioning uses carve radius zero: occupancy is read-only. Keep a shared
    // source shape intact and allocate private grids only after a real split exists.
    VoxelShape const shape = voxel_shape_cpu[shape_i]; // stable copy for this event

    f32 const vs = shape.voxel_size;
    daxa_u32 const old_count = body.primitive_count;
    if (old_count == 0u) return false;
    FragFix const parent_ctx{ev.body_id, ev.body_id};
    std::vector<daxa_u32> labels;
    std::vector<FragmentComponent> components;
    std::vector<FractureChildAllocation> allocations;
    if (partitioned) {
      for (auto const &child : batch) {
        components.push_back(child.component);
        allocations.push_back(child.allocation);
      }
    } else {
      rigid_body_manager->carve_and_label(shape, ev.body_id, labels, components, allocations);
    }

    // GPU already merged slivers and sorted children by count/label. This is
    // build metadata only: no host topology decision or label-map upload.
    if (components.size()<=1u) return false;
    std::vector<std::pair<daxa_u32,daxa_u32>> comps;
    std::map<daxa_u32,std::array<daxa_u32,6>> bbox;
    daxa_u32 solid_count=0u;
    for (auto const &c:components)
    {
      comps.emplace_back(c.label,c.count);
      bbox[c.label]={c.lo_x,c.lo_y,c.lo_z,c.hi_x,c.hi_y,c.hi_z};
      solid_count+=c.count;
    }
    if (solid_count!=old_count)
    {
      std::cerr << "FRACTURE: invalid GPU plan; parent retained" << std::endl;
      return false;
    }
    // Reservations and occupancy are already complete on the GPU. The host
    // mirrors compact build metadata for Vulkan AS command recording.
    size_t allocation_index=0u;
    // Component bounding boxes already include the conservation merge above.
    daxa_u32 const DX = dims.x, DY = dims.y;
    bool const verify_surface = bb_getenv("BB_CENSUS_VERIFY") != nullptr || bb_getenv("BB_SDF_VERIFY") != nullptr;

    // allocate a cropped shape for `label` (count solid voxels). Returns the shape index and
    // fills crop_off (bbox_min * vs), or MAX_U32 if the pools can't fit it.
    auto emit_cropped = [&](daxa_u32 label, daxa_u32 count, glm::vec3 &crop_off) -> daxa_u32 {
      auto const &b = bbox[label];
      daxa_u32vec3 const cd(b[3] - b[0] + 1u, b[4] - b[1] + 1u, b[5] - b[2] + 1u);
      daxa_u32 const ccells = cd.x * cd.y * cd.z, cwords = (ccells + 31u) / 32u,
                     cnodes = (cd.x + 1u) * (cd.y + 1u) * (cd.z + 1u);
      auto const &allocation=allocations.at(allocation_index++);
      VoxelShape ns=shape;ns.dims=cd;ns.surf_count=0u;
      ns.occ_offset=allocation.offsets[0];ns.sdf_offset=allocation.offsets[1];ns.surf_offset=allocation.offsets[2];
      // The CPU free lists are verification mirrors, never admission authority.
      auto mirror=[&](FreeListPool &pool,daxa_u32 size,daxa_u32 cap,daxa_u32 expected) {
        bool ok=false;auto offset=pool.alloc(size,cap,ok);
        if (!ok || offset!=expected) { std::cerr << "GPU allocator mirror FAILED" << std::endl;std::abort(); }
      };
      if (pool_verify_on()) {
        mirror(occ_pool_,cwords,BB_MAX_VOXEL_OCC_U32S,ns.occ_offset);
        mirror(sdf_pool_,cnodes,BB_MAX_VOXEL_SDF_F32S,ns.sdf_offset);
        mirror(surf_pool_,ccells,BB_MAX_VOXEL_SURF_COUNT,ns.surf_offset);
      }
      if (verify_surface) {
        prepare_pool_storage(voxel_occ_cpu,ns.occ_offset,cwords,0u);
        prepare_pool_storage(voxel_sdf_cpu,ns.sdf_offset,cnodes,0.0f);
        prepare_pool_storage(voxel_surf_cpu,ns.surf_offset,ccells,0u);
      }
      // re-pack: parent cell (x,y,z) with this label -> cropped cell (x-lo, y-lo, z-lo)
      for (daxa_u32 c = 0u; verify_surface && c < labels.size(); ++c)
      {
        if (labels[c] != label) { continue; }
        daxa_u32 const x = c % DX, y = (c / DX) % DY, z = c / (DX * DY);
        daxa_u32 const cc = (x - b[0]) + (y - b[1]) * cd.x + (z - b[2]) * cd.x * cd.y;
        voxel_occ_cpu[ns.occ_offset + cc / 32u] |= 1u << (cc % 32u);
        if (verify_surface && (x == 0u || x + 1u == dims.x || y == 0u || y + 1u == dims.y || z == 0u || z + 1u == dims.z ||
            labels[c - 1u] != label || labels[c + 1u] != label ||
            labels[c - DX] != label || labels[c + DX] != label ||
            labels[c - DX * DY] != label || labels[c + DX * DY] != label))
          ++ns.surf_count;
      }
      daxa_u32 const nsi=allocation.offsets[4];
      std::erase(free_shape_slots_,nsi);
      if (nsi>=voxel_shape_cpu.size()) {
        voxel_shape_cpu.resize(nsi+1u);voxel_shape_prims.resize(nsi+1u);
        shape_private.resize(nsi+1u);voxel_derived_cpu.resize(nsi+1u);
      }
      voxel_shape_cpu[nsi] = ns;
      if (bb_getenv("BB_SDF_VERIFY")) refresh_fracture_sdf_reference(nsi);
      shape_private[nsi] = true;
      voxel_shape_prims[nsi].clear(); // Runtime primitives are generated exclusively on the GPU.
      crop_off = glm::vec3((f32)b[0] * vs, (f32)b[1] * vs, (f32)b[2] * vs);
      return nsi;
    };

    // largest component: the BODY keeps living, but on a fresh cropped shape (the old
    // parent slice is freed at the end). Admission already reserved room for all
    // components before any body or shape metadata changed.
    glm::vec3 crop0;
    daxa_u32 const shape0 = emit_cropped(comps[0].first, comps[0].second, crop0);
    if (shape0 == MAX_U32) { std::cerr << "FRACTURE: reservation invariant failed" << std::endl; std::abort(); }
    body.shape_index = shape0 + 1u;
    body.primitive_count = comps[0].second;
    { FragFix ff = parent_ctx; ff.body = ev.body_id; ff.crop_off = crop0; fixes.push_back(ff); }

    // the rest become new fragment bodies on their own cropped shapes. Every component here is a
    // kept target (the conservation merge folded all sub-MIN_FRAG slivers into their neighbours),
    // so nothing is skipped: total voxel count out == total in.
    RigidBody const fragment_template = body; // push_back may invalidate the parent reference
    for (size_t k = 1; k < comps.size(); ++k)
    {
      glm::vec3 cropk;
      daxa_u32 const fsi = emit_cropped(comps[k].first, comps[k].second, cropk);
      if (fsi == MAX_U32) { std::cerr << "FRACTURE: reservation invariant failed" << std::endl; std::abort(); }

      RigidBody frag = fragment_template; // stable snapshot across vector growth
      frag.shape_index = fsi + 1u;
      frag.primitive_count = comps[k].second;
      frag.primitive_offset = 0u;
      frag.island_index = MAX_U32;
      frag.manifold_node_index = MAX_U32;
      frag.active_index = MAX_U32;
      frag.sleep_timer = 0u;
      frag.flags = RigidBodyFlag::DYNAMIC | RigidBodyFlag::GRAVITY;
      // Mass/inertia/position/velocity are provisional until GPU finalization.
      // REUSE a retired (tombstoned) body slot when one exists, else append. id == index in
      // both paths; the active bookkeeping is rebuilt wholesale by respawn.
      daxa_u32 const slot=allocations[k].offsets[5];
      std::erase(free_body_slots_,slot);frag.id=slot;
      if (slot<rigid_bodies.size()) rigid_bodies[slot]=frag;
      else if (slot==rigid_bodies.size()) rigid_bodies.push_back(frag);
      else { std::cerr << "GPU body slot gap FAILED" << std::endl;std::abort(); }
      FragFix ff = parent_ctx;
      ff.body = slot;         // FragFix.body indexes rigid_bodies (== id)
      ff.crop_off = cropk;
      fixes.push_back(ff);
    }
    // Free a private parent only after emitting all fragments. Shared source shapes
    // remain resident for their other bodies and scene spawners.
    if (shape_private[shape_i]) retire_shape(shape_i); // shared source shapes still serve siblings
    return true;
  }

  // Publish GPU-owned geometry and refreshed body/AS metadata.
  void respawn_after_fracture(std::vector<FragFix> const &fixes, std::span<daxa_u32 const> extra_changed = {})
  {
    daxa_u32 const live_count=rigid_body_count;
    // BB_RESPAWN_TIMING: coarse per-phase respawn timing (env-gated tooling for the
    // incremental-AS work; zero cost when off).
    static bool const _t = bb_getenv("BB_RESPAWN_TIMING") != nullptr;
    auto _now = [] { return std::chrono::high_resolution_clock::now(); };
    auto _ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
    auto _t0 = _now();
    // 2. GPU rebuild chain (SDF + surface + inertia), INCREMENTAL: only the shapes created
    // this batch (the fixed bodies' shapes) changed geometry; the rest are already correct on
    // the GPU. A cull-only respawn (empty fixes) rebuilds nothing.
    std::vector<daxa_u32> dirty_shapes;
    dirty_shapes.reserve(fixes.size());
    for (auto const &fx : fixes)
    {
      if (fx.body < rigid_bodies.size() && rigid_bodies[fx.body].shape_index != 0u)
      {
        dirty_shapes.push_back(rigid_bodies[fx.body].shape_index - 1u);
      }
    }
    rigid_body_manager->build_voxel_pools_gpu(voxel_shape_cpu, voxel_sdf_cpu, voxel_surf_cpu, voxel_derived_cpu, &dirty_shapes);
    if (bb_getenv("BB_CENSUS_VERIFY") != nullptr || bb_getenv("BB_SDF_VERIFY") != nullptr)
    {
      auto const occupancy = rigid_body_manager->read_voxel_occupancy(BB_MAX_VOXEL_OCC_U32S);
      auto const *gpu = occupancy.data();
      for (auto si : dirty_shapes)
      {
        auto const &sh = voxel_shape_cpu[si];
        auto const words = (sh.dims.x*sh.dims.y*sh.dims.z+31u)/32u;
        if (std::memcmp(gpu+sh.occ_offset, voxel_occ_cpu.data()+sh.occ_offset, words*sizeof(daxa_u32)))
        { std::cerr << "[PACK-VERIFY] FAILED shape=" << si << std::endl; std::abort(); }
        std::cout << "[PACK-VERIFY] shape=" << si << " words=" << words << " MATCH" << std::endl;
      }
    }
    // Finalize after the AS manager records build metadata. The derived
    // records stay on the GPU and feed finalization in the AABB-generation submit.
    std::vector<FragmentPublicationReference> finalizations;
    finalizations.reserve(fixes.size());
    for (auto const &fx : fixes)
    {
      finalizations.push_back({.body = fx.body, .parent_id = fx.parent_id,
          .crop_off = {fx.crop_off.x, fx.crop_off.y, fx.crop_off.z}});
    }
    // Only AS build ranges are mirrored here. GPU publication computes the
    // same prefix and writes every physical body and primitive directly.
    aabb.clear();voxel_prim_sites.clear();
    daxa_u32 primitive_offset=0u;
    for (auto const &body:rigid_bodies) {
      if (body.shape_index!=0u) voxel_prim_sites.emplace_back(body.shape_index-1u,primitive_offset);
      primitive_offset+=body.primitive_count;
    }
    // 6. rebuild the active-body bookkeeping (counts + id set) from the live vector, so
    //    retired tombstones drop out and reused/new fragments join - all in one place
    rebuild_active_bookkeeping();
    auto _t1 = _now(); // pools+rebuild+derived+fixup+prims done
    // 7. AS rebuild + sim refresh (the reset() tail, minus the pause). INCREMENTAL by default:
    //    only the bodies whose geometry changed this fracture get their BLAS rebuilt; the ~hundreds
    //    of untouched bodies keep their baked BLAS (the dominant respawn cost). BB_AS_FULL forces
    //    the old full rebuild (reset + build all) for A/B comparison. The incremental path manages
    //    its own upload counters, so it must NOT be preceded by reset_for_reload (that clears the
    //    baseline and would force a full build every time).
    static bool const _as_full = bb_getenv("BB_AS_FULL") != nullptr;
    bool _ok;
    if (_as_full)
    {
      accel_struct_mngr->reset_for_reload();
      _ok = accel_struct_mngr->build_accel_structs(rigid_bodies, aabb, voxel_prims_hook(finalizations,true,live_count),true);
    }
    else
    {
      std::vector<daxa_u32> changed_bodies(extra_changed.begin(),extra_changed.end());
      changed_bodies.reserve(fixes.size());
      for (auto const &fx : fixes) changed_bodies.push_back(fx.body);
      _ok = accel_struct_mngr->update_accel_structs_incremental(rigid_bodies, aabb, voxel_prims_hook(finalizations,true,live_count), changed_bodies,true);
    }
    if (!_ok)
    {
      std::cerr << "FRACTURE: AS rebuild failed!" << std::endl;
      return;
    }
    auto _t1b = _now(); // AS structs (CPU: dirty-diff + create/size-query for changed bodies) done
    accel_struct_mngr->build_AS(true);
    auto _t2 = _now(); // publication enqueued; renderer completes before final TLAS
    rigid_body_manager->update_sim();
    rigid_body_manager->update_active_rigid_body_list();
    // Both update methods refresh both parities and restore current bindings.
    // Repeating them only submitted the same uploads twice and advanced the render clock.
    // All callers run inside the renderer's scene-edit block. That block publishes
    // the final TLAS once after fracture, culling and spawning, before tracing rays.
    // Publishing here built the same TLAS twice and inserted an extra device wait.
    if (_t) { auto _t3 = _now(); std::cout << "[RESPAWN-MS] pools+fixup=" << _ms(_t0,_t1) << " blas_cpu=" << _ms(_t1,_t1b) << " blas_gpu=" << _ms(_t1b,_t2) << " sim_upload=" << _ms(_t2,_t3) << " total=" << _ms(_t0,_t3) << " bodies=" << rigid_body_count << std::endl; }
    if (pool_verify_on()) { verify_pools("respawn"); }
    std::cout << "[FRACTURE] respawn: " << rigid_body_count << " bodies, "
              << voxel_shape_cpu.size() << " shapes" << std::endl;
  }

  // entry point, called from the render loop after each stepped frame (event source = the
  // dedicated GPU->host bridge buffer, NOT SimConfig - see FractureEventBuffer)
  void process_fracture_events(FractureEventBuffer const &fb)
  {
    daxa_u32 const serial = fb.serial;
    if (serial == fracture_serial_seen) { return; }
    daxa_u32 const n = std::min(fb.count, BB_MAX_FRACTURE_EVENTS);
    fracture_serial_seen = serial;
    rigid_body_manager->acknowledge_fracture_events(serial);
    auto const fracture_start = std::chrono::steady_clock::now();
    sync_pools_if_needed(); // capture the post-load high-water once, before the first alloc
    // MAIN-queue barriers order geometry edits after prior tracing. The batch
    // result wait completes those readers before the host retires any AS handles.
    auto const fracture_synced = std::chrono::steady_clock::now();
    bool any = false;
    std::vector<FragFix> fixes;
    bool const verify = bb_getenv("BB_CENSUS_VERIFY") || bb_getenv("BB_SDF_VERIFY") ||
                        bb_getenv("BB_POOL_VERIFY") || bb_getenv("BB_FRAGMENT_VERIFY");
    std::vector<FractureBatchChild> children;
    if (!verify) {
      std::vector<FracturePartitionInput> inputs;
      for (daxa_u32 s = 0u; s < n; ++s) {
        auto const id = fb.events[s].body_id;
        if (id >= rigid_bodies.size()) continue;
        auto const &body = rigid_bodies[id];
        if (body.shape_index == 0u || (body.flags & RigidBodyFlag::DYNAMIC) == RigidBodyFlag::NONE) continue;
        inputs.push_back({id, fracture_recorded_passes(voxel_shape_cpu[body.shape_index - 1u])});
      }
      children = rigid_body_manager->fracture_batch_gpu(inputs);
    }
    for (daxa_u32 s = 0u; s < n; ++s)
    {
      FractureEventSummary const &ev = fb.events[s];
      std::cout << "[FRACTURE] body " << ev.body_id << " impulse " << ev.impulse << std::endl;
      auto first = std::find_if(children.begin(), children.end(), [&](auto const &c) { return c.parent_id == ev.body_id; });
      auto last = first;
      while (last != children.end() && last->parent_id == ev.body_id) ++last;
      any = apply_fracture(ev, fixes, !verify, std::span<FractureBatchChild const>(first, last)) || any;
    }
    auto const fracture_split = std::chrono::steady_clock::now();
    if (any) { respawn_after_fracture(fixes); }
    if (bb_getenv("BB_RESPAWN_TIMING")) {
      auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b-a).count(); };
      auto const end = std::chrono::steady_clock::now();
      std::cout << "[FRACTURE-MS] sync=" << ms(fracture_start, fracture_synced)
                << " split=" << ms(fracture_synced, fracture_split)
                << " publish=" << ms(fracture_split, end)
                << " total=" << ms(fracture_start, end) << " events=" << n << std::endl;
    }
  }

  // KILL PLANE: retire any dynamic body that has fallen out of the world, freeing its shape
  // back to the pools (that memory is what exhausts under heavy shattering; without this the
  // fragments that fly off never release their slice). Gated by the render loop on the cheap
  // dbg_min_y signal so the readback + AS rebuild only run when something actually left.
  // One scene edit transaction: retirement makes slots available to a spawn
  // before the host mirrors metadata and publishes AS/body changes once.
  void process_scene_edits(daxa_u32 dbg_min_y_encoded)
  {
    bool const cull=any_body_below_kill_plane(dbg_min_y_encoded);
    bool spawn=false;
    if (spawner_on_) {
      static daxa_u32 const cadence=[] {
        char const *e=bb_getenv("BB_FRACTURE_SPAWN_STEPS");
        return e ? (daxa_u32)std::max(1,std::atoi(e)) : 45u;
      }();
      if (++spawn_step_>=cadence) { spawn_step_=0u;spawn=true; }
    }
    if (!cull && !spawn) return;
    sync_pools_if_needed();
    auto const edit=rigid_body_manager->edit_fracture_scene_gpu(cull,spawn,kill_y());
    if (edit.retired_count==0u && edit.spawn_id==MAX_U32) return;
    std::vector<daxa_u32> changed(edit.ids,edit.ids+edit.retired_count);
    for (auto id:changed) retire_body(id);
    if (edit.retired_count)
      std::cout << "[FRACTURE] GPU cull: " << edit.retired_count << " bodies" << std::endl;
    if (edit.spawn_id!=MAX_U32) {
      auto body=spawn_template(edit.spawn_template); // AS metadata only
      daxa_u32 const slot=edit.spawn_id;body.id=slot;
      std::erase(free_body_slots_,slot);
      if (slot<rigid_bodies.size()) rigid_bodies[slot]=body;
      else if (slot==rigid_bodies.size()) rigid_bodies.push_back(body);
      else { std::cerr << "GPU spawn slot gap FAILED" << std::endl;std::abort(); }
      changed.push_back(slot);
    }
    if (bb_getenv("BB_RESPAWN_TIMING"))
      std::cout << "[SCENE-EDIT] retired=" << edit.retired_count
                << " spawned=" << (edit.spawn_id!=MAX_U32 ? 1u : 0u) << std::endl;
    // A reused retired slot may occur twice; the AS dirty mask coalesces IDs.
    respawn_after_fracture({},changed);
  }
  // cheap gate (from the per-frame readback): is any dynamic body below the kill plane?
  bool any_body_below_kill_plane(daxa_u32 dbg_min_y_encoded)
  {
    if (dbg_min_y_encoded == 0xFFFFFFFFu) { return false; }
    f32 const miny = (f32)dbg_min_y_encoded / 1000.0f - 100.0f;
    return miny < kill_y();
  }
  // kill-plane height (m). BB_KILL_Y overrides the default (-20 = out of the world); a value
  // like 0.4 culls even settled debris, which is how the recycling path is stress-tested.
  f32 kill_y()
  {
    static f32 const y = [] {
      char const *e = bb_getenv("BB_KILL_Y");
      return e ? (f32)std::atof(e) : -20.0f;
    }();
    return y;
  }

  bool load_scene()
  {
    if (!initialized)
    {
      return false;
    }

    // Headless scene selection: BB_SCENE=N picks the launch scene ONCE (startup only, so F1-F10
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

    // Scene dispatch by current_scene (set at startup default + switched at runtime via F1-F10).
    // scene_1 .. scene_8 are compile-time builders; switch_scene() resets the host state so a
    // different builder can repopulate the shared vectors.
    // F3: BB_SCENE_FILE=path loads a data-driven scene from a text file instead of a compile-time
    // builder (no rebuild needed to iterate on a repro scene). F1-F10 still switch the built-in scenes.
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
        case 9: scene_9(); break; // FRACTURE showcase: strength-coded beams on anvils
        case 10: scene_10(); break; // breakable box pool with unbreakable walls
        case 11: scene_11(); break; // FRACTURE soak/free-list stress: runtime spawner + kill plane
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
        rigid_body.inv_inertia = cuboid_get_inverse_intertia(rigid_body.mass, rigid_body.minimum, rigid_body.maximum);
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
    status_manager->next_frame();
    rigid_body_manager->update_sim();
    status_manager->next_frame();

    material_TG.execute();
    light_TG.execute();

    // upload the voxel shape pools (static for the scene's lifetime; host-writable buffers
    // owned by the rigid body manager, read by the narrow phase)
    if (!voxel_shape_cpu.empty())
    {
      std::memcpy(device.buffer_host_address_as<VoxelShape>(rigid_body_manager->get_voxel_shapes_buffer()).value(),
                  voxel_shape_cpu.data(), voxel_shape_cpu.size() * sizeof(VoxelShape));
      rigid_body_manager->upload_voxel_occupancy(voxel_occ_cpu);
      // SDF and surface buffers are device-owned and entirely authored by the
      // builder. CPU arrays are optional reference data, never GPU initialization.
      rigid_body_manager->build_voxel_pools_gpu(voxel_shape_cpu, voxel_sdf_cpu, voxel_surf_cpu, voxel_derived_cpu);
    }

    // TODO: Handle error
    if (!accel_struct_mngr->build_accel_structs(rigid_bodies, aabb, voxel_prims_hook())) {
      std::cerr << "ERROR: Failed to build acceleration structures in scene_manager!" << std::endl;
      return false;
    }
    accel_struct_mngr->build_AS();
    rigid_body_manager->update_active_rigid_body_list(); // GPU body inputs now exist in both parities


    // The renderer builds the initial TLAS from its first coherent GPU snapshot.
    // The instance shader retains the same transform convention as runtime.

    std::cout << "SUCCESS: Scene loaded successfully with " << rigid_body_count << " rigid bodies!" << std::endl;
    return initialized;
  }

  // Rebuild the authored scene, including fresh GPU pools. Runtime metadata is
  // no longer a CPU copy of initial or live body physics after a fracture.
  bool reset()
  {
    return initialized && switch_scene(current_scene,true);
  }

  // Switch to a different scene at runtime (F1-F10): tear down the host-side scene state, rebuild
  // from scene_N(), and pause (like reset). The scene_N() builders push_back into the shared
  // vectors and assume them empty + load_scene() accumulates ids/counts/lights, so everything that
  // accumulates must be cleared here before re-running load_scene(). n is 1..10.
  bool switch_scene(int n, bool reload = false)
  {
    if (!initialized)
    {
      return false;
    }
    if (n < 1 || n > 11)
    {
      std::cerr << "SCENE: ignoring out-of-range scene " << n << std::endl;
      return false;
    }
    if (n == current_scene && !reload)
    {
      return reset(); // same scene restores the authored bodies and GPU pools
    }
    // Reset/switch can arrive while the preceding frame is still tracing. Both
    // queues must release the old scene before host uploads or BLAS retirement.
    device.wait_idle();
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
    occ_pool_.reset(); surf_pool_.reset(); sdf_pool_.reset();
    free_shape_slots_.clear();
    free_body_slots_.clear();
    spawner_on_ = false; spawn_shapes_.clear(); spawn_strengths_.clear(); spawn_mats_.clear();
    pools_synced_ = false; // re-sync to the new scene's post-load high-water on its first fracture
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
  // active scene (1..10); load_scene() dispatches on it, switch_scene() (F1-F10) changes it. Default
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
  // Last-consumed pending-impact generation (acknowledged without clearing GPU payloads).
  std::vector<bool> shape_private;
  daxa_u32 fracture_serial_seen = 0;
  // FRACTURE memory pools (phase 1): recycle freed shape slices instead of append-only
  // growth. high_water is synced to the post-load vector sizes so the LOAD path stays a
  // plain bump (untouched, zero regression); only the fracture path allocs/frees through
  // these. free_shape_slots_ recycles VoxelShape indices (and the parallel per-shape
  // vectors) of retired shapes.
  FreeListPool occ_pool_, surf_pool_, sdf_pool_;
  std::vector<daxa_u32> free_shape_slots_;
  std::vector<daxa_u32> free_body_slots_; // retired (tombstoned) rigid_bodies indices, reused by new fragments
  bool pools_synced_ = false;
  // FRACTURE SOAK (scene_11): a runtime spawner that rains DIVERSE breakable voxel shapes so
  // the free-list juggles many allocation sizes (each source shape has a different pool
  // footprint) over a long alloc/free churn. Deterministic (seeded) so any failure replays.
  bool spawner_on_ = false;
  std::vector<VoxelShapeBuild> spawn_shapes_;
  std::vector<f32> spawn_strengths_;
  std::vector<daxa_u32> spawn_mats_;
  daxa_u32 spawn_step_ = 0;

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
