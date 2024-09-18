#pragma once

#include "defines.hpp"
#include "task_manager.hpp"

BB_NAMESPACE_BEGIN

struct RigidBodyManager{
  // Device reference
  daxa::Device& device;
  // Initialization flag
  bool initialized = false;
  // Task manager reference
  TaskManager& task_manager;
  // Compute pipeline reference
  std::shared_ptr<daxa::ComputePipeline> pipeline;

  TaskGraph RB_TG;

  daxa::BufferId dispatch_buffer;
  daxa::BufferId sim_config;

  // Task graph information for rigid body simulation
  daxa::TaskBuffer task_dispatch_buffer{{.initial_buffers = {}, .name = "RB_dispatch_buffer"}};
  daxa::TaskBuffer task_sim_config{{.initial_buffers = {}, .name = "RB_sim_config"}};
  daxa::TaskBuffer task_rigid_bodies{{.initial_buffers = {}, .name = "RB_task"}};
  daxa::TaskBuffer task_aabbs{{.initial_buffers = {}, .name = "RB_aabb_task"}};

  explicit RigidBodyManager(daxa::Device& device, TaskManager task_manager) : device(device), task_manager(task_manager) {
    if(device.is_valid())
    {
      pipeline = task_manager.create_compute(RigidBodySim{}.info);
    }
  }

  ~RigidBodyManager() {
    destroy();
  }

  bool create(char const* name)
  {
    if (initialized)
    {
      return false;
    }

    dispatch_buffer = device.create_buffer({
        .size = sizeof(daxa_u32vec3),
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
        .name = "RB_dispatch_buffer",
    });

    *device.buffer_host_address_as<daxa_u32vec3>(dispatch_buffer).value() = daxa_u32vec3(1, 1, 1);

    sim_config = device.create_buffer({
        .size = sizeof(SimConfig),
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
        .name = "sim_config",
    });

    *device.buffer_host_address_as<SimConfig>(sim_config).value() = SimConfig{
      .rigid_body_count = 0,
      .dt = TIME_STEP,
      .gravity = -GRAVITY,
    };

    auto user_callback = [this](daxa::TaskInterface ti, auto& self) {
        ti.recorder.set_pipeline(*pipeline);
        ti.recorder.push_constant(RigidBodySimPushConstants{.task_head = ti.attachment_shader_blob});
        ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodySimTaskHead::AT.dispatch_buffer).ids[0], .offset = 0});
    };

    using TTask = TaskTemplate<RigidBodySimTaskHead::Task, decltype(user_callback)>;

    // Instantiate the task using the template class
    TTask task(std::array{
        daxa::attachment_view(RigidBodySimTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
        daxa::attachment_view(RigidBodySimTaskHead::AT.sim_config, task_sim_config),
        daxa::attachment_view(RigidBodySimTaskHead::AT.rigid_bodies, task_rigid_bodies),
        daxa::attachment_view(RigidBodySimTaskHead::AT.aabbs, task_aabbs),
      }, user_callback);

    std::array<TTask, 1> tasks = { task };

    std::array<daxa::TaskBuffer, 4> buffers = {
        task_dispatch_buffer,
        task_sim_config,
        task_rigid_bodies,
        task_aabbs
    };

    RB_TG = task_manager.create_task_graph(name, std::span<TTask>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, {}, {});

    RB_TG.submit();
    RB_TG.complete();

    return initialized = true;
  }

  void destroy()
  {
    if (!initialized)
    {
      return;
    }

    device.destroy_buffer(dispatch_buffer);
    device.destroy_buffer(sim_config);

    initialized = false;
  }

  bool simulate()
  {
    if (!initialized)
    {
      return !initialized;
    }

    RB_TG.execute();

    return initialized;
  }

  bool update_resources(daxa::BufferId rigid_bodies, daxa::BufferId aabbs)
  {
    if (!initialized)
    {
      return !initialized;
    }

    task_dispatch_buffer.set_buffers({.buffers = std::array{dispatch_buffer}});
    task_sim_config.set_buffers({.buffers = std::array{sim_config}});
    task_rigid_bodies.set_buffers({.buffers = std::array{rigid_bodies}});
    task_aabbs.set_buffers({.buffers = std::array{aabbs}});

    return initialized;
  }

  bool update_dispatch_buffer(daxa_u32 rigid_body_count)
  {
    if (!initialized)
    {
      return !initialized;
    }

    *device.buffer_host_address_as<daxa_u32vec3>(dispatch_buffer).value() = daxa_u32vec3((rigid_body_count + RIGID_BODY_SIM_COMPUTE_X - 1) / RIGID_BODY_SIM_COMPUTE_X, 1, 1);

    *device.buffer_host_address_as<SimConfig>(sim_config).value() = SimConfig{
      .rigid_body_count = rigid_body_count,
      .dt = TIME_STEP,
      .gravity = -GRAVITY,
    };

    return initialized;
  }
};

BB_NAMESPACE_END