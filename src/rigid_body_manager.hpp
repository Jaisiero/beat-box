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
  std::shared_ptr<TaskManager> task_manager;
  // Compute pipeline reference
  std::shared_ptr<daxa::ComputePipeline> pipeline_GJK;
  std::shared_ptr<daxa::ComputePipeline> pipeline;

  TaskGraph RB_TG;

  daxa::BufferId sim_config;
  daxa::BufferId collisions;

  // Task graph information for rigid body simulation
  daxa::TaskBuffer task_dispatch_buffer{{.initial_buffers = {}, .name = "RB_dispatch"}};
  daxa::TaskBuffer task_sim_config{{.initial_buffers = {}, .name = "RB_sim_config"}};
  daxa::TaskBuffer task_rigid_bodies{{.initial_buffers = {}, .name = "RB_task"}};
  daxa::TaskBuffer task_aabbs{{.initial_buffers = {}, .name = "RB_aabb_task"}};
  daxa::TaskBuffer task_collisions{{.initial_buffers = {}, .name = "RB_collisions"}};

  explicit RigidBodyManager(daxa::Device& device, 
  std::shared_ptr<TaskManager> task_manager) : device(device), task_manager(task_manager) {
    if(device.is_valid())
    {
      pipeline_GJK = task_manager->create_compute(GJKSim{}.info);
      pipeline = task_manager->create_compute(RigidBodySim{}.info);
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

    sim_config = device.create_buffer({
        .size = sizeof(SimConfig),
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
        .name = "sim_config",
    });

    collisions = device.create_buffer({
        .size = sizeof(Manifold) * MAX_RIGID_BODY_COUNT * MAX_RIGID_BODY_COUNT, // TODO: Change to a more reasonable size
        .name = "collisions",
    });

    *device.buffer_host_address_as<SimConfig>(sim_config).value() = SimConfig{
      .rigid_body_count = 0,
      .dt = TIME_STEP,
      .gravity = -GRAVITY,
      .collision_count = 0,
    };

    auto user_callback = [this](daxa::TaskInterface ti, auto& self) {
        ti.recorder.set_pipeline(*pipeline_GJK);
        ti.recorder.push_constant(GJKPushConstants{.task_head = ti.attachment_shader_blob});
        ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(GJKTaskHead::AT.dispatch_buffer).ids[0], .offset = 0});
    };

    using TTask = TaskTemplate<GJKTaskHead::Task, decltype(user_callback)>;

    // Instantiate the task using the template class
    TTask task(std::array{
        daxa::attachment_view(GJKTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
        daxa::attachment_view(GJKTaskHead::AT.sim_config, task_sim_config),
        daxa::attachment_view(GJKTaskHead::AT.rigid_bodies, task_rigid_bodies),
        daxa::attachment_view(GJKTaskHead::AT.aabbs, task_aabbs),
        daxa::attachment_view(GJKTaskHead::AT.collisions, task_collisions),
      }, user_callback);

    auto user_callback2 = [this](daxa::TaskInterface ti, auto& self) {
        ti.recorder.set_pipeline(*pipeline);
        ti.recorder.push_constant(RigidBodySimPushConstants{.task_head = ti.attachment_shader_blob});
        ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodySimTaskHead::AT.dispatch_buffer).ids[0], .offset = 0});
    };

    using TTask2 = TaskTemplate<RigidBodySimTaskHead::Task, decltype(user_callback2)>;

    // Instantiate the task using the template class
    TTask2 task2(std::array{
        daxa::attachment_view(RigidBodySimTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
        daxa::attachment_view(RigidBodySimTaskHead::AT.sim_config, task_sim_config),
        daxa::attachment_view(RigidBodySimTaskHead::AT.rigid_bodies, task_rigid_bodies),
        daxa::attachment_view(RigidBodySimTaskHead::AT.aabbs, task_aabbs),
      }, user_callback2);

    std::array<daxa::TaskBuffer, 5> buffers = {
        task_dispatch_buffer,
        task_sim_config,
        task_rigid_bodies,
        task_aabbs,
        task_collisions,
    };

    RB_TG = task_manager->create_task_graph(name, std::span<daxa::TaskBuffer>(buffers), {}, {}, {});

    RB_TG.add_task(task);
    RB_TG.add_task(task2);


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

    device.destroy_buffer(sim_config);
    device.destroy_buffer(collisions);

    initialized = false;
  }

  bool simulate()
  {
    if (!initialized)
    {
      return !initialized;
    }

    device.buffer_host_address_as<SimConfig>(sim_config).value()->collision_count = 0;

    RB_TG.execute();

    return initialized;
  }

  bool update_resources(daxa::BufferId dispatch_buffer, daxa::BufferId rigid_bodies, daxa::BufferId aabbs)
  {
    if (!initialized)
    {
      return !initialized;
    }

    task_dispatch_buffer.set_buffers({.buffers = std::array{dispatch_buffer}});
    task_sim_config.set_buffers({.buffers = std::array{sim_config}});
    task_rigid_bodies.set_buffers({.buffers = std::array{rigid_bodies}});
    task_aabbs.set_buffers({.buffers = std::array{aabbs}});
    task_collisions.set_buffers({.buffers = std::array{collisions}});

    return initialized;
  }

  bool update_sim(daxa_u32 rigid_body_count)
  {
    if (!initialized)
    {
      return !initialized;
    }

    *device.buffer_host_address_as<SimConfig>(sim_config).value() = SimConfig{
      .rigid_body_count = rigid_body_count,
      .dt = TIME_STEP,
      .gravity = -GRAVITY,
      .collision_count = 0,
    };

    return initialized;
  }
};

BB_NAMESPACE_END