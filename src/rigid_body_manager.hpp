#pragma once

#include "defines.hpp"
#include <daxa/utils/task_graph.hpp>

BB_NAMESPACE_BEGIN

struct RigidBodySimTask : RigidBodySimTaskHead::Task
{
  AttachmentViews views = {};
  std::shared_ptr<daxa::ComputePipeline> pipeline = {};

  void callback(daxa::TaskInterface ti)
  {
    ti.recorder.set_pipeline(*pipeline);
    ti.recorder.push_constant(RigidBodySimPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodySimTask::AT.dispatch_buffer).ids[0], .offset = 0});
  };
};

struct RigidBodyManager{
  // Device reference
  daxa::Device& device;
  // Initialization flag
  bool initialized = false;

  daxa::BufferId dispatch_buffer;
  daxa::BufferId sim_config;

  // Task graph information for rigid body simulation
  daxa::TaskGraph rigid_body_task_graph;  
  daxa::TaskBuffer task_dispatch_buffer{{.initial_buffers = {}, .name = "RB_dispatch_buffer"}};
  daxa::TaskBuffer task_sim_config{{.initial_buffers = {}, .name = "RB_sim_config"}};
  daxa::TaskBuffer task_rigid_bodies{{.initial_buffers = {}, .name = "RB_task"}};
  daxa::TaskBuffer task_aabbs{{.initial_buffers = {}, .name = "RB_aabb_task"}};

  explicit RigidBodyManager(daxa::Device& device) : device(device) {}

  ~RigidBodyManager() {
    destroy();
  }

  bool create(char const* name, std::shared_ptr<daxa::ComputePipeline> pipeline)
  {
    if (initialized)
    {
      return false;
    }

    rigid_body_task_graph = daxa::TaskGraph({
        .device = device,
        .name = name,
    });

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

    rigid_body_task_graph.use_persistent_buffer(task_dispatch_buffer);
    rigid_body_task_graph.use_persistent_buffer(task_sim_config);
    rigid_body_task_graph.use_persistent_buffer(task_rigid_bodies);
    rigid_body_task_graph.use_persistent_buffer(task_aabbs);

    rigid_body_task_graph.add_task(RigidBodySimTask{
      .views = std::array{
        daxa::attachment_view(RigidBodySimTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
        daxa::attachment_view(RigidBodySimTaskHead::AT.sim_config, task_sim_config),
        daxa::attachment_view(RigidBodySimTaskHead::AT.rigid_bodies, task_rigid_bodies),
        daxa::attachment_view(RigidBodySimTaskHead::AT.aabbs, task_aabbs),
      },
      .pipeline = pipeline,
    });

    rigid_body_task_graph.submit({});
    rigid_body_task_graph.complete({});

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

    rigid_body_task_graph.execute({});

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