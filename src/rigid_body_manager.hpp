#pragma once

#include "defines.hpp"
#include "task_manager.hpp"

BB_NAMESPACE_BEGIN

struct RigidBodyManager{
  // Device reference
  daxa::Device& device;
  // Initialization flag
  bool initialized = false;
  // Current index for compute execution
  daxa_u32 compute_index = 0;

  // Task manager reference
  std::shared_ptr<TaskManager> task_manager;
  // Compute pipeline reference
  std::shared_ptr<daxa::ComputePipeline> pipeline_RC;
  std::shared_ptr<daxa::ComputePipeline> pipeline_BP;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CS_dispatcher;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CS;
  std::shared_ptr<daxa::ComputePipeline> pipeline;

  // TaskGraph for rigid body simulation
  TaskGraph RB_TG;

  // TaskGraph for read-back of simulation configuration
  TaskGraph readback_SC_TG;

  // TaskGraph to update simulation configuration
  TaskGraph update_SC_TG;

  daxa::BufferId sim_config_host_buffer;
  daxa::BufferId sim_config[DOUBLE_BUFFERING] = {};
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
      pipeline_RC = task_manager->create_compute(ResetCollisionsInfo{}.info);
      pipeline_BP = task_manager->create_compute(BroadPhaseInfo{}.info);
      pipeline_CS_dispatcher = task_manager->create_compute(CollisionSolverDispatcherInfo{}.info);
      pipeline_CS = task_manager->create_compute(CollisionSolverInfo{}.info);
      pipeline = task_manager->create_compute(RigidBodySim{}.info);
    }
  }

  ~RigidBodyManager() {
    destroy();
  }

  daxa::BufferId get_sim_config_buffer() {
    return sim_config[compute_index];
  }

  bool create(char const* name)
  {
    if (initialized)
    {
      return false;
    }

    sim_config_host_buffer = device.create_buffer({
        .size = sizeof(SimConfig),
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "sim_config_host",
    });

    for(auto i = 0u; i < DOUBLE_BUFFERING; ++i)
      sim_config[i] = device.create_buffer({
          .size = sizeof(SimConfig),
          .name = "sim_config_" + std::to_string(i),
      });

    collisions = device.create_buffer({
        .size = sizeof(Manifold) * MAX_RIGID_BODY_COUNT * MAX_RIGID_BODY_COUNT, // TODO: Change to a more reasonable size
        .name = "collisions",
    });

    *device.buffer_host_address_as<SimConfig>(sim_config_host_buffer).value() = SimConfig{
      .rigid_body_count = 0,
      .dt = TIME_STEP,
      .gravity = -GRAVITY,
      .collision_count = 0,
    };

    auto user_callback0 = [this](daxa::TaskInterface ti, auto& self) {
        ti.recorder.set_pipeline(*pipeline_RC);
        ti.recorder.push_constant(ResetCollisionsPushConstants{.task_head = ti.attachment_shader_blob});
        ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
    };

    using TTaskRC = TaskTemplate<ResetCollisionsTaskHead::Task, decltype(user_callback0)>;

    // Instantiate the task using the template class
    TTaskRC task_RC(std::array{
        daxa::attachment_view(ResetCollisionsTaskHead::AT.sim_config, task_sim_config),
      }, user_callback0);

    auto user_callback = [this](daxa::TaskInterface ti, auto& self) {
        ti.recorder.set_pipeline(*pipeline_BP);
        ti.recorder.push_constant(BroadPhasePushConstants{.task_head = ti.attachment_shader_blob});
        ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(BroadPhaseTaskHead::AT.dispatch_buffer).ids[0], .offset = 0});
    };

    using TTask = TaskTemplate<BroadPhaseTaskHead::Task, decltype(user_callback)>;

    // Instantiate the task using the template class
    TTask task(std::array{
        daxa::attachment_view(BroadPhaseTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
        daxa::attachment_view(BroadPhaseTaskHead::AT.sim_config, task_sim_config),
        daxa::attachment_view(BroadPhaseTaskHead::AT.rigid_bodies, task_rigid_bodies),
        daxa::attachment_view(BroadPhaseTaskHead::AT.aabbs, task_aabbs),
        daxa::attachment_view(BroadPhaseTaskHead::AT.collisions, task_collisions),
      }, user_callback);

    auto user_callback_CS_dispatcher = [this](daxa::TaskInterface ti, auto& self) {
        ti.recorder.set_pipeline(*pipeline_CS_dispatcher);
        ti.recorder.push_constant(CollisionSolverDispatcherPushConstants{.task_head = ti.attachment_shader_blob});
        ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
    };

    using TTask_CS_dispatcher = TaskTemplate<CollisionSolverDispatcherTaskHead::Task, decltype(user_callback_CS_dispatcher)>;

    // Instantiate the task using the template class
    TTask_CS_dispatcher task_CS_dispatcher(std::array{
        daxa::attachment_view(CollisionSolverDispatcherTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
        daxa::attachment_view(CollisionSolverDispatcherTaskHead::AT.sim_config, task_sim_config),
      }, user_callback_CS_dispatcher);

    auto user_callback_CS = [this](daxa::TaskInterface ti, auto& self) {
        ti.recorder.set_pipeline(*pipeline_CS);
        ti.recorder.push_constant(CollisionSolverPushConstants{.task_head = ti.attachment_shader_blob});
        ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CollisionSolverTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3)});
    };

    using TTask_CS = TaskTemplate<CollisionSolverTaskHead::Task, decltype(user_callback_CS)>;

    // Instantiate the task using the template class
    TTask_CS task_CS(std::array{
        daxa::attachment_view(CollisionSolverTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
        daxa::attachment_view(CollisionSolverTaskHead::AT.sim_config, task_sim_config),
        daxa::attachment_view(CollisionSolverTaskHead::AT.rigid_bodies, task_rigid_bodies),
        daxa::attachment_view(CollisionSolverTaskHead::AT.collisions, task_collisions),
      }, user_callback_CS);

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

    RB_TG.add_task(task_RC);
    RB_TG.add_task(task);
    RB_TG.add_task(task_CS_dispatcher);
    RB_TG.add_task(task_CS);
    RB_TG.add_task(task2);


    RB_TG.submit();
    RB_TG.complete();


    record_read_back_sim_config_tasks(readback_SC_TG);
    readback_SC_TG.submit();
    readback_SC_TG.complete();

    record_update_sim_config_tasks(update_SC_TG);
    update_SC_TG.submit();
    update_SC_TG.complete();

    return initialized = true;
  }


  void record_read_back_sim_config_tasks(TaskGraph &readback_SC_TG)
  {
    daxa::InlineTaskInfo task_readback_SC({
        .attachments = {
            daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_sim_config),
        },
        .task = [this](daxa::TaskInterface const &ti)
        { 
          ti.recorder.copy_buffer_to_buffer({
            .src_buffer = sim_config[compute_index],
            .dst_buffer = sim_config_host_buffer,
            .size = sizeof(SimConfig),
          });

        },
        .name = "read back sim config",
    });

    std::array<daxa::TaskBuffer, 1> buffers = {
        task_sim_config,
    };
    
    std::array<daxa::InlineTaskInfo, 1> tasks = {
        task_readback_SC,
    };


    readback_SC_TG = task_manager->create_task_graph("Read back Simulation Configuration", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, {}, {});
  }

  
  void record_update_sim_config_tasks(TaskGraph &update_SC_TG)
  {
    daxa::InlineTaskInfo task_update_SC({
        .attachments = {
            daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_sim_config),
        },
        .task = [this](daxa::TaskInterface const &ti)
        {
          ti.recorder.copy_buffer_to_buffer({
            .src_buffer = sim_config_host_buffer,
            .dst_buffer = sim_config[compute_index],
            .size = sizeof(SimConfig),
          });
        },
        .name = "update sim config",
    });

    std::array<daxa::TaskBuffer, 1> buffers = {
        task_sim_config,
    };

    std::array<daxa::InlineTaskInfo, 1> tasks = {
        task_update_SC,
    };

    update_SC_TG = task_manager->create_task_graph("Update Simulation Configuration", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, {}, {});
  }

  void destroy()
  {
    if (!initialized)
    {
      return;
    }

    device.destroy_buffer(sim_config_host_buffer);
    for(auto i = 0u; i < DOUBLE_BUFFERING; ++i)
      device.destroy_buffer(sim_config[i]);
    device.destroy_buffer(collisions);

    initialized = false;
  }

  bool simulate()
  {
    if (!initialized)
    {
      return !initialized;
    }

    update_buffers();

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
    task_rigid_bodies.set_buffers({.buffers = std::array{rigid_bodies}});
    task_aabbs.set_buffers({.buffers = std::array{aabbs}});
    task_collisions.set_buffers({.buffers = std::array{collisions}});

    return initialized;
  }

  // NOTE: this function reset simulation configuration
  bool update_sim(daxa_u32 rigid_body_count)
  {
    if (!initialized)
    {
      return !initialized;
    }

    *device.buffer_host_address_as<SimConfig>(sim_config_host_buffer).value() = SimConfig{
      .rigid_body_count = rigid_body_count,
      .dt = TIME_STEP,
      .gravity = -GRAVITY,
      .collision_count = 0,
      .collision_point_count = 0,
    };

    update_buffers();

    update_SC_TG.execute();
    
    update_buffers();

    update_SC_TG.execute();

    return initialized;
  }

  bool read_back_sim_config()
  {
    if (!initialized)
    {
      return !initialized;
    }

    readback_SC_TG.execute();
    // TODO: change for wait queue
    device.wait_idle();

    return initialized;
  }

private: 
  void update_buffers()
  {
    compute_index = (compute_index + 1) % DOUBLE_BUFFERING;
    task_sim_config.set_buffers({.buffers = std::array{sim_config[compute_index]}});
  }

};

BB_NAMESPACE_END