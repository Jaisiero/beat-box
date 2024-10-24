#include "rigid_body_manager.hpp"
#include "renderer_manager.hpp"

BB_NAMESPACE_BEGIN

RigidBodyManager::RigidBodyManager(daxa::Device &device,
                                            std::shared_ptr<TaskManager> task_manager) : device(device), task_manager(task_manager)
{
  if (device.is_valid())
  {
    pipeline_BP = task_manager->create_compute(BroadPhaseInfo{}.info);
    pipeline_CS_dispatcher = task_manager->create_compute(CollisionSolverDispatcherInfo{}.info);
    pipeline = task_manager->create_compute(RigidBodySim{}.info);
    pipeline_CPS = task_manager->create_compute(CollisionPreSolverInfo{}.info);
    pipeline_CS = task_manager->create_compute(CollisionSolverInfo{}.info);
    create_points_pipeline = task_manager->create_compute(CreateContactPoints{}.info);
  }
}

RigidBodyManager::~RigidBodyManager()
{
  destroy();
}

daxa::BufferId RigidBodyManager::get_sim_config_buffer()
{
  // TODO: should frame_index be passed as a parameter?
  return sim_config[renderer_manager->get_frame_index()];
}

daxa::BufferId RigidBodyManager::get_collision_buffer()
{
  return collisions[renderer_manager->get_frame_index()];
}

bool RigidBodyManager::create(char const *name, std::shared_ptr<RendererManager> renderer, daxa_u32 iterations)
{
  if (initialized)
  {
    return false;
  }

  iteration_count = iterations;

  renderer_manager = renderer;

  sim_config_host_buffer = device.create_buffer({
      .size = sizeof(SimConfig),
      .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
      .name = "sim_config_host",
  });

  for (auto i = 0u; i < DOUBLE_BUFFERING; ++i)
    sim_config[i] = device.create_buffer({
        .size = sizeof(SimConfig),
        .name = "sim_config_" + std::to_string(i),
    });

  for (auto i = 0u; i < DOUBLE_BUFFERING; ++i)
    collisions[i] = device.create_buffer({
        .size = sizeof(Manifold) * MAX_RIGID_BODY_COUNT * MAX_RIGID_BODY_COUNT, // TODO: Change to a more reasonable size
        .name = "collisions" + std::to_string(i),
    });

  *device.buffer_host_address_as<SimConfig>(sim_config_host_buffer).value() = SimConfig{
      .rigid_body_count = 0,
      .dt = TIME_STEP,
      .gravity = -GRAVITY,
      .flags = SimFlag::ACCUM_IMPULSE|SimFlag::FRICTION,
      .g_c_info = GlobalCollisionInfo{
          .collision_count = 0,
          .collision_point_count = 0
      },
  };
  
  daxa::InlineTaskInfo task_old_RC({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_sim_config),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_old_sim_config),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        auto offset_collision_count = sizeof(SimConfig) - sizeof(GlobalCollisionInfo) - sizeof(GlobalCollisionInfo);
        auto offset_old_collision_count = sizeof(SimConfig) - sizeof(GlobalCollisionInfo);

        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = ti.get(task_sim_config).ids[0],
            .dst_buffer = ti.get(task_old_sim_config).ids[0],
            .src_offset = offset_collision_count,
            .dst_offset = offset_old_collision_count,
            .size = sizeof(GlobalCollisionInfo),
        });
      },
      .name = "reset sim config",
  });

  daxa::InlineTaskInfo task_RC({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_sim_config),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        auto reset_c_info = GlobalCollisionInfo{
            .collision_count = 0,
            .collision_point_count = 0,
        };

        auto dst_offset = sizeof(SimConfig) - sizeof(GlobalCollisionInfo) - sizeof(GlobalCollisionInfo);
        
        allocate_fill_copy(ti, reset_c_info, ti.get(task_sim_config), dst_offset);
      },
      .name = "reset sim config",
  });

  auto user_callback = [this](daxa::TaskInterface ti, auto &self)
  {
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
                 daxa::attachment_view(BroadPhaseTaskHead::AT.old_collisions, task_old_collisions),
             },
             user_callback);

  auto user_callback_CS_dispatcher = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_CS_dispatcher);
    ti.recorder.push_constant(CollisionSolverDispatcherPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };

  using TTask_CS_dispatcher = TaskTemplate<CollisionSolverDispatcherTaskHead::Task, decltype(user_callback_CS_dispatcher)>;

  // Instantiate the task using the template class
  TTask_CS_dispatcher task_CS_dispatcher(std::array{
                                             daxa::attachment_view(CollisionSolverDispatcherTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
                                             daxa::attachment_view(CollisionSolverDispatcherTaskHead::AT.sim_config, task_sim_config),
                                         },
                                         user_callback_CS_dispatcher);

  auto user_callback_CPS = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_CPS);
    ti.recorder.push_constant(CollisionPreSolverPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CollisionPreSolverTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3)});
  };

  using TTask_CPS = TaskTemplate<CollisionPreSolverTaskHead::Task, decltype(user_callback_CPS)>;

  // Instantiate the task using the template class
  TTask_CPS task_CPS(std::array{
                       daxa::attachment_view(CollisionPreSolverTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
                       daxa::attachment_view(CollisionPreSolverTaskHead::AT.sim_config, task_sim_config),
                       daxa::attachment_view(CollisionPreSolverTaskHead::AT.rigid_bodies, task_rigid_bodies),
                       daxa::attachment_view(CollisionPreSolverTaskHead::AT.collisions, task_collisions),
                   },
                   user_callback_CPS);

  auto user_callback_CS = [this](daxa::TaskInterface ti, auto &self)
  {
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
                   },
                   user_callback_CS);

  auto user_callback_advect = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline);
    ti.recorder.push_constant(RigidBodySimPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodySimTaskHead::AT.dispatch_buffer).ids[0], .offset = 0});
  };

  using TTaskAdvect = TaskTemplate<RigidBodySimTaskHead::Task, decltype(user_callback_advect)>;

  // Instantiate the task using the template class
  TTaskAdvect task_advect(std::array{
                   daxa::attachment_view(RigidBodySimTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
                   daxa::attachment_view(RigidBodySimTaskHead::AT.sim_config, task_sim_config),
                   daxa::attachment_view(RigidBodySimTaskHead::AT.rigid_bodies, task_rigid_bodies),
                   daxa::attachment_view(RigidBodySimTaskHead::AT.aabbs, task_aabbs),
               },
               user_callback_advect);

  auto user_callback_CP = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*create_points_pipeline);
    ti.recorder.push_constant(CreatePointsPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CreatePointsTaskHead::AT.dispatch_buffer).ids[0],
                                   .offset = sizeof(daxa_u32vec3)});
  };

  using TTaskCP = TaskTemplate<CreatePointsTaskHead::Task, decltype(user_callback_CP)>;

  // Instantiate the task using the template class
  TTaskCP task_CP(std::array{
                      daxa::attachment_view(CreatePointsTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
                      daxa::attachment_view(CreatePointsTaskHead::AT.sim_config, task_sim_config),
                      daxa::attachment_view(CreatePointsTaskHead::AT.collisions, task_collisions),
                      daxa::attachment_view(CreatePointsTaskHead::AT.point_aabbs, task_points),
                  },
                  user_callback_CP);

  std::array<daxa::TaskBuffer, 8> buffers = {
      task_dispatch_buffer,
      task_sim_config,
      task_old_sim_config,
      task_rigid_bodies,
      task_aabbs,
      task_collisions,
      task_old_collisions,
      task_points,
  };

  RB_TG = task_manager->create_task_graph(name, std::span<daxa::TaskBuffer>(buffers), {}, {}, {});

  RB_TG.add_task(task_old_RC);
  RB_TG.add_task(task_RC);
  RB_TG.add_task(task);
  RB_TG.add_task(task_CS_dispatcher);
  RB_TG.add_task(task_advect);
  RB_TG.add_task(task_CPS);
  for(auto i = 0u; i < iteration_count; ++i)
    RB_TG.add_task(task_CS);
  RB_TG.add_task(task_CP);

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

void RigidBodyManager::record_read_back_sim_config_tasks(TaskGraph &readback_SC_TG)
{
  daxa::InlineTaskInfo task_readback_SC({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::COMPUTE_SHADER_READ_WRITE, task_sim_config),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = sim_config[renderer_manager->get_frame_index()],
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

void RigidBodyManager::record_update_sim_config_tasks(TaskGraph &update_SC_TG)
{
  daxa::InlineTaskInfo task_update_SC({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_sim_config),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = sim_config_host_buffer,
            .dst_buffer = sim_config[renderer_manager->get_frame_index()],
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

void RigidBodyManager::destroy()
{
  if (!initialized)
  {
    return;
  }

  device.destroy_buffer(sim_config_host_buffer);
  for (auto i = 0u; i < DOUBLE_BUFFERING; ++i)
    device.destroy_buffer(sim_config[i]);
  for (auto i = 0u; i < DOUBLE_BUFFERING; ++i)
    device.destroy_buffer(collisions[i]);

  initialized = false;
}

bool RigidBodyManager::simulate(daxa::BufferId rigid_bodies,
                                daxa::BufferId points_buffer)
{
  if (!initialized)
  {
    return !initialized;
  }

  
  update_buffers(rigid_bodies);
  // TODO: refactor
  task_points.set_buffers({.buffers = std::array{points_buffer}});

  RB_TG.execute();

  return initialized;
}

bool RigidBodyManager::update_resources(daxa::BufferId dispatch_buffer, daxa::BufferId rigid_bodies, daxa::BufferId aabbs, daxa::BufferId points_buffer)
{
  if (!initialized)
  {
    return !initialized;
  }

  task_dispatch_buffer.set_buffers({.buffers = std::array{dispatch_buffer}});
  task_rigid_bodies.set_buffers({.buffers = std::array{rigid_bodies}});
  task_aabbs.set_buffers({.buffers = std::array{aabbs}});
  task_points.set_buffers({.buffers = std::array{points_buffer}});

  return initialized;
}

// NOTE: this function reset simulation configuration
bool RigidBodyManager::update_sim(daxa_u32 rigid_body_count, daxa::BufferId rigid_body_buffer)
{
  if (!initialized)
  {
    return !initialized;
  }

  *device.buffer_host_address_as<SimConfig>(sim_config_host_buffer).value() = SimConfig{
      .rigid_body_count = rigid_body_count,
      .dt = TIME_STEP,
      .gravity = -GRAVITY,
      .flags = SimFlag::ACCUM_IMPULSE|SimFlag::FRICTION,
      .g_c_info = GlobalCollisionInfo{
          .collision_count = 0,
          .collision_point_count = 0,
      },
  };

  update_buffers(rigid_body_buffer);

  update_SC_TG.execute();

  return initialized;
}

bool RigidBodyManager::read_back_sim_config()
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

void RigidBodyManager::update_buffers(daxa::BufferId rigid_bodies)
{
  daxa_u32 previous_frame = (renderer_manager->get_frame_index() + DOUBLE_BUFFERING - 1) % DOUBLE_BUFFERING;

  task_sim_config.set_buffers({.buffers = std::array{sim_config[renderer_manager->get_frame_index()]}});
  task_old_sim_config.set_buffers({.buffers = std::array{sim_config[previous_frame]}});
  task_rigid_bodies.set_buffers({.buffers = std::array{rigid_bodies}});
  task_collisions.set_buffers({.buffers = std::array{collisions[renderer_manager->get_frame_index()]}});
  task_old_collisions.set_buffers({.buffers = std::array{collisions[previous_frame]}});
}

BB_NAMESPACE_END