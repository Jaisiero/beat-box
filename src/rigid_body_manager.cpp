#include "rigid_body_manager.hpp"
#include "renderer_manager.hpp"

BB_NAMESPACE_BEGIN

RigidBodyManager::RigidBodyManager(daxa::Device &device,
                                            std::shared_ptr<TaskManager> task_manager,
                                            std::shared_ptr<AccelerationStructureManager> accel_struct_mngr) : device(device), task_manager(task_manager), accel_struct_mngr(accel_struct_mngr)
{
  if (device.is_valid())
  {
    pipeline_RBL = task_manager->create_compute(ResetBodyLinksInfo{}.info);
    pipeline_BP = task_manager->create_compute(BroadPhaseInfo{}.info);
    pipeline_advect = task_manager->create_compute(RigidBodySim{}.info);
    pipeline_IC = task_manager->create_compute(IslandCounterInfo{}.info);
    pipeline_CS_dispatcher = task_manager->create_compute(CollisionSolverDispatcherInfo{}.info);
    pipeline_IB = task_manager->create_compute(IslandBuilderInfo{}.info);
    pipeline_IPS = task_manager->create_compute(IslandPrefixSumInfo{}.info);
    pipeline_IBL = task_manager->create_compute(BodyLink2IslandInfo{}.info);
    pipeline_SBLI = task_manager->create_compute(SortBodyLinksInIslandInfo{}.info);
    pipeline_CPS = task_manager->create_compute(CollisionPreSolverInfo{}.info);
    pipeline_CS = task_manager->create_compute(CollisionSolverInfo{}.info);
    pipeline_IP = task_manager->create_compute(IntegratePositionsInfo{}.info);
    pipeline_CSR = task_manager->create_compute(CollisionSolverRelaxationInfo{}.info);
    create_points_pipeline = task_manager->create_compute(CreateContactPoints{}.info);
    update_pipeline = task_manager->create_compute(UpdateRigidBodies{}.info);
  }
}

RigidBodyManager::~RigidBodyManager()
{
  destroy();
}

SimConfig& RigidBodyManager::get_sim_config_reference() {
  return *device.buffer_host_address_as<SimConfig>(sim_config_host_buffer[renderer_manager->get_frame_index()]).value();
}

daxa::BufferId RigidBodyManager::get_sim_config_host_buffer() {
  return sim_config_host_buffer[renderer_manager->get_frame_index()];
}



void RigidBodyManager::record_active_rigid_body_list_upload_tasks(TaskGraph &ARBL_TG) {
  daxa::InlineTaskInfo task_update_active_rigid_bodies({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_active_rigid_bodies),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        auto active_rigid_bodies = renderer_manager->get_active_rigid_bodies();

        allocate_fill_copy(ti, active_rigid_bodies, ti.get(task_active_rigid_bodies), 0);
      },
      .name = "upload materials",
  });
  std::array<daxa::TaskBuffer, 1> buffers = {
    task_active_rigid_bodies
  };
  std::array<daxa::InlineTaskInfo, 1> tasks = {
    task_update_active_rigid_bodies
  };
  ARBL_TG = task_manager->create_task_graph("Active Rigid Body List Upload", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, {}, {});
}

bool RigidBodyManager::create(char const *name, std::shared_ptr<RendererManager> renderer, std::shared_ptr<GUIManager> gui, daxa_u32 iterations)
{
  if (initialized)
  {
    return false;
  }

  renderer_manager = renderer;
  gui_manager = gui;
  iteration_count = iterations;

  for(auto i = 0u; i < DOUBLE_BUFFERING; ++i) {
    sim_config_host_buffer[i] = device.create_buffer({
        .size = sizeof(SimConfig),
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "sim_config_host_" + std::to_string(i),
    });
    sim_config[i] = device.create_buffer({
        .size = sizeof(SimConfig),
        .name = "sim_config_" + std::to_string(i),
    });
    collisions[i] = device.create_buffer({
        .size = sizeof(Manifold) * MAX_RIGID_BODY_COUNT * 6, // TODO: Change to a more reasonable size
        .name = "collisions" + std::to_string(i),
    });
    active_rigid_bodies[i] = device.create_buffer({
        .size = sizeof(ActiveRigidBody) * MAX_RIGID_BODY_COUNT,
        .name = "active_rigid_bodies" + std::to_string(i),
    });
    scratch_body_links[i] = device.create_buffer({
        .size = sizeof(BodyLink) * MAX_RIGID_BODY_COUNT,
        .name = "scratch_body_links" + std::to_string(i),
    });
    body_links[i] = device.create_buffer({
        .size = sizeof(BodyLinkIsland) * MAX_RIGID_BODY_COUNT,
        .name = "body_links" + std::to_string(i),
    });
    island_buffer[i] = device.create_buffer({
        .size = sizeof(Island) * MAX_RIGID_BODY_COUNT,
        .name = "islands" + std::to_string(i),
    });
    *device.buffer_host_address_as<SimConfig>(sim_config_host_buffer[i]).value() = SimConfig{
        .solver_type = renderer_manager->get_solver(),
        .rigid_body_count = 0,
        .active_rigid_body_count = 0,
        .island_count = 0,
        .dt = TIME_STEP,
        .gravity = -GRAVITY,
        .flags = sim_flags,
        .frame_count = 0,
        .g_c_info = GlobalCollisionInfo{
            .collision_count = 0,
            .collision_point_count = 0
        },
    };
  }
  
  daxa::InlineTaskInfo task_RC({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_sim_config),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        // TODO: simplify this
        auto rigid_body_count = renderer_manager->get_rigid_body_count();
        auto rigid_body_count_offset = sizeof(SimSolverType);
        allocate_fill_copy(ti, rigid_body_count, ti.get(task_sim_config), rigid_body_count_offset);

        auto active_rigid_body_count = renderer_manager->get_active_rigid_body_count();
        auto active_rigid_body_offset = sizeof(SimSolverType) + sizeof(daxa_u32);
        allocate_fill_copy(ti, active_rigid_body_count, ti.get(task_sim_config), active_rigid_body_offset);

        auto island_count = 0;
        auto island_count_offset = sizeof(SimSolverType) + sizeof(daxa_u32) * 2;
        allocate_fill_copy(ti, island_count, ti.get(task_sim_config), island_count_offset);

        auto frame_count = renderer_manager->get_frame_count();
        auto frame_count_dst_offset = sizeof(SimConfig) - sizeof(GlobalCollisionInfo) - sizeof(daxa_u64);        
        allocate_fill_copy(ti, frame_count, ti.get(task_sim_config), frame_count_dst_offset);

        auto reset_c_info = GlobalCollisionInfo{
            .collision_count = 0,
            .collision_point_count = 0,
        };
        auto dst_offset = sizeof(SimConfig) - sizeof(GlobalCollisionInfo);
        allocate_fill_copy(ti, reset_c_info, ti.get(task_sim_config), dst_offset);
      },
      .name = "reset sim config",
  });

  
  // Task for reseting body links for islands
  auto user_callback_RBL = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_RBL);
    ti.recorder.push_constant(ResetBodyLinkPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(ResetBodyLinkTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_RBL = TaskTemplate<ResetBodyLinkTaskHead::Task, decltype(user_callback_RBL)>;

  // Instantiate the task using the template class
  TTask_RBL task_RBL(std::array{
                         daxa::attachment_view(ResetBodyLinkTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(ResetBodyLinkTaskHead::AT.sim_config, task_sim_config),
                         daxa::attachment_view(ResetBodyLinkTaskHead::AT.rigid_bodies, task_rigid_bodies),
                         daxa::attachment_view(ResetBodyLinkTaskHead::AT.active_rigid_bodies, task_active_rigid_bodies),
                         daxa::attachment_view(ResetBodyLinkTaskHead::AT.scratch_body_links, task_scratch_body_links),
                     },
                     user_callback_RBL);

  auto user_callback_BP = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_BP);
    ti.recorder.push_constant(BroadPhasePushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(BroadPhaseTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_BP = TaskTemplate<BroadPhaseTaskHead::Task, decltype(user_callback_BP)>;

  // Instantiate the task using the template class
  TTask_BP task_BP(std::array{
                 daxa::attachment_view(BroadPhaseTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                 daxa::attachment_view(BroadPhaseTaskHead::AT.sim_config, task_sim_config),
                 daxa::attachment_view(BroadPhaseTaskHead::AT.previous_sim_config, task_old_sim_config),
                 daxa::attachment_view(BroadPhaseTaskHead::AT.rigid_bodies, task_rigid_bodies),
                 daxa::attachment_view(BroadPhaseTaskHead::AT.collisions, task_collisions),
                 daxa::attachment_view(BroadPhaseTaskHead::AT.old_collisions, task_old_collisions),
                  daxa::attachment_view(BroadPhaseTaskHead::AT.scratch_body_links, task_scratch_body_links),
             },
             user_callback_BP);

  auto user_callback_IC = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_IC);
    ti.recorder.push_constant(IslandCounterPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IslandCounterTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_IC = TaskTemplate<IslandCounterTaskHead::Task, decltype(user_callback_IC)>;

  // Instantiate the task using the template class
  TTask_IC task_IC(std::array{
                       daxa::attachment_view(IslandCounterTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(IslandCounterTaskHead::AT.sim_config, task_sim_config),
                        daxa::attachment_view(IslandCounterTaskHead::AT.scratch_body_links, task_scratch_body_links),
                        daxa::attachment_view(IslandCounterTaskHead::AT.islands, task_islands),
                   },
                   user_callback_IC);

  auto user_callback_CS_dispatcher = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_CS_dispatcher);
    ti.recorder.push_constant(CollisionSolverDispatcherPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };

  using TTask_CS_dispatcher = TaskTemplate<CollisionSolverDispatcherTaskHead::Task, decltype(user_callback_CS_dispatcher)>;

  // Instantiate the task using the template class
  TTask_CS_dispatcher task_CS_dispatcher(std::array{
                                             daxa::attachment_view(CollisionSolverDispatcherTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                                             daxa::attachment_view(CollisionSolverDispatcherTaskHead::AT.sim_config, task_sim_config),
                                         },
                                         user_callback_CS_dispatcher);

  auto user_callback_IB = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_IB);
    ti.recorder.push_constant(IslandBuilderPushConstants{.task_head = ti.attachment_shader_blob}); 
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IslandBuilderTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_IB = TaskTemplate<IslandBuilderTaskHead::Task, decltype(user_callback_IB)>;

  // Instantiate the task using the template class
  TTask_IB task_IB(std::array{
                       daxa::attachment_view(IslandBuilderTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(IslandBuilderTaskHead::AT.sim_config, task_sim_config),
                        daxa::attachment_view(IslandBuilderTaskHead::AT.scratch_body_links, task_scratch_body_links),
                        daxa::attachment_view(IslandBuilderTaskHead::AT.islands, task_islands),
                   },
                   user_callback_IB);
                   
  auto user_callback_IPS = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_IPS);
    ti.recorder.push_constant(IslandPrefixSumPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };

  using TTask_IPS = TaskTemplate<IslandPrefixSumTaskHead::Task, decltype(user_callback_IPS)>;

  // Instantiate the task using the template class
  TTask_IPS task_IPS(std::array{
                       daxa::attachment_view(IslandPrefixSumTaskHead::AT.sim_config, task_sim_config),
                        daxa::attachment_view(IslandPrefixSumTaskHead::AT.islands, task_islands),
                   },
                   user_callback_IPS);    

  auto user_callback_IBL = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_IBL);
    ti.recorder.push_constant(IslandBuilderBodyLink2IslandPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IslandBuilderBodyLink2IslandTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_IBL = TaskTemplate<IslandBuilderBodyLink2IslandTaskHead::Task, decltype(user_callback_IBL)>;

  // Instantiate the task using the template class
  TTask_IBL task_IBL(std::array{
                       daxa::attachment_view(IslandBuilderBodyLink2IslandTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(IslandBuilderBodyLink2IslandTaskHead::AT.sim_config, task_sim_config),
                        daxa::attachment_view(IslandBuilderBodyLink2IslandTaskHead::AT.scratch_body_links, task_scratch_body_links),
                        daxa::attachment_view(IslandBuilderBodyLink2IslandTaskHead::AT.islands, task_islands),
                        daxa::attachment_view(IslandBuilderBodyLink2IslandTaskHead::AT.body_links, task_body_links),
                   },
                   user_callback_IBL);               

  auto user_callback_CPS = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_CPS);
    ti.recorder.push_constant(CollisionPreSolverPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CollisionPreSolverTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * ISLAND_DISPATCH_COUNT_OFFSET});
  };

  using TTask_CPS = TaskTemplate<CollisionPreSolverTaskHead::Task, decltype(user_callback_CPS)>;

  // Instantiate the task using the template class
  TTask_CPS task_CPS(std::array{
                       daxa::attachment_view(CollisionPreSolverTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(CollisionPreSolverTaskHead::AT.sim_config, task_sim_config),
                       daxa::attachment_view(CollisionPreSolverTaskHead::AT.rigid_bodies, task_rigid_bodies),
                       daxa::attachment_view(CollisionPreSolverTaskHead::AT.collisions, task_collisions),
                   },
                   user_callback_CPS);

  auto user_callback_CS = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_CS);
    ti.recorder.push_constant(CollisionSolverPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CollisionSolverTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * ISLAND_DISPATCH_COUNT_OFFSET});
  };

  using TTask_CS = TaskTemplate<CollisionSolverTaskHead::Task, decltype(user_callback_CS)>;

  // Instantiate the task using the template class
  TTask_CS task_CS(std::array{
                       daxa::attachment_view(CollisionSolverTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.sim_config, task_sim_config),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.rigid_bodies, task_rigid_bodies),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.collisions, task_collisions),
                   },
                   user_callback_CS);


  auto user_callback_IP = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_IP);
    ti.recorder.push_constant(RigidBodyIntegratePositionsPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IntegratePositionsTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_IP = TaskTemplate<IntegratePositionsTaskHead::Task, decltype(user_callback_IP)>;

  // Instantiate the task using the template class
  TTask_IP task_IP(std::array{
                       daxa::attachment_view(IntegratePositionsTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(IntegratePositionsTaskHead::AT.sim_config, task_sim_config),
                       daxa::attachment_view(IntegratePositionsTaskHead::AT.rigid_bodies, task_rigid_bodies),
                   },
                   user_callback_IP);

  auto user_callback_CSR = [this](daxa::TaskInterface ti, auto &self)
  {
    if(solver_type == SimSolverType::PGS_SOFT) {
      ti.recorder.set_pipeline(*pipeline_CSR);
      ti.recorder.push_constant(CollisionSolverRelaxationPushConstants{.task_head = ti.attachment_shader_blob});
      ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CollisionSolverRelaxationTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * ISLAND_DISPATCH_COUNT_OFFSET});
    }
  };

  using TTask_CSR = TaskTemplate<CollisionSolverRelaxationTaskHead::Task, decltype(user_callback_CSR)>;

  // Instantiate the task using the template class
  TTask_CSR task_CSR(std::array{
                       daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.sim_config, task_sim_config),
                       daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.rigid_bodies, task_rigid_bodies),
                       daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.collisions, task_collisions),
                   },
                   user_callback_CSR);

  auto user_callback_advect = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_advect);
    ti.recorder.push_constant(RigidBodySimPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodySimTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTaskAdvect = TaskTemplate<RigidBodySimTaskHead::Task, decltype(user_callback_advect)>;

  // Instantiate the task using the template class
  TTaskAdvect task_advect(std::array{
                   daxa::attachment_view(RigidBodySimTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                   daxa::attachment_view(RigidBodySimTaskHead::AT.sim_config, task_sim_config),
                   daxa::attachment_view(RigidBodySimTaskHead::AT.rigid_bodies, task_rigid_bodies),
                   daxa::attachment_view(RigidBodySimTaskHead::AT.aabbs, accel_struct_mngr->task_aabb_buffer),
               },
               user_callback_advect);

  auto user_callback_CP = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*create_points_pipeline);
    ti.recorder.push_constant(CreatePointsPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CreatePointsTaskHead::AT.dispatch_buffer).ids[0],
                                   .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTaskCP = TaskTemplate<CreatePointsTaskHead::Task, decltype(user_callback_CP)>;

  // Instantiate the task using the template class
  TTaskCP task_CP(std::array{
                      daxa::attachment_view(CreatePointsTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                      daxa::attachment_view(CreatePointsTaskHead::AT.sim_config, task_sim_config),
                      daxa::attachment_view(CreatePointsTaskHead::AT.collisions, task_collisions),
                      daxa::attachment_view(CreatePointsTaskHead::AT.vertex_buffer, gui->task_vertex_buffer),
                      daxa::attachment_view(CreatePointsTaskHead::AT.line_vertex_buffer, gui->task_line_vertex_buffer),
                  },
                  user_callback_CP);

  auto user_callback_update = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*update_pipeline);
    ti.recorder.push_constant(RigidBodyUpdatePushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodyUpdateTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTaskUpdate = TaskTemplate<RigidBodyUpdateTaskHead::Task, decltype(user_callback_update)>;

  // Instantiate the task using the template class
  TTaskUpdate task_update(std::array{
                              daxa::attachment_view(RigidBodyUpdateTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                              daxa::attachment_view(RigidBodyUpdateTaskHead::AT.sim_config, task_sim_config),
                              daxa::attachment_view(RigidBodyUpdateTaskHead::AT.rigid_bodies, task_rigid_bodies),
                              daxa::attachment_view(RigidBodyUpdateTaskHead::AT.rigid_bodies_update, task_next_rigid_bodies),
                              daxa::attachment_view(RigidBodyUpdateTaskHead::AT.axes_vertex_buffer, gui->task_axes_vertex_buffer),
                          },
                          user_callback_update);

  std::array<daxa::TaskBuffer, 15> buffers = {
      accel_struct_mngr->task_dispatch_buffer,
      task_sim_config,
      task_old_sim_config,
      task_active_rigid_bodies,
      task_scratch_body_links,
      task_body_links,
      task_islands,
      task_rigid_bodies,
      task_next_rigid_bodies,
      accel_struct_mngr->task_aabb_buffer,
      task_collisions,
      task_old_collisions,
      gui->task_vertex_buffer,
      gui->task_line_vertex_buffer,
      gui->task_axes_vertex_buffer,
  };

  RB_TG = task_manager->create_task_graph(name, std::span<daxa::TaskBuffer>(buffers), {}, {}, {});

  RB_TG.add_task(task_RC);
  RB_TG.add_task(task_RBL);
  RB_TG.add_task(task_BP);
  RB_TG.add_task(task_advect);
  RB_TG.add_task(task_IC);
  RB_TG.add_task(task_CS_dispatcher);
  RB_TG.add_task(task_IB);
  RB_TG.add_task(task_IPS);
  RB_TG.add_task(task_IBL);
  RB_TG.add_task(task_CPS);
  for(auto i = 0u; i < iteration_count; ++i)
    RB_TG.add_task(task_CS);
  RB_TG.add_task(task_IP);
  for(auto i = 0u; i < iteration_count; ++i)
    RB_TG.add_task(task_CSR);
  RB_TG.add_task(task_CP);
  RB_TG.add_task(task_update);

  RB_TG.submit();
  RB_TG.complete();

  record_read_back_sim_config_tasks(readback_SC_TG);
  readback_SC_TG.submit();
  readback_SC_TG.complete();

  record_update_sim_config_tasks(update_SC_TG);
  update_SC_TG.submit();
  update_SC_TG.complete();

  record_active_rigid_body_list_upload_tasks(ARB_TG);
  ARB_TG.submit();
  ARB_TG.complete();

  return initialized = true;
}

void RigidBodyManager::record_read_back_sim_config_tasks(TaskGraph &readback_SC_TG)
{
  daxa::InlineTaskInfo task_readback_SC({
      .attachments = {
        daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_old_sim_config),
        daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_sim_config_host),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = ti.get(task_old_sim_config).ids[0],
            .dst_buffer = ti.get(task_sim_config_host).ids[0],
            .size = sizeof(SimConfig),
        });
      },
      .name = "read back sim config",
  });

  std::array<daxa::TaskBuffer, 2> buffers = {
    task_old_sim_config,
    task_sim_config_host,
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
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_sim_config_host),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_sim_config),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = ti.get(task_sim_config_host).ids[0],
            .dst_buffer = ti.get(task_sim_config).ids[0],
            .size = sizeof(SimConfig),
        });
      },
      .name = "update sim config",
  });

  std::array<daxa::TaskBuffer, 2> buffers = {
      task_sim_config,
      task_sim_config_host,
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

  for (auto i = 0u; i < DOUBLE_BUFFERING; ++i) {
    device.destroy_buffer(sim_config_host_buffer[i]);
    device.destroy_buffer(sim_config[i]);
    device.destroy_buffer(collisions[i]);
    device.destroy_buffer(active_rigid_bodies[i]);
    device.destroy_buffer(scratch_body_links[i]);
    device.destroy_buffer(body_links[i]);
    device.destroy_buffer(island_buffer[i]);
  }

  initialized = false;
}

bool RigidBodyManager::is_dirty(){
  return sim_flag_dirty[renderer_manager->get_frame_index()];
}

void RigidBodyManager::clean_dirty(){
  sim_flag_dirty[renderer_manager->get_frame_index()] = false;
}

bool RigidBodyManager::simulate()
{
  if (!initialized)
  {
    return !initialized;
  }
  
  update_buffers();

  RB_TG.execute();

  return initialized;
}

bool RigidBodyManager::update_resources()
{
  if (!initialized)
  {
    return !initialized;
  }

  task_rigid_bodies.set_buffers({.buffers = std::array{accel_struct_mngr->get_rigid_body_buffer()}});

  return initialized;
}

// NOTE: this function reset simulation configuration
bool RigidBodyManager::update_sim()
{
  if (!initialized)
  {
    return !initialized;
  }

  *device.buffer_host_address_as<SimConfig>(sim_config_host_buffer[renderer_manager->get_frame_index()]).value() = SimConfig{
      .solver_type = solver_type,
      .rigid_body_count = renderer_manager->get_rigid_body_count(),
      .active_rigid_body_count = renderer_manager->get_active_rigid_body_count(),
      .island_count = 0,
      .dt = TIME_STEP,
      .gravity = -GRAVITY,
      .flags = sim_flags,
      .frame_count = renderer_manager->get_frame_count(),
      .g_c_info = GlobalCollisionInfo{
          .collision_count = 0,
          .collision_point_count = 0,
      },
  };

  update_buffers();

  update_SC_TG.execute();

  return initialized;
}

bool RigidBodyManager::read_back_sim_config()
{
  if (!initialized)
  {
    return !initialized;
  }

  update_buffers();

  readback_SC_TG.execute();

  return initialized;
}

bool RigidBodyManager::update_active_rigid_body_list()
{
  if (!initialized)
  {
    return !initialized;
  }

  update_buffers();

  ARB_TG.execute();

  return initialized;
}

void RigidBodyManager::update_buffers()
{
  daxa_u32 previous_frame = renderer_manager->get_previous_frame_index();
  daxa_u32 current_frame = renderer_manager->get_frame_index();

  task_sim_config_host.set_buffers({.buffers = std::array{sim_config_host_buffer[current_frame]}});
  task_sim_config.set_buffers({.buffers = std::array{sim_config[current_frame]}});
  task_old_sim_config.set_buffers({.buffers = std::array{sim_config[previous_frame]}});
  task_rigid_bodies.set_buffers({.buffers = std::array{accel_struct_mngr->get_rigid_body_buffer()}});
  task_next_rigid_bodies.set_buffers({.buffers = std::array{accel_struct_mngr->get_next_rigid_body_buffer()}});
  task_collisions.set_buffers({.buffers = std::array{collisions[current_frame]}});
  task_old_collisions.set_buffers({.buffers = std::array{collisions[previous_frame]}});
  task_scratch_body_links.set_buffers({.buffers = std::array{scratch_body_links[current_frame]}});
  task_body_links.set_buffers({.buffers = std::array{body_links[current_frame]}});
  task_islands.set_buffers({.buffers = std::array{island_buffer[current_frame]}});
  task_active_rigid_bodies.set_buffers({.buffers = std::array{active_rigid_bodies[current_frame]}});
}

BB_NAMESPACE_END