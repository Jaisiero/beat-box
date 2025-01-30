#include "rigid_body_manager.hpp"
#include "renderer_manager.hpp"

BB_NAMESPACE_BEGIN

RigidBodyManager::RigidBodyManager(daxa::Device &device,
                                   std::shared_ptr<TaskManager> task_manager,
                                   std::shared_ptr<AccelerationStructureManager> accel_struct_mngr) : device(device), task_manager(task_manager), accel_struct_mngr(accel_struct_mngr)
{
  if (device.is_valid())
  {
    pipeline_RBD = task_manager->create_compute(RigidBodyDispatcherInfo{}.info);
    pipeline_GMC = task_manager->create_compute(GenerateMortonCodesInfo{}.info);
    pipeline_RBRSH = task_manager->create_compute(RigidBodyRadixSortHistogramInfo{}.info);
    pipeline_RBSRS = task_manager->create_compute(RigidBodySingleRadixSortInfo{}.info);
    pipeline_RBLBVHGH = task_manager->create_compute(RigidBodyGenerateHierarchyLinearBVHInfo{}.info);
    pipeline_BBBLBVHGH = task_manager->create_compute(RigidBodyBuildBoundingBoxesLinearBVHInfo{}.info);
    pipeline_RBR = task_manager->create_compute(RigidBodyReorderingInfo{}.info);
    pipeline_RBL = task_manager->create_compute(ResetBodyLinksInfo{}.info);
    pipeline_BP = task_manager->create_compute(BroadPhaseInfo{}.info);
    pipeline_NPD = task_manager->create_compute(NarrowPhaseDispatcherInfo{}.info);
    pipeline_NP = task_manager->create_compute(NarrowPhaseInfo{}.info);
    pipeline_advect = task_manager->create_compute(RigidBodySim{}.info);
    pipeline_IC = task_manager->create_compute(IslandCounterInfo{}.info);
    pipeline_CS_dispatcher = task_manager->create_compute(CollisionSolverDispatcherInfo{}.info);
    pipeline_ID = task_manager->create_compute(IslandDispatcherInfo{}.info);
    pipeline_IB = task_manager->create_compute(IslandBuilderInfo{}.info);
    pipeline_IPS = task_manager->create_compute(IslandPrefixSumInfo{}.info);
    pipeline_IBL = task_manager->create_compute(BodyLink2IslandInfo{}.info);
    pipeline_SBLI = task_manager->create_compute(SortBodyLinksInIslandInfo{}.info);
    pipeline_MIB = task_manager->create_compute(ManifoldIslandBuilderInfo{}.info);
    pipeline_CIG = task_manager->create_compute(ContactIslandGatherInfo{}.info);
    pipeline_CID = task_manager->create_compute(ContactIslandDispatcherInfo{}.info);
    pipeline_MIPS = task_manager->create_compute(ManifoldIslandPrefixSumInfo{}.info);
    pipeline_IML = task_manager->create_compute(ManifoldLink2IslandInfo{}.info);
    pipeline_SMLI = task_manager->create_compute(SortManifoldLinksInIslandInfo{}.info);
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

SimConfig &RigidBodyManager::get_sim_config_reference()
{
  return *device.buffer_host_address_as<SimConfig>(sim_config_host_buffer[renderer_manager->get_frame_index()]).value();
}

daxa::BufferId RigidBodyManager::get_sim_config_host_buffer()
{
  return sim_config_host_buffer[renderer_manager->get_frame_index()];
}

void RigidBodyManager::record_active_rigid_body_list_upload_tasks(TaskGraph &ARBL_TG)
{
  daxa::InlineTaskInfo task_update_active_rigid_bodies({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_active_rigid_bodies),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_rigid_body_entries),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        auto active_rigid_bodies = renderer_manager->get_active_rigid_bodies();

        allocate_fill_copy(ti, active_rigid_bodies, ti.get(task_active_rigid_bodies), 0);

        ti.recorder.clear_buffer({.buffer = ti.get(task_rigid_body_entries).ids[0], .size = sizeof(RigidBodyEntry) * renderer_manager->get_rigid_body_count(), .clear_value = MAX_U32});

      },
      .name = "upload materials",
  });
  std::array<daxa::TaskBuffer, 2> buffers = {
      task_active_rigid_bodies,
      task_rigid_body_entries,
      };
  std::array<daxa::InlineTaskInfo, 1> tasks = {
      task_update_active_rigid_bodies};
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

  for (auto i = 0u; i < DOUBLE_BUFFERING; ++i)
  {
    sim_config_host_buffer[i] = device.create_buffer({
        .size = sizeof(SimConfig),
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "sim_config_host_" + std::to_string(i),
    });
    sim_config[i] = device.create_buffer({
        .size = sizeof(SimConfig),
        .name = "sim_config_" + std::to_string(i),
    });
    // FIXME: think about a better way to handle this
    auto max_number_of_workgroups = (MAX_RIGID_BODY_COUNT + RADIX_SORT_WORKGROUP_SIZE - 1) / RADIX_SORT_WORKGROUP_SIZE;
    global_histograms[i] = device.create_buffer({
        .size = sizeof(daxa_u32) * RADIX_SORT_BINS * max_number_of_workgroups,
        .name = "global_histograms" + std::to_string(i),
    });
    lbvh_nodes[i] = device.create_buffer({
        .size = sizeof(LBVHNode) * MAX_LBVH_NODE_COUNT,
        .name = "lbvh_nodes" + std::to_string(i),
    });
    broad_phase_collisions[i] = device.create_buffer({
        .size = sizeof(BroadPhaseCollision) * MAX_COLLISION_COUNT,
        .name = "broad_phase_collisions" + std::to_string(i),
    });
    collision_entries[i] = device.create_buffer({
        .size = sizeof(CollisionEntry) * MAX_COLLISION_COUNT,
        .name = "collision_entries" + std::to_string(i),
    });
    collisions[i] = device.create_buffer({
        .size = sizeof(Manifold) * MAX_COLLISION_COUNT,
        .name = "collisions" + std::to_string(i),
    });
    rigid_body_entries[i] = device.create_buffer({
        .size = sizeof(RigidBodyEntry) * MAX_RIGID_BODY_COUNT,
        .name = "rigid_body_map" + std::to_string(i),
    });
    active_rigid_bodies[i] = device.create_buffer({
        .size = sizeof(ActiveRigidBody) * MAX_RIGID_BODY_COUNT,
        .name = "active_rigid_bodies" + std::to_string(i),
    });
    rigid_body_link_manifolds[i] = device.create_buffer({
        .size = sizeof(ManifoldNode) * MAX_COLLISION_COUNT,
        .name = "rigid_body_link_manifolds" + std::to_string(i),
    });
    scratch_body_links[i] = device.create_buffer({
        .size = sizeof(BodyLink) * MAX_RIGID_BODY_COUNT,
        .name = "scratch_body_links" + std::to_string(i),
    });
    body_links[i] = device.create_buffer({
        .size = sizeof(BodyLinkIsland) * MAX_RIGID_BODY_COUNT,
        .name = "body_links" + std::to_string(i),
    });
    manifold_links[i] = device.create_buffer({
        .size = sizeof(ManifoldLinkIsland) * MAX_COLLISION_COUNT,
        .name = "manifold_links" + std::to_string(i),
    });
    island_buffer[i] = device.create_buffer({
        .size = sizeof(Island) * MAX_RIGID_BODY_COUNT,
        .name = "islands" + std::to_string(i),
    });
    contact_island_buffer[i] = device.create_buffer({
        .size = sizeof(ContactIsland) * MAX_RIGID_BODY_COUNT,
        .name = "contact_islands" + std::to_string(i),
    });
    *device.buffer_host_address_as<SimConfig>(sim_config_host_buffer[i]).value() = SimConfig{
        .solver_type = renderer_manager->get_solver(),
        .rigid_body_count = 0,
        .active_rigid_body_count = 0,
        .island_count = 0,
        .contact_island_count = 0,
        .manifold_node_count = 0,
        .broad_phase_collision_count = 0,
        .dt = TIME_STEP,
        .gravity = -GRAVITY,
        .flags = sim_flags,
        .frame_count = 0,
        .g_c_info = GlobalCollisionInfo{
            .collision_count = 0,
            .collision_point_count = 0},
    };
  }
  tmp_morton_codes = device.create_buffer({
      .size = sizeof(MortonCode) * MAX_RIGID_BODY_COUNT,
      .name = "tmp_morton_codes",
  });
  morton_codes = device.create_buffer({
      .size = sizeof(MortonCode) * MAX_RIGID_BODY_COUNT,
      .name = "morton_codes",
  });
  lbvh_construction_info = device.create_buffer({
      .size = sizeof(LBVHConstructionInfo) * MAX_LBVH_NODE_COUNT,
      .name = "lbvh_construction_info",
  });
  rigid_body_scratch = device.create_buffer({
      .size = sizeof(RigidBody) * MAX_RIGID_BODY_COUNT,
      .name = "rigid_body_scratch",
  });
  collision_scratch = device.create_buffer({
      .size = sizeof(Manifold) * MAX_COLLISION_COUNT,
      .name = "collision_scratch",
  });

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

        auto contact_island_count = 0;
        auto contact_island_count_offset = sizeof(SimSolverType) + sizeof(daxa_u32) * 3;
        allocate_fill_copy(ti, contact_island_count, ti.get(task_sim_config), contact_island_count_offset);

        auto manifold_node_count = 0;
        auto manifold_node_count_offset = sizeof(SimSolverType) + sizeof(daxa_u32) * 4;
        allocate_fill_copy(ti, manifold_node_count, ti.get(task_sim_config), manifold_node_count_offset);

        auto broad_phase_collision_count = 0;
        auto broad_phase_collision_count_offset = sizeof(SimSolverType) + sizeof(daxa_u32) * 5;
        allocate_fill_copy(ti, broad_phase_collision_count, ti.get(task_sim_config), broad_phase_collision_count_offset);

        shift = 0;
        auto radix_shift_offset = sizeof(SimSolverType) + sizeof(daxa_u32) * 6;
        allocate_fill_copy(ti, shift, ti.get(task_sim_config), radix_shift_offset);

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

  daxa::InlineTaskInfo task_CRB({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_rigid_bodies),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_rigid_body_scratch),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        ti.recorder.copy_buffer_to_buffer({.src_buffer = ti.get(task_rigid_bodies).ids[0], .dst_buffer = ti.get(task_rigid_body_scratch).ids[0], .size = sizeof(RigidBody) * renderer_manager->get_rigid_body_count()});
      },
      .name = "copy rigid bodies",
  });

  
  // Calculate first dispatch count for rigid body dispatcher
  auto user_callback_RBD = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_RBD);
    ti.recorder.push_constant(RigidBodyDispatcherPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };

  using TTask_RBD = TaskTemplate<RigidBodyDispatcherTaskHead::Task, decltype(user_callback_RBD)>;

  // Instantiate the task using the template class
  TTask_RBD task_RBD(std::array{
                         daxa::attachment_view(RigidBodyDispatcherTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(RigidBodyDispatcherTaskHead::AT.sim_config, task_sim_config),
                     },
                     user_callback_RBD);

  auto user_callback_GMC = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_GMC);
    ti.recorder.push_constant(RigidBodyGenerateMortonCodePushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodyGenerateMortonCodeTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_GMC = TaskTemplate<RigidBodyGenerateMortonCodeTaskHead::Task, decltype(user_callback_GMC)>;

  // Instantiate the task using the template class
  TTask_GMC task_GMC(std::array{
                       daxa::attachment_view(RigidBodyGenerateMortonCodeTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(RigidBodyGenerateMortonCodeTaskHead::AT.sim_config, task_sim_config),
                        daxa::attachment_view(RigidBodyGenerateMortonCodeTaskHead::AT.rigid_bodies, task_rigid_body_scratch),
                        daxa::attachment_view(RigidBodyGenerateMortonCodeTaskHead::AT.morton_codes, task_morton_codes),
                   },
                   user_callback_GMC);

  

  auto user_callback_RBSRH = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_RBRSH);
    ti.recorder.push_constant(RigidBodyRadixSortHistogramPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodyRadixSortHistogramTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * RADIX_SORT_RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_RBSRH = TaskTemplate<RigidBodyRadixSortHistogramTaskHead::Task, decltype(user_callback_RBSRH)>;

  // Instantiate the task using the template class
  TTask_RBSRH task_RBSRH(std::array{
                       daxa::attachment_view(RigidBodyRadixSortHistogramTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(RigidBodyRadixSortHistogramTaskHead::AT.sim_config, task_sim_config),
                        daxa::attachment_view(RigidBodyRadixSortHistogramTaskHead::AT.morton_codes, task_morton_codes),
                        daxa::attachment_view(RigidBodyRadixSortHistogramTaskHead::AT.
                        global_histograms, task_radix_sort_histograms),
                   },
                   user_callback_RBSRH);

  TTask_RBSRH task_RBSRH_swap(std::array{
                       daxa::attachment_view(RigidBodyRadixSortHistogramTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(RigidBodyRadixSortHistogramTaskHead::AT.sim_config, task_sim_config),
                        daxa::attachment_view(RigidBodyRadixSortHistogramTaskHead::AT.morton_codes, task_tmp_morton_codes),
                        daxa::attachment_view(RigidBodyRadixSortHistogramTaskHead::AT.
                        global_histograms, task_radix_sort_histograms),
                   },
                   user_callback_RBSRH);

  auto user_callback_RBSRS = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_RBSRS);
    ti.recorder.push_constant(RigidBodySingleRadixSortPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodySingleRadixSortTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * RADIX_SORT_RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_RBSRS = TaskTemplate<RigidBodySingleRadixSortTaskHead::Task, decltype(user_callback_RBSRS)>;

  // Instantiate the task using the template class
  TTask_RBSRS task_RBSRS(std::array{
                       daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.sim_config, task_sim_config),
                        daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.morton_codes_in, task_morton_codes),
                        daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.morton_codes_out, task_tmp_morton_codes),
                        daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.
                        global_histograms, task_radix_sort_histograms),
                   },
                   user_callback_RBSRS);

  // Instantiate the task using the template class
  TTask_RBSRS task_RBSRS_swap(std::array{
                                  daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                                  daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.sim_config, task_sim_config),
                                  daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.morton_codes_in, task_tmp_morton_codes),
                                  daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.morton_codes_out, task_morton_codes),
                                  daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.global_histograms, task_radix_sort_histograms),
                              },
                              user_callback_RBSRS);

  daxa::InlineTaskInfo task_URS({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_sim_config),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        shift += BIT_SHIFT;
        auto radix_shift_offset = sizeof(SimSolverType) + sizeof(daxa_u32) * 6;
        allocate_fill_copy(ti, shift, ti.get(task_sim_config), radix_shift_offset);
      },
      .name = "update radix shift",
  });

  // Task for Generating Hierarchy for Linear Bounding Volume Hierarchy
  auto user_callback_RBLBVHGH = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_RBLBVHGH);
    ti.recorder.push_constant(RigidBodyGenerateHierarchyLinearBVHPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodyGenerateHierarchyLinearBVHTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_RBLBVHGH = TaskTemplate<RigidBodyGenerateHierarchyLinearBVHTaskHead::Task, decltype(user_callback_RBLBVHGH)>;

  // Instantiate the task using the template class
  TTask_RBLBVHGH task_RBLBVHGH(std::array{
                       daxa::attachment_view(RigidBodyGenerateHierarchyLinearBVHTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(RigidBodyGenerateHierarchyLinearBVHTaskHead::AT.sim_config, task_sim_config),
                        daxa::attachment_view(RigidBodyGenerateHierarchyLinearBVHTaskHead::AT.morton_codes, task_morton_codes),
                        daxa::attachment_view(RigidBodyGenerateHierarchyLinearBVHTaskHead::AT.rigid_bodies, task_rigid_body_scratch),
                        daxa::attachment_view(RigidBodyGenerateHierarchyLinearBVHTaskHead::AT.lbvh_nodes, task_lbvh_nodes),
                        daxa::attachment_view(RigidBodyGenerateHierarchyLinearBVHTaskHead::AT.lbvh_construction_info, task_lbvh_construction_info),
                   },
                   user_callback_RBLBVHGH);

  // Task for Building Bounding Boxes for Linear Bounding Volume Hierarchy
  auto user_callback_BBBLBVHGH = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_BBBLBVHGH);
    ti.recorder.push_constant(RigidBodyBuildBoundingBoxesLBVHPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodyBuildBoundingBoxesLinearBVHTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_BBBLBVHGH = TaskTemplate<RigidBodyBuildBoundingBoxesLinearBVHTaskHead::Task, decltype(user_callback_BBBLBVHGH)>;

  // Instantiate the task using the template class
  TTask_BBBLBVHGH task_BBBLBVHGH(std::array{
                       daxa::attachment_view(RigidBodyBuildBoundingBoxesLinearBVHTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(RigidBodyBuildBoundingBoxesLinearBVHTaskHead::AT.sim_config, task_sim_config),
                        daxa::attachment_view(RigidBodyBuildBoundingBoxesLinearBVHTaskHead::AT.lbvh_nodes, task_lbvh_nodes),
                        daxa::attachment_view(RigidBodyBuildBoundingBoxesLinearBVHTaskHead::AT.lbvh_construction_info, task_lbvh_construction_info),
                   },
                   user_callback_BBBLBVHGH);

  // Task for Reordering Rigid Bodies
  auto user_callback_RBR = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_RBR);
    ti.recorder.push_constant(RigidBodyReorderingPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodyReorderingTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_RBR = TaskTemplate<RigidBodyReorderingTaskHead::Task, decltype(user_callback_RBR)>;

  // Instantiate the task using the template class
  TTask_RBR task_RBR(std::array{
                         daxa::attachment_view(RigidBodyReorderingTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(RigidBodyReorderingTaskHead::AT.sim_config, task_sim_config),
                         daxa::attachment_view(RigidBodyReorderingTaskHead::AT.rigid_body_map, task_rigid_body_entries),
                         daxa::attachment_view(RigidBodyReorderingTaskHead::AT.rigid_bodies, task_rigid_body_scratch),
                          daxa::attachment_view(RigidBodyReorderingTaskHead::AT.morton_codes, task_morton_codes),
                          daxa::attachment_view(RigidBodyReorderingTaskHead::AT.lbvh_nodes, task_lbvh_nodes),
                          daxa::attachment_view(RigidBodyReorderingTaskHead::AT.rigid_body_sorted, task_rigid_bodies),
                     },
                     user_callback_RBR);


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
                         daxa::attachment_view(ResetBodyLinkTaskHead::AT.rigid_body_map, task_rigid_body_entries),
                         daxa::attachment_view(ResetBodyLinkTaskHead::AT.rigid_body_map_prev, task_previous_rigid_body_entries),
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
                       daxa::attachment_view(BroadPhaseTaskHead::AT.rigid_bodies, task_rigid_bodies),
                        daxa::attachment_view(BroadPhaseTaskHead::AT.active_rigid_bodies, task_active_rigid_bodies),
                        daxa::attachment_view(BroadPhaseTaskHead::AT.lbvh_nodes, task_lbvh_nodes),
                        daxa::attachment_view(BroadPhaseTaskHead::AT.broad_phase_collisions, task_broad_phase_collisions),
                   },
                   user_callback_BP);

  // Calculate first dispatch count for 
  auto user_callback_NPD = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_NPD);
    ti.recorder.push_constant(NarrowPhaseDispatcherPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };

  using TTask_NPD = TaskTemplate<NarrowPhaseDispatcherTaskHead::Task, decltype(user_callback_NPD)>;

  // Instantiate the task using the template class
  TTask_NPD task_NPD(std::array{
                         daxa::attachment_view(NarrowPhaseDispatcherTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(NarrowPhaseDispatcherTaskHead::AT.sim_config, task_sim_config),
                     },
                     user_callback_NPD);

  auto user_callback_NP = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_NP);
    ti.recorder.push_constant(NarrowPhasePushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(NarrowPhaseTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * NARROW_PHASE_COLLISION_DISPATCH_COUNT_OFFSET});
  };

  using TTask_NP = TaskTemplate<NarrowPhaseTaskHead::Task, decltype(user_callback_NP)>;

  // Instantiate the task using the template class
  TTask_NP task_NP(std::array{
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.sim_config, task_sim_config),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.previous_sim_config, task_old_sim_config),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.broad_phase_collisions, task_broad_phase_collisions),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.rigid_body_map, task_rigid_body_entries),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.rigid_bodies, task_rigid_bodies),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.rigid_body_link_manifolds, task_rigid_body_link_manifolds),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.collision_map, task_collision_entries),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.collisions, task_collision_scratch),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.rigid_body_map_prev, task_previous_rigid_body_entries),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.previous_rigid_bodies, task_previous_rigid_bodies),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.previous_rigid_body_link_manifolds, task_previous_rigid_body_link_manifolds),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.collision_map_prev, task_collision_entries_previous),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.old_collisions, task_old_collisions),
                       daxa::attachment_view(NarrowPhaseTaskHead::AT.scratch_body_links, task_scratch_body_links),
                   },
                   user_callback_NP);

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
                          },
                          user_callback_advect);

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

  auto user_callback_ID = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_ID);
    ti.recorder.push_constant(IslandDispatcherPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };

  using TTask_ID = TaskTemplate<IslandDispatcherTaskHead::Task, decltype(user_callback_ID)>;

  // Instantiate the task using the template class
  TTask_ID task_ID(std::array{
                       daxa::attachment_view(IslandDispatcherTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(IslandDispatcherTaskHead::AT.sim_config, task_sim_config),
                   },
                   user_callback_ID);

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
                       daxa::attachment_view(IslandBuilderTaskHead::AT.active_rigid_bodies, task_active_rigid_bodies),
                       daxa::attachment_view(IslandBuilderTaskHead::AT.rigid_body_map, task_rigid_body_entries),
                       daxa::attachment_view(IslandBuilderTaskHead::AT.rigid_bodies, task_rigid_bodies),
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

  auto user_callback_SBLI = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_SBLI);
    ti.recorder.push_constant(IslandBuilderSortBodyLinkInIslandPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IslandBuilderSortBodyLinkInIslandTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * ISLAND_DISPATCH_COUNT_OFFSET});
  };

  using TTask_SBLI = TaskTemplate<IslandBuilderSortBodyLinkInIslandTaskHead::Task, decltype(user_callback_SBLI)>;

  // Instantiate the task using the template class
  TTask_SBLI task_SBLI(std::array{
                           daxa::attachment_view(IslandBuilderSortBodyLinkInIslandTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                           daxa::attachment_view(IslandBuilderSortBodyLinkInIslandTaskHead::AT.sim_config, task_sim_config),
                           daxa::attachment_view(IslandBuilderSortBodyLinkInIslandTaskHead::AT.islands, task_islands),
                           daxa::attachment_view(IslandBuilderSortBodyLinkInIslandTaskHead::AT.body_links, task_body_links),
                       },
                       user_callback_SBLI);

  auto user_callback_MIB = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_MIB);
    ti.recorder.push_constant(ManifoldIslandBuilderPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(ManifoldIslandBuilderTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * COLLISION_DISPATCH_COUNT_OFFSET});
  };

  using TTask_MIB = TaskTemplate<ManifoldIslandBuilderTaskHead::Task, decltype(user_callback_MIB)>;

  // Instantiate the task using the template class
  TTask_MIB task_MIB(std::array{
                         daxa::attachment_view(ManifoldIslandBuilderTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(ManifoldIslandBuilderTaskHead::AT.sim_config, task_sim_config),
                         daxa::attachment_view(ManifoldIslandBuilderTaskHead::AT.scratch_body_links, task_scratch_body_links),
                         daxa::attachment_view(ManifoldIslandBuilderTaskHead::AT.collisions, task_collision_scratch),
                         daxa::attachment_view(ManifoldIslandBuilderTaskHead::AT.rigid_bodies, task_rigid_bodies),
                         daxa::attachment_view(ManifoldIslandBuilderTaskHead::AT.islands, task_islands),
                     },
                     user_callback_MIB);

  auto user_callback_CGI = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_CIG);
    ti.recorder.push_constant(ContactIslandGatherPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(ContactIslandGatherTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * ISLAND_DISPATCH_COUNT_OFFSET});
  };

  using TTask_CGI = TaskTemplate<ContactIslandGatherTaskHead::Task, decltype(user_callback_CGI)>;

  // Instantiate the task using the template class
  TTask_CGI task_CGI(std::array{
                         daxa::attachment_view(ContactIslandGatherTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(ContactIslandGatherTaskHead::AT.sim_config, task_sim_config),
                         daxa::attachment_view(ContactIslandGatherTaskHead::AT.islands, task_islands),
                         daxa::attachment_view(ContactIslandGatherTaskHead::AT.contact_islands, task_contact_islands),
                     },
                     user_callback_CGI);

  auto user_callback_CID = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_CID);
    ti.recorder.push_constant(ContactIslandDispatcherPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };

  using TTask_CID = TaskTemplate<ContactIslandDispatcherTaskHead::Task, decltype(user_callback_CID)>;

  // Instantiate the task using the template class
  TTask_CID task_CID(std::array{
                         daxa::attachment_view(ContactIslandDispatcherTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(ContactIslandDispatcherTaskHead::AT.sim_config, task_sim_config),
                     },
                     user_callback_CID);

  auto user_callback_MIPS = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_MIPS);
    ti.recorder.push_constant(ManifoldIslandPrefixSumPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };

  using TTask_MIPS = TaskTemplate<ManifoldIslandPrefixSumTaskHead::Task, decltype(user_callback_MIPS)>;

  // Instantiate the task using the template class
  TTask_MIPS task_MIPS(std::array{
                           daxa::attachment_view(ManifoldIslandPrefixSumTaskHead::AT.sim_config, task_sim_config),
                           daxa::attachment_view(ManifoldIslandPrefixSumTaskHead::AT.contact_islands, task_contact_islands),
                           daxa::attachment_view(ManifoldIslandPrefixSumTaskHead::AT.islands, task_islands),
                       },
                       user_callback_MIPS);

  auto user_callback_IML = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_IML);
    ti.recorder.push_constant(IslandBuilderManifoldLink2IslandPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IslandBuilderManifoldLink2IslandTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * COLLISION_DISPATCH_COUNT_OFFSET});
  };

  using TTask_IML = TaskTemplate<IslandBuilderManifoldLink2IslandTaskHead::Task, decltype(user_callback_IML)>;

  // Instantiate the task using the template class
  TTask_IML task_IML(std::array{
                         daxa::attachment_view(IslandBuilderManifoldLink2IslandTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(IslandBuilderManifoldLink2IslandTaskHead::AT.sim_config, task_sim_config),
                         daxa::attachment_view(IslandBuilderManifoldLink2IslandTaskHead::AT.scratch_body_links, task_scratch_body_links),
                         daxa::attachment_view(IslandBuilderManifoldLink2IslandTaskHead::AT.collision_map, task_collision_entries),
                         daxa::attachment_view(IslandBuilderManifoldLink2IslandTaskHead::AT.collisions, task_collision_scratch),
                         daxa::attachment_view(IslandBuilderManifoldLink2IslandTaskHead::AT.rigid_bodies, task_rigid_bodies),
                         daxa::attachment_view(IslandBuilderManifoldLink2IslandTaskHead::AT.islands, task_islands),
                         daxa::attachment_view(IslandBuilderManifoldLink2IslandTaskHead::AT.contact_islands, task_contact_islands),
                         daxa::attachment_view(IslandBuilderManifoldLink2IslandTaskHead::AT.manifold_links, task_manifold_links),
                         daxa::attachment_view(IslandBuilderManifoldLink2IslandTaskHead::AT.collision_sorted, task_collisions),
                     },
                     user_callback_IML);

  auto user_callback_SMLI = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_SMLI);
    ti.recorder.push_constant(IslandBuilderSortManifoldLinkInIslandPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IslandBuilderSortManifoldLinkInIslandTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * CONTACT_ISLAND_DISPATCH_COUNT_OFFSET});
  };

  using TTask_SMLI = TaskTemplate<IslandBuilderSortManifoldLinkInIslandTaskHead::Task, decltype(user_callback_SMLI)>;

  // Instantiate the task using the template class
  TTask_SMLI task_SMLI(std::array{
                           daxa::attachment_view(IslandBuilderSortManifoldLinkInIslandTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                           daxa::attachment_view(IslandBuilderSortManifoldLinkInIslandTaskHead::AT.sim_config, task_sim_config),
                           daxa::attachment_view(IslandBuilderSortManifoldLinkInIslandTaskHead::AT.contact_islands, task_contact_islands),
                           daxa::attachment_view(IslandBuilderSortManifoldLinkInIslandTaskHead::AT.manifold_links, task_manifold_links),
                       },
                       user_callback_SMLI);

  auto user_callback_CPS = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_CPS);
    ti.recorder.push_constant(CollisionPreSolverPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CollisionPreSolverTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * CONTACT_ISLAND_DISPATCH_COUNT_OFFSET});
  };

  using TTask_CPS = TaskTemplate<CollisionPreSolverTaskHead::Task, decltype(user_callback_CPS)>;

  // Instantiate the task using the template class
  TTask_CPS task_CPS(std::array{
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.sim_config, task_sim_config),
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.collision_map, task_collision_entries),
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.rigid_bodies, task_rigid_bodies),
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.collisions, task_collisions),
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.contact_islands, task_contact_islands),
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.manifold_links, task_manifold_links),
                     },
                     user_callback_CPS);

  auto user_callback_CS = [this](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*pipeline_CS);
    ti.recorder.push_constant(CollisionSolverPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CollisionSolverTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * CONTACT_ISLAND_DISPATCH_COUNT_OFFSET});
  };

  using TTask_CS = TaskTemplate<CollisionSolverTaskHead::Task, decltype(user_callback_CS)>;

  // Instantiate the task using the template class
  TTask_CS task_CS(std::array{
                       daxa::attachment_view(CollisionSolverTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.sim_config, task_sim_config),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.collision_map, task_collision_entries),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.rigid_bodies, task_rigid_bodies),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.collisions, task_collisions),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.contact_islands, task_contact_islands),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.manifold_links, task_manifold_links),
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
    if (solver_type == SimSolverType::PGS_SOFT)
    {
      ti.recorder.set_pipeline(*pipeline_CSR);
      ti.recorder.push_constant(CollisionSolverRelaxationPushConstants{.task_head = ti.attachment_shader_blob});
      ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CollisionSolverRelaxationTaskHead::AT.dispatch_buffer).ids[0], .offset = sizeof(daxa_u32vec3) * CONTACT_ISLAND_DISPATCH_COUNT_OFFSET});
    }
  };

  using TTask_CSR = TaskTemplate<CollisionSolverRelaxationTaskHead::Task, decltype(user_callback_CSR)>;

  // Instantiate the task using the template class
  TTask_CSR task_CSR(std::array{
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.sim_config, task_sim_config),
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.collision_map, task_collision_entries),
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.rigid_bodies, task_rigid_bodies),
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.collisions, task_collisions),
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.contact_islands, task_contact_islands),
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.manifold_links,
                                               task_manifold_links),
                     },
                     user_callback_CSR);

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

  std::array<daxa::TaskBuffer, 32> buffers = {
      accel_struct_mngr->task_dispatch_buffer,
      task_sim_config,
      task_old_sim_config,
      task_morton_codes,
      task_tmp_morton_codes,
      task_radix_sort_histograms,
      task_lbvh_nodes,
      task_lbvh_construction_info,
      task_rigid_body_entries,
      task_previous_rigid_body_entries,
      task_active_rigid_bodies,
      task_broad_phase_collisions,
      task_scratch_body_links,
      task_body_links,
      task_manifold_links,
      task_islands,
      task_contact_islands,
      task_previous_rigid_bodies,
      task_rigid_bodies,
      task_next_rigid_bodies,
      task_rigid_body_scratch,
      accel_struct_mngr->task_aabb_buffer,
      task_rigid_body_link_manifolds,
      task_collision_entries,
      task_collision_scratch,
      task_collisions,
      task_previous_rigid_body_link_manifolds,
      task_collision_entries_previous,
      task_old_collisions,
      gui->task_vertex_buffer,
      gui->task_line_vertex_buffer,
      gui->task_axes_vertex_buffer,
  };

  RB_TG = task_manager->create_task_graph(name, std::span<daxa::TaskBuffer>(buffers), {}, {}, {});

  RB_TG.add_task(task_RC);
  RB_TG.add_task(task_CRB);
  RB_TG.add_task(task_RBD);
  RB_TG.add_task(task_GMC);
  for(auto i = 0u; i < ITERATIONS/2; ++i) {
    RB_TG.add_task(task_RBSRH);
    RB_TG.add_task(task_RBSRS);
    RB_TG.add_task(task_URS);
    RB_TG.add_task(task_RBSRH_swap);
    RB_TG.add_task(task_RBSRS_swap);
    RB_TG.add_task(task_URS);
  }
  RB_TG.add_task(task_RBLBVHGH);
  RB_TG.add_task(task_BBBLBVHGH);
  RB_TG.add_task(task_RBR);
  RB_TG.add_task(task_RBL);
  RB_TG.add_task(task_BP);
  RB_TG.add_task(task_NPD);
  RB_TG.add_task(task_NP);
  RB_TG.add_task(task_advect);
  RB_TG.add_task(task_IC);
  RB_TG.add_task(task_CS_dispatcher);
  RB_TG.add_task(task_ID);
  RB_TG.add_task(task_IB);
  RB_TG.add_task(task_IPS);
  RB_TG.add_task(task_IBL);
  RB_TG.add_task(task_SBLI);
  RB_TG.add_task(task_MIB);
  RB_TG.add_task(task_CGI);
  RB_TG.add_task(task_CID);
  RB_TG.add_task(task_MIPS);
  RB_TG.add_task(task_IML);
  RB_TG.add_task(task_SMLI);
  RB_TG.add_task(task_CPS);
  for (auto i = 0u; i < iteration_count; ++i)
    RB_TG.add_task(task_CS);
  RB_TG.add_task(task_IP);
  for (auto i = 0u; i < iteration_count; ++i)
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

  for (auto i = 0u; i < DOUBLE_BUFFERING; ++i)
  {
    device.destroy_buffer(sim_config_host_buffer[i]);
    device.destroy_buffer(sim_config[i]);
    device.destroy_buffer(lbvh_nodes[i]);
    device.destroy_buffer(broad_phase_collisions[i]);
    device.destroy_buffer(collision_entries[i]);
    device.destroy_buffer(collisions[i]);
    device.destroy_buffer(active_rigid_bodies[i]);
    device.destroy_buffer(rigid_body_entries[i]);
    device.destroy_buffer(scratch_body_links[i]);
    device.destroy_buffer(body_links[i]);
    device.destroy_buffer(manifold_links[i]);
    device.destroy_buffer(island_buffer[i]);
    device.destroy_buffer(contact_island_buffer[i]);
    device.destroy_buffer(rigid_body_link_manifolds[i]);
    device.destroy_buffer(global_histograms[i]);
  }
  device.destroy_buffer(tmp_morton_codes);
  device.destroy_buffer(morton_codes);
  device.destroy_buffer(lbvh_construction_info);
  device.destroy_buffer(rigid_body_scratch);
  device.destroy_buffer(collision_scratch);

  initialized = false;
}

bool RigidBodyManager::is_dirty()
{
  return sim_flag_dirty[renderer_manager->get_frame_index()];
}

void RigidBodyManager::clean_dirty()
{
  sim_flag_dirty[renderer_manager->get_frame_index()] = false;
}

daxa::BufferId RigidBodyManager::get_lbvh_node_buffer()
{
  return lbvh_nodes[renderer_manager->get_frame_index()];
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
      .contact_island_count = 0,
      .manifold_node_count = 0,
      .radix_shift = 0,
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
  task_morton_codes.set_buffers({.buffers = std::array{morton_codes}});
  task_tmp_morton_codes.set_buffers({.buffers = std::array{tmp_morton_codes}});
  task_radix_sort_histograms.set_buffers({.buffers = std::array{global_histograms[current_frame]}});
  task_previous_rigid_bodies.set_buffers({.buffers = std::array{accel_struct_mngr->get_previous_rigid_body_buffer()}});
  task_rigid_bodies.set_buffers({.buffers = std::array{accel_struct_mngr->get_rigid_body_buffer()}});
  task_next_rigid_bodies.set_buffers({.buffers = std::array{accel_struct_mngr->get_next_rigid_body_buffer()}});
  task_lbvh_nodes.set_buffers({.buffers = std::array{lbvh_nodes[current_frame]}});
  task_lbvh_construction_info.set_buffers({.buffers = std::array{lbvh_construction_info}});
  task_active_rigid_bodies.set_buffers({.buffers = std::array{active_rigid_bodies[current_frame]}});
  task_rigid_body_entries.set_buffers({.buffers = std::array{rigid_body_entries[current_frame]}});
  task_previous_rigid_body_entries.set_buffers({.buffers = std::array{rigid_body_entries[previous_frame]}});
  task_broad_phase_collisions.set_buffers({.buffers = std::array{broad_phase_collisions[current_frame]}});
  task_rigid_body_scratch.set_buffers({.buffers = std::array{rigid_body_scratch}});
  task_rigid_body_link_manifolds.set_buffers({.buffers = std::array{rigid_body_link_manifolds[current_frame]}});
  task_collision_entries.set_buffers({.buffers = std::array{collision_entries[current_frame]}});
  task_collisions.set_buffers({.buffers = std::array{collisions[current_frame]}});
  task_collision_scratch.set_buffers({.buffers = std::array{collision_scratch}});
  task_previous_rigid_body_link_manifolds.set_buffers({.buffers = std::array{rigid_body_link_manifolds[previous_frame]}});
  task_collision_entries_previous.set_buffers({.buffers = std::array{collision_entries[previous_frame]}});
  task_old_collisions.set_buffers({.buffers = std::array{collisions[previous_frame]}});
  task_scratch_body_links.set_buffers({.buffers = std::array{scratch_body_links[current_frame]}});
  task_body_links.set_buffers({.buffers = std::array{body_links[current_frame]}});
  task_manifold_links.set_buffers({.buffers = std::array{manifold_links[current_frame]}});
  task_islands.set_buffers({.buffers = std::array{island_buffer[current_frame]}});
  task_contact_islands.set_buffers({.buffers = std::array{contact_island_buffer[current_frame]}});
}

BB_NAMESPACE_END