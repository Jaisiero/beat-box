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
    pipeline_CBBLBVHGH = task_manager->create_compute(RigidBodyConvertBoundingBoxesLinearBVHInfo{}.info);
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
    // graph coloring
    pipeline_GCD = task_manager->create_compute(GraphColorDispatcherInfo{}.info);
    pipeline_GCR = task_manager->create_compute(GraphColorResetInfo{}.info);
    pipeline_GCOR = task_manager->create_compute(GraphColorOwnerResetInfo{}.info);
    pipeline_GCP1 = task_manager->create_compute(GraphColorAssignP1Info{}.info);
    pipeline_GCP2 = task_manager->create_compute(GraphColorAssignP2Info{}.info);
    pipeline_GCV = task_manager->create_compute(GraphColorValidateInfo{}.info);
    pipeline_GCV2 = task_manager->create_compute(GraphColorValidate2Info{}.info);
    pipeline_GCS_CPS = task_manager->create_compute(GraphColorPreSolverInfo{}.info);
    pipeline_GCS_CS = task_manager->create_compute(GraphColorSolverInfo{}.info);
    pipeline_GCS_CSR = task_manager->create_compute(GraphColorRelaxInfo{}.info);
    pipeline_GCS_CPS_OV = task_manager->create_compute(GraphColorPreSolverOverflowInfo{}.info);
    pipeline_GCS_CS_OV = task_manager->create_compute(GraphColorSolverOverflowInfo{}.info);
    pipeline_GCS_CSR_OV = task_manager->create_compute(GraphColorRelaxOverflowInfo{}.info);
    pipeline_SLR = task_manager->create_compute(SleepReduceInfo{}.info);
    pipeline_SLV = task_manager->create_compute(SleepVetoInfo{}.info);
    pipeline_SLA = task_manager->create_compute(SleepApplyInfo{}.info);
    pipeline_AVBD_CR = task_manager->create_compute(AvbdColorResetInfo{}.info);
    pipeline_AVBD_CRND = task_manager->create_compute(AvbdColorRoundInfo{}.info);
    pipeline_AVBD_CV = task_manager->create_compute(AvbdColorValidateInfo{}.info);
    pipeline_AVBD_PRE = task_manager->create_compute(AvbdPrepareInfo{}.info);
    pipeline_AVBD_FIN = task_manager->create_compute(AvbdFinalizeInfo{}.info);
    pipeline_AVBD_WS = task_manager->create_compute(AvbdWarmstartInfo{}.info);
    pipeline_AVBD_PRIM = task_manager->create_compute(AvbdPrimalInfo{}.info);
    pipeline_AVBD_DUAL = task_manager->create_compute(AvbdDualInfo{}.info);
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
  return *device.buffer_host_address_as<SimConfig>(sim_config_host_buffer[renderer_manager->get_sim_frame_index()]).value();
}

daxa::BufferId RigidBodyManager::get_sim_config_host_buffer()
{
  return sim_config_host_buffer[renderer_manager->get_sim_frame_index()];
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

        // Initialize rigid_body_entries with identity mapping so the RT shader can
        // access rigid bodies correctly even when simulation is paused.
        // The simulation reordering task will overwrite this when it runs.
        auto rb_count = renderer_manager->get_rigid_body_count();
        std::vector<RigidBodyEntry> identity_entries(rb_count);
        for (u32 i = 0; i < rb_count; ++i) { identity_entries[i].index = i; }
        allocate_fill_copy(ti, identity_entries, ti.get(task_rigid_body_entries), 0);
      },
      .name = "upload rigid body list",
  });
  std::array<daxa::TaskBuffer, 2> buffers = {
      task_active_rigid_bodies,
      task_rigid_body_entries,
      };
  std::array<daxa::InlineTaskInfo, 1> tasks = {
      task_update_active_rigid_bodies};
  ARBL_TG = task_manager->create_task_graph("Active Rigid Body List Upload", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, {}, {}, false, daxa::QUEUE_COMPUTE_0);
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
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
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
        .size = sizeof(ManifoldNode) * BB_MAX_MANIFOLD_NODE_COUNT,
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
    // voxel collision shape data (host-writable: filled once by the scene at load time;
    // static afterwards, addressed through SimConfig - the NP head is at the push limit)
    if (voxel_shapes.is_empty())
    {
      voxel_shapes = device.create_buffer({
          .size = sizeof(VoxelShape) * BB_MAX_VOXEL_SHAPE_COUNT,
          .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
          .name = "voxel_shapes",
      });
      voxel_occupancy = device.create_buffer({
          .size = sizeof(daxa_u32) * BB_MAX_VOXEL_OCC_U32S,
          .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
          .name = "voxel_occupancy",
      });
      voxel_surface = device.create_buffer({
          .size = sizeof(daxa_u32) * BB_MAX_VOXEL_SURF_COUNT,
          .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
          .name = "voxel_surface",
      });
    }
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
        .g_c_info = GlobalCollisionInfo{
            .collision_count = 0,
            .collision_point_count = 0},
        .frame_count = 0,
        .voxel_shapes_addr = device.device_address(voxel_shapes).value(),
        .voxel_occupancy_addr = device.device_address(voxel_occupancy).value(),
        .voxel_surface_addr = device.device_address(voxel_surface).value(),
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
  // graph coloring buffers (raw u32 arrays)
  body_color_mask = device.create_buffer({
      .size = sizeof(daxa_u32) * MAX_RIGID_BODY_COUNT,
      .name = "body_color_mask",
  });
  manifold_color = device.create_buffer({
      .size = sizeof(daxa_u32) * MAX_COLLISION_COUNT,
      .name = "manifold_color",
  });
  body_color_owner = device.create_buffer({
      .size = sizeof(daxa_u32) * MAX_RIGID_BODY_COUNT * BB_MAX_COLORS,
      .name = "body_color_owner",
  });
  color_count = device.create_buffer({
      .size = sizeof(daxa_u32) * (BB_MAX_COLORS + 1),
      .name = "color_count",
  });
  // AVBD buffers
  avbd_state = device.create_buffer({
      .size = sizeof(AvbdBodyState) * MAX_RIGID_BODY_COUNT,
      .name = "avbd_state",
  });
  avbd_body_color = device.create_buffer({
      .size = sizeof(daxa_u32) * MAX_RIGID_BODY_COUNT,
      .name = "avbd_body_color",
  });

  daxa::InlineTaskInfo task_RC({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_sim_config),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        // TODO: simplify this
        // Use offsetof for every field so the per-frame reset stays correct
        // regardless of SimConfig field order/padding (previously these were
        // hand-computed byte offsets that silently break on any reorder).
        auto rigid_body_count = renderer_manager->get_rigid_body_count();
        allocate_fill_copy(ti, rigid_body_count, ti.get(task_sim_config), offsetof(SimConfig, rigid_body_count));

        auto active_rigid_body_count = renderer_manager->get_active_rigid_body_count();
        allocate_fill_copy(ti, active_rigid_body_count, ti.get(task_sim_config), offsetof(SimConfig, active_rigid_body_count));

        auto island_count = 0;
        allocate_fill_copy(ti, island_count, ti.get(task_sim_config), offsetof(SimConfig, island_count));

        auto contact_island_count = 0;
        allocate_fill_copy(ti, contact_island_count, ti.get(task_sim_config), offsetof(SimConfig, contact_island_count));

        auto manifold_node_count = 0;
        allocate_fill_copy(ti, manifold_node_count, ti.get(task_sim_config), offsetof(SimConfig, manifold_node_count));

        auto broad_phase_collision_count = 0;
        allocate_fill_copy(ti, broad_phase_collision_count, ti.get(task_sim_config), offsetof(SimConfig, broad_phase_collision_count));

        auto frame_flags = sim_flags;
        if (suppress_warm_starting_once)
        {
          frame_flags &= ~SimFlag::WARM_STARTING;
          suppress_warm_starting_once = false;
        }
        allocate_fill_copy(ti, frame_flags, ti.get(task_sim_config), offsetof(SimConfig, flags));

        shift = 0;
        allocate_fill_copy(ti, shift, ti.get(task_sim_config), offsetof(SimConfig, radix_shift));

        auto frame_count = renderer_manager->get_frame_count();
        allocate_fill_copy(ti, frame_count, ti.get(task_sim_config), offsetof(SimConfig, frame_count));

        auto reset_c_info = GlobalCollisionInfo{
            .collision_count = 0,
            .collision_point_count = 0,
        };
        allocate_fill_copy(ti, reset_c_info, ti.get(task_sim_config), offsetof(SimConfig, g_c_info));

        // dbg_fresh accumulates in the narrow phase, so its reset must precede it (the
        // graph-coloring stat reset runs between narrow phase and readback and would
        // wipe the value before the CPU ever saw it)
        auto reset_fresh = std::array<daxa_u32, 4>{}; // dbg_fresh, dbg_fresh_tag, dbg_pen, pad
        allocate_fill_copy(ti, reset_fresh, ti.get(task_sim_config), offsetof(SimConfig, dbg_fresh));

        // DIAG: zero the explosion-latch fields once per config buffer (device memory starts
        // undefined; the latch CAS needs a 0 start). Two frames cover both double-buffered configs.
        static daxa_u32 dbg_latch_init_runs = 0;
        if (dbg_latch_init_runs < 2)
        {
          ++dbg_latch_init_runs;
          auto zeroes = std::array<daxa_u32, 8>{}; // dbg_ex_stage..dbg_id_sum are contiguous
          allocate_fill_copy(ti, zeroes, ti.get(task_sim_config), offsetof(SimConfig, dbg_ex_stage));
        }
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
        ti.recorder.copy_buffer_to_buffer({.src_buffer = ti.get(task_rigid_bodies).id, .dst_buffer = ti.get(task_rigid_body_scratch).id, .size = sizeof(RigidBody) * renderer_manager->get_rigid_body_count()});
      },
      .name = "copy rigid bodies",
  });

  
  // Calculate first dispatch count for rigid body dispatcher
  auto user_callback_RBD = [this](daxa::TaskInterface ti, auto &)
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

  auto user_callback_GMC = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_GMC);
    ti.recorder.push_constant(RigidBodyGenerateMortonCodePushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodyGenerateMortonCodeTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
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

  

  auto user_callback_RBSRH = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_RBRSH);
    ti.recorder.push_constant(RigidBodyRadixSortHistogramPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodyRadixSortHistogramTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * RADIX_SORT_RIGID_BODY_DISPATCH_COUNT_OFFSET});
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

  auto user_callback_RBSRS = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_RBSRS);
    ti.recorder.push_constant(RigidBodySingleRadixSortPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodySingleRadixSortTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * RADIX_SORT_RIGID_BODY_DISPATCH_COUNT_OFFSET});
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
        allocate_fill_copy(ti, shift, ti.get(task_sim_config), offsetof(SimConfig, radix_shift));
      },
      .name = "update radix shift",
  });

  // Task for Generating Hierarchy for Linear Bounding Volume Hierarchy
  auto user_callback_RBLBVHGH = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_RBLBVHGH);
    ti.recorder.push_constant(RigidBodyGenerateHierarchyLinearBVHPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodyGenerateHierarchyLinearBVHTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
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
  auto user_callback_BBBLBVHGH = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_BBBLBVHGH);
    ti.recorder.push_constant(RigidBodyBuildBoundingBoxesLBVHPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodyBuildBoundingBoxesLinearBVHTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
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

  // Task for Converting integer-mapped LBVH bounds back into float AABBs (internal nodes)
  auto user_callback_CBBLBVHGH = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_CBBLBVHGH);
    ti.recorder.push_constant(RigidBodyConvertBoundingBoxesLBVHPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodyConvertBoundingBoxesLinearBVHTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_CBBLBVHGH = TaskTemplate<RigidBodyConvertBoundingBoxesLinearBVHTaskHead::Task, decltype(user_callback_CBBLBVHGH)>;

  TTask_CBBLBVHGH task_CBBLBVHGH(std::array{
                       daxa::attachment_view(RigidBodyConvertBoundingBoxesLinearBVHTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(RigidBodyConvertBoundingBoxesLinearBVHTaskHead::AT.sim_config, task_sim_config),
                        daxa::attachment_view(RigidBodyConvertBoundingBoxesLinearBVHTaskHead::AT.lbvh_nodes, task_lbvh_nodes),
                        daxa::attachment_view(RigidBodyConvertBoundingBoxesLinearBVHTaskHead::AT.lbvh_construction_info, task_lbvh_construction_info),
                   },
                   user_callback_CBBLBVHGH);

  // Task for Reordering Rigid Bodies
  auto user_callback_RBR = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_RBR);
    ti.recorder.push_constant(RigidBodyReorderingPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodyReorderingTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
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
  auto user_callback_RBL = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_RBL);
    ti.recorder.push_constant(ResetBodyLinkPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(ResetBodyLinkTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
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

  auto user_callback_BP = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_BP);
    ti.recorder.push_constant(BroadPhasePushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(BroadPhaseTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
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
  auto user_callback_NPD = [this](daxa::TaskInterface ti, auto &)
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

  auto user_callback_NP = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_NP);
    ti.recorder.push_constant(NarrowPhasePushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(NarrowPhaseTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * NARROW_PHASE_COLLISION_DISPATCH_COUNT_OFFSET});
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

  auto user_callback_advect = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_advect);
    ti.recorder.push_constant(RigidBodySimPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodySimTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTaskAdvect = TaskTemplate<RigidBodySimTaskHead::Task, decltype(user_callback_advect)>;

  // Instantiate the task using the template class
  TTaskAdvect task_advect(std::array{
                              daxa::attachment_view(RigidBodySimTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                              daxa::attachment_view(RigidBodySimTaskHead::AT.sim_config, task_sim_config),
                              daxa::attachment_view(RigidBodySimTaskHead::AT.rigid_bodies, task_rigid_bodies),
                          },
                          user_callback_advect);

  auto user_callback_IC = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_IC);
    ti.recorder.push_constant(IslandCounterPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IslandCounterTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
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

  auto user_callback_CS_dispatcher = [this](daxa::TaskInterface ti, auto &)
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

  auto user_callback_ID = [this](daxa::TaskInterface ti, auto &)
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

  auto user_callback_IB = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_IB);
    ti.recorder.push_constant(IslandBuilderPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IslandBuilderTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
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

  auto user_callback_IPS = [this](daxa::TaskInterface ti, auto &)
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

  auto user_callback_IBL = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_IBL);
    ti.recorder.push_constant(IslandBuilderBodyLink2IslandPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IslandBuilderBodyLink2IslandTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
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

  auto user_callback_SBLI = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_SBLI);
    ti.recorder.push_constant(IslandBuilderSortBodyLinkInIslandPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IslandBuilderSortBodyLinkInIslandTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * ISLAND_DISPATCH_COUNT_OFFSET});
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

  auto user_callback_MIB = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_MIB);
    ti.recorder.push_constant(ManifoldIslandBuilderPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(ManifoldIslandBuilderTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * COLLISION_DISPATCH_COUNT_OFFSET});
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

  auto user_callback_CGI = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_CIG);
    ti.recorder.push_constant(ContactIslandGatherPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(ContactIslandGatherTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * ISLAND_DISPATCH_COUNT_OFFSET});
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

  auto user_callback_CID = [this](daxa::TaskInterface ti, auto &)
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

  auto user_callback_MIPS = [this](daxa::TaskInterface ti, auto &)
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

  auto user_callback_IML = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_IML);
    ti.recorder.push_constant(IslandBuilderManifoldLink2IslandPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IslandBuilderManifoldLink2IslandTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * COLLISION_DISPATCH_COUNT_OFFSET});
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

  auto user_callback_SMLI = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_SMLI);
    ti.recorder.push_constant(IslandBuilderSortManifoldLinkInIslandPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IslandBuilderSortManifoldLinkInIslandTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * CONTACT_ISLAND_DISPATCH_COUNT_OFFSET});
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

  auto user_callback_CPS = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_CPS);
    ti.recorder.push_constant(CollisionPreSolverPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CollisionPreSolverTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * CONTACT_ISLAND_DISPATCH_COUNT_OFFSET});
  };

  using TTask_CPS = TaskTemplate<CollisionPreSolverTaskHead::Task, decltype(user_callback_CPS)>;

  // Instantiate the task using the template class
  TTask_CPS task_CPS(std::array{
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.sim_config, task_sim_config),
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.rigid_bodies, task_rigid_bodies),
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.collision_map, task_collision_entries),
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.collisions, task_collisions),
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.contact_islands, task_contact_islands),
                         daxa::attachment_view(CollisionPreSolverTaskHead::AT.manifold_links, task_manifold_links),
                     },
                     user_callback_CPS);

  auto user_callback_CS = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_CS);
    ti.recorder.push_constant(CollisionSolverPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CollisionSolverTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * CONTACT_ISLAND_DISPATCH_COUNT_OFFSET});
  };

  using TTask_CS = TaskTemplate<CollisionSolverTaskHead::Task, decltype(user_callback_CS)>;

  // Instantiate the task using the template class
  TTask_CS task_CS(std::array{
                       daxa::attachment_view(CollisionSolverTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.sim_config, task_sim_config),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.rigid_bodies, task_rigid_bodies),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.collision_map, task_collision_entries),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.collisions, task_collisions),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.contact_islands, task_contact_islands),
                       daxa::attachment_view(CollisionSolverTaskHead::AT.manifold_links, task_manifold_links),
                   },
                   user_callback_CS);

  auto user_callback_IP = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_IP);
    ti.recorder.push_constant(RigidBodyIntegratePositionsPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IntegratePositionsTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };

  using TTask_IP = TaskTemplate<IntegratePositionsTaskHead::Task, decltype(user_callback_IP)>;

  // Instantiate the task using the template class
  TTask_IP task_IP(std::array{
                       daxa::attachment_view(IntegratePositionsTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(IntegratePositionsTaskHead::AT.sim_config, task_sim_config),
                       daxa::attachment_view(IntegratePositionsTaskHead::AT.rigid_bodies, task_rigid_bodies),
                   },
                   user_callback_IP);

  auto user_callback_CSR = [this](daxa::TaskInterface ti, auto &)
  {
    if (solver_type == SimSolverType::PGS_SOFT)
    {
      ti.recorder.set_pipeline(*pipeline_CSR);
      ti.recorder.push_constant(CollisionSolverRelaxationPushConstants{.task_head = ti.attachment_shader_blob});
      ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CollisionSolverRelaxationTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * CONTACT_ISLAND_DISPATCH_COUNT_OFFSET});
    }
  };

  using TTask_CSR = TaskTemplate<CollisionSolverRelaxationTaskHead::Task, decltype(user_callback_CSR)>;

  // Instantiate the task using the template class
  TTask_CSR task_CSR(std::array{
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.sim_config, task_sim_config),
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.rigid_bodies, task_rigid_bodies),
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.collision_map, task_collision_entries),
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.collisions, task_collisions),
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.contact_islands, task_contact_islands),
                         daxa::attachment_view(CollisionSolverRelaxationTaskHead::AT.manifold_links,
                                               task_manifold_links),
                     },
                     user_callback_CSR);

  auto user_callback_CP = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*create_points_pipeline);
    ti.recorder.push_constant(CreatePointsPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(CreatePointsTaskHead::AT.dispatch_buffer).id,
                                   .offset = sizeof(daxa_u32vec3) * COLLISION_DISPATCH_COUNT_OFFSET});
  };

  using TTaskCP = TaskTemplate<CreatePointsTaskHead::Task, decltype(user_callback_CP)>;

  // Instantiate the task using the template class
  TTaskCP task_CP(std::array{
                      daxa::attachment_view(CreatePointsTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                      daxa::attachment_view(CreatePointsTaskHead::AT.sim_config, task_sim_config),
                      daxa::attachment_view(CreatePointsTaskHead::AT.collisions, task_collisions),
                      daxa::attachment_view(CreatePointsTaskHead::AT.manifold_color, task_manifold_color),
                      daxa::attachment_view(CreatePointsTaskHead::AT.vertex_buffer, gui->task_vertex_buffer),
                      daxa::attachment_view(CreatePointsTaskHead::AT.line_vertex_buffer, gui->task_line_vertex_buffer),
                  },
                  user_callback_CP);

  auto user_callback_update = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*update_pipeline);
    ti.recorder.push_constant(RigidBodyUpdatePushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodyUpdateTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
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

  std::array<daxa::TaskBuffer, 38> buffers = {
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
      task_body_color_mask,
      task_manifold_color,
      task_body_color_owner,
      task_color_count,
      task_avbd_state,
      task_avbd_body_color,
  };

  // the whole sim runs on the async compute queue; the render graph waits the sim timeline
  RB_TG = task_manager->create_task_graph(name, std::span<daxa::TaskBuffer>(buffers), {}, {}, {}, false, daxa::QUEUE_COMPUTE_0);

  // ---- graph coloring tasks (Phase 2: color + validate, coexisting with the island solver) ----
  // Contacts are an EDGE coloring: colors needed ~= max body degree (Vizing), and one round commits at
  // most one contact per (body,color), so rounds needed ~= max degree too. Transient pile-compression
  // spikes reach degree ~24-30, so run the full mask capacity; converged frames early-out per thread.
  static const daxa_u32 GRAPH_COLOR_MAX_ROUNDS = BB_MAX_COLORS;

  auto user_callback_GCD = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_GCD);
    ti.recorder.push_constant(RigidBodyDispatcherPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };
  using TTask_GCD = TaskTemplate<RigidBodyDispatcherTaskHead::Task, decltype(user_callback_GCD)>;
  TTask_GCD task_GCD(std::array{
                         daxa::attachment_view(RigidBodyDispatcherTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(RigidBodyDispatcherTaskHead::AT.sim_config, task_sim_config),
                     },
                     user_callback_GCD);

  // the 5 coloring passes share GraphColorTaskHead bindings and dispatch over graph_color_dispatch
  auto gc_views = std::array{
      daxa::attachment_view(GraphColorTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
      daxa::attachment_view(GraphColorTaskHead::AT.sim_config, task_sim_config),
      daxa::attachment_view(GraphColorTaskHead::AT.collisions, task_collisions),
      daxa::attachment_view(GraphColorTaskHead::AT.rigid_bodies, task_rigid_bodies),
      daxa::attachment_view(GraphColorTaskHead::AT.body_color_mask, task_body_color_mask),
      daxa::attachment_view(GraphColorTaskHead::AT.manifold_color, task_manifold_color),
      daxa::attachment_view(GraphColorTaskHead::AT.body_color_owner, task_body_color_owner),
      daxa::attachment_view(GraphColorTaskHead::AT.color_count, task_color_count),
  };
  auto gc_dispatch = [this](daxa::TaskInterface ti, std::shared_ptr<daxa::ComputePipeline> &pl)
  {
    ti.recorder.set_pipeline(*pl);
    ti.recorder.push_constant(GraphColorPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(GraphColorTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * GRAPH_COLOR_DISPATCH_COUNT_OFFSET});
  };

  auto user_callback_GCR = [this, gc_dispatch](daxa::TaskInterface ti, auto &) { gc_dispatch(ti, pipeline_GCR); };
  using TTask_GCR = TaskTemplate<GraphColorTaskHead::Task, decltype(user_callback_GCR)>;
  TTask_GCR task_GCR(gc_views, user_callback_GCR);

  auto user_callback_GCOR = [this, gc_dispatch](daxa::TaskInterface ti, auto &) { gc_dispatch(ti, pipeline_GCOR); };
  using TTask_GCOR = TaskTemplate<GraphColorTaskHead::Task, decltype(user_callback_GCOR)>;
  TTask_GCOR task_GCOR(gc_views, user_callback_GCOR);

  auto user_callback_GCP1 = [this, gc_dispatch](daxa::TaskInterface ti, auto &) { gc_dispatch(ti, pipeline_GCP1); };
  using TTask_GCP1 = TaskTemplate<GraphColorTaskHead::Task, decltype(user_callback_GCP1)>;
  TTask_GCP1 task_GCP1(gc_views, user_callback_GCP1);

  auto user_callback_GCP2 = [this, gc_dispatch](daxa::TaskInterface ti, auto &) { gc_dispatch(ti, pipeline_GCP2); };
  using TTask_GCP2 = TaskTemplate<GraphColorTaskHead::Task, decltype(user_callback_GCP2)>;
  TTask_GCP2 task_GCP2(gc_views, user_callback_GCP2);

  auto user_callback_GCV = [this, gc_dispatch](daxa::TaskInterface ti, auto &) { gc_dispatch(ti, pipeline_GCV); };
  using TTask_GCV = TaskTemplate<GraphColorTaskHead::Task, decltype(user_callback_GCV)>;
  TTask_GCV task_GCV(gc_views, user_callback_GCV);

  auto user_callback_GCV2 = [this, gc_dispatch](daxa::TaskInterface ti, auto &) { gc_dispatch(ti, pipeline_GCV2); };
  using TTask_GCV2 = TaskTemplate<GraphColorTaskHead::Task, decltype(user_callback_GCV2)>;
  TTask_GCV2 task_GCV2(gc_views, user_callback_GCV2);

  // ---- neighborhood sleeping tasks (see entry_sleep_reduce/veto/apply) ----
  auto sleep_views = std::array{
      daxa::attachment_view(SleepTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
      daxa::attachment_view(SleepTaskHead::AT.sim_config, task_sim_config),
      daxa::attachment_view(SleepTaskHead::AT.rigid_bodies, task_rigid_bodies),
      daxa::attachment_view(SleepTaskHead::AT.collisions, task_collision_scratch),
  };
  auto sleep_dispatch = [this](daxa::TaskInterface ti, std::shared_ptr<daxa::ComputePipeline> &pl, daxa_u32 dispatch_offset)
  {
    ti.recorder.set_pipeline(*pl);
    ti.recorder.push_constant(SleepPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(SleepTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * dispatch_offset});
  };
  auto user_callback_SLR = [this, sleep_dispatch](daxa::TaskInterface ti, auto &) { sleep_dispatch(ti, pipeline_SLR, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  using TTask_SLR = TaskTemplate<SleepTaskHead::Task, decltype(user_callback_SLR)>;
  TTask_SLR task_SLR(sleep_views, user_callback_SLR);

  auto user_callback_SLV = [this, sleep_dispatch](daxa::TaskInterface ti, auto &) { sleep_dispatch(ti, pipeline_SLV, COLLISION_DISPATCH_COUNT_OFFSET); };
  using TTask_SLV = TaskTemplate<SleepTaskHead::Task, decltype(user_callback_SLV)>;
  TTask_SLV task_SLV(sleep_views, user_callback_SLV);

  auto user_callback_SLA = [this, sleep_dispatch](daxa::TaskInterface ti, auto &) { sleep_dispatch(ti, pipeline_SLA, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  using TTask_SLA = TaskTemplate<SleepTaskHead::Task, decltype(user_callback_SLA)>;
  TTask_SLA task_SLA(sleep_views, user_callback_SLA);

  // ---- AVBD tasks (F1: body vertex coloring + validator; runs after IML so manifold lists,
  // collision_map and the packed manifolds are final for this step) ----
  auto avbd_views = std::array{
      daxa::attachment_view(AvbdTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
      daxa::attachment_view(AvbdTaskHead::AT.sim_config, task_sim_config),
      daxa::attachment_view(AvbdTaskHead::AT.rigid_bodies, task_rigid_bodies),
      daxa::attachment_view(AvbdTaskHead::AT.collisions, task_collisions),
      daxa::attachment_view(AvbdTaskHead::AT.manifold_nodes, task_rigid_body_link_manifolds),
      daxa::attachment_view(AvbdTaskHead::AT.collision_map, task_collision_entries),
      daxa::attachment_view(AvbdTaskHead::AT.avbd_state, task_avbd_state),
      daxa::attachment_view(AvbdTaskHead::AT.body_color, task_avbd_body_color),
  };
  auto avbd_dispatch = [this](daxa::TaskInterface ti, std::shared_ptr<daxa::ComputePipeline> &pl, daxa_u32 pc_color, daxa_f32 stab_alpha, daxa_u32 dispatch_offset)
  {
    ti.recorder.set_pipeline(*pl);
    ti.recorder.push_constant(AvbdPushConstants{.task_head = ti.attachment_shader_blob, .color = pc_color, .stab_alpha = stab_alpha});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(AvbdTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * dispatch_offset});
  };
  auto user_callback_AVBD_CR = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_CR, 0u, 1.0f, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_CR = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_CR)>;
  TTask_AVBD_CR task_AVBD_CR(avbd_views, user_callback_AVBD_CR);

  auto make_avbd_round = [this, avbd_dispatch](daxa_u32 round)
  {
    return [this, avbd_dispatch, round](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_CRND, round, 1.0f, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  };
  using TTask_AVBD_CRND = TaskTemplate<AvbdTaskHead::Task, decltype(make_avbd_round(0u))>;
  std::vector<TTask_AVBD_CRND> task_AVBD_CRND_vec;
  task_AVBD_CRND_vec.reserve(BB_AVBD_COLOR_ROUNDS);
  for (daxa_u32 rd = 0u; rd < BB_AVBD_COLOR_ROUNDS; ++rd)
  {
    task_AVBD_CRND_vec.emplace_back(avbd_views, make_avbd_round(rd));
  }

  auto user_callback_AVBD_CV = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_CV, 0u, 1.0f, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_CV = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_CV)>;
  TTask_AVBD_CV task_AVBD_CV(avbd_views, user_callback_AVBD_CV);

  auto user_callback_AVBD_PRE = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_PRE, 0u, 1.0f, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_PRE = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_PRE)>;
  TTask_AVBD_PRE task_AVBD_PRE(avbd_views, user_callback_AVBD_PRE);

  auto user_callback_AVBD_FIN = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_FIN, 0u, 1.0f, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_FIN = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_FIN)>;
  TTask_AVBD_FIN task_AVBD_FIN(avbd_views, user_callback_AVBD_FIN);

  auto user_callback_AVBD_WS = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_WS, 0u, 1.0f, COLLISION_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_WS = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_WS)>;
  TTask_AVBD_WS task_AVBD_WS(avbd_views, user_callback_AVBD_WS);

  auto make_avbd_primal = [this, avbd_dispatch](daxa_u32 c, daxa_f32 stab_alpha)
  {
    return [this, avbd_dispatch, c, stab_alpha](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_PRIM, c, stab_alpha, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  };
  using TTask_AVBD_PRIM = TaskTemplate<AvbdTaskHead::Task, decltype(make_avbd_primal(0u, 1.0f))>;
  std::vector<TTask_AVBD_PRIM> task_AVBD_PRIM_vec;     // main sweeps: alpha = 1 (delta-only constraint)
  std::vector<TTask_AVBD_PRIM> task_AVBD_PRIM_PS_vec;  // post-stabilization sweep: alpha = 0 (full C0)
  task_AVBD_PRIM_vec.reserve(BB_AVBD_MAX_BODY_COLORS);
  task_AVBD_PRIM_PS_vec.reserve(BB_AVBD_MAX_BODY_COLORS);
  for (daxa_u32 c = 0u; c < BB_AVBD_MAX_BODY_COLORS; ++c)
  {
    task_AVBD_PRIM_vec.emplace_back(avbd_views, make_avbd_primal(c, 1.0f));
    task_AVBD_PRIM_PS_vec.emplace_back(avbd_views, make_avbd_primal(c, 0.0f));
  }

  auto user_callback_AVBD_DUAL = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_DUAL, 0u, 1.0f, COLLISION_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_DUAL = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_DUAL)>;
  TTask_AVBD_DUAL task_AVBD_DUAL(avbd_views, user_callback_AVBD_DUAL);

  // ---- per-color solver tasks (Phase 3): one dispatch per color, each filters manifold_color==color ----
  static const daxa_u32 MAX_COLORS_SOLVE = BB_MAX_COLORS_SOLVE; // shared.inl: per-color solver dispatch count; empty colors are cheap no-ops
  auto gc_solve_views = std::array{
      daxa::attachment_view(GraphColorSolveTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
      daxa::attachment_view(GraphColorSolveTaskHead::AT.sim_config, task_sim_config),
      daxa::attachment_view(GraphColorSolveTaskHead::AT.collisions, task_collisions),
      daxa::attachment_view(GraphColorSolveTaskHead::AT.rigid_bodies, task_rigid_bodies),
      daxa::attachment_view(GraphColorSolveTaskHead::AT.manifold_color, task_manifold_color),
  };
  auto make_gcs = [this](std::shared_ptr<daxa::ComputePipeline> pl, daxa_u32 c) {
    return [this, pl, c](daxa::TaskInterface ti, auto &) {
      ti.recorder.set_pipeline(*pl);
      ti.recorder.push_constant(GraphColorSolvePushConstants{.task_head = ti.attachment_shader_blob, .color = c});
      ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(GraphColorSolveTaskHead::AT.dispatch_buffer).id,
                                     .offset = sizeof(daxa_u32vec3) * COLLISION_DISPATCH_COUNT_OFFSET});
    };
  };
  using TTask_GCS = TaskTemplate<GraphColorSolveTaskHead::Task, decltype(make_gcs(pipeline_GCS_CS, 0u))>;
  std::vector<TTask_GCS> task_GCS_CPS_vec, task_GCS_CS_vec, task_GCS_CSR_vec;
  task_GCS_CPS_vec.reserve(MAX_COLORS_SOLVE);
  task_GCS_CS_vec.reserve(MAX_COLORS_SOLVE);
  task_GCS_CSR_vec.reserve(MAX_COLORS_SOLVE);
  for (daxa_u32 c = 0u; c < MAX_COLORS_SOLVE; ++c)
  {
    task_GCS_CPS_vec.emplace_back(gc_solve_views, make_gcs(pipeline_GCS_CPS, c));
    task_GCS_CS_vec.emplace_back(gc_solve_views, make_gcs(pipeline_GCS_CS, c));
    task_GCS_CSR_vec.emplace_back(gc_solve_views, make_gcs(pipeline_GCS_CSR, c));
  }

  // overflow bucket: serial single-thread solve of manifolds the per-color dispatches skip
  // (uncolored / color>=MAX_COLORS_SOLVE). Early-outs on graph_color_overflow==0, so it is
  // free except on degenerate frames.
  auto make_gcs_ov = [this](std::shared_ptr<daxa::ComputePipeline> pl) {
    return [this, pl](daxa::TaskInterface ti, auto &) {
      ti.recorder.set_pipeline(*pl);
      ti.recorder.push_constant(GraphColorSolvePushConstants{.task_head = ti.attachment_shader_blob, .color = 0u});
      ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
    };
  };
  using TTask_GCS_OV = TaskTemplate<GraphColorSolveTaskHead::Task, decltype(make_gcs_ov(pipeline_GCS_CS_OV))>;
  TTask_GCS_OV task_GCS_CPS_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CPS_OV));
  TTask_GCS_OV task_GCS_CS_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CS_OV));
  TTask_GCS_OV task_GCS_CSR_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CSR_OV));

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
  RB_TG.add_task(task_CBBLBVHGH);
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
  // neighborhood sleeping: decide sleep/wake BEFORE the solve so sleeping bodies skip it this step
  RB_TG.add_task(task_SLR);
  RB_TG.add_task(task_SLV);
  RB_TG.add_task(task_SLA);
  // FIXME: that's a really expensive sort
  // RB_TG.add_task(task_SBLI);
  RB_TG.add_task(task_MIB);
  RB_TG.add_task(task_CGI);
  RB_TG.add_task(task_CID);
  RB_TG.add_task(task_MIPS);
  RB_TG.add_task(task_IML);
  // FIXME: that's a really expensive sort too
  // RB_TG.add_task(task_SMLI);
  // graph coloring. MUST run after task_IML: the coloring, the validator AND the per-color solver
  // all index `task_collisions`, which task_IML (re)writes in island-sorted order THIS frame.
  // Coloring earlier (e.g. right after the narrow phase) reads the previous content of the buffer
  // (2 frames old under double buffering), so the colors describe a different manifold->body
  // mapping than the one the per-color solver dispatches over. Two same-body manifolds can then
  // land in one color and solve concurrently -> racy Jacobi-style overcorrection -> a resting
  // body gets ejected at thousands of m/s within one CS sweep (and the validator stays at 0
  // violations because it validated the SAME stale data the colorer saw).
  RB_TG.add_task(task_GCD);
  RB_TG.add_task(task_GCR);
  for (auto r = 0u; r < GRAPH_COLOR_MAX_ROUNDS; ++r)
  {
    RB_TG.add_task(task_GCOR);
    RB_TG.add_task(task_GCP1);
    RB_TG.add_task(task_GCP2);
  }
  RB_TG.add_task(task_GCOR); // reset owner-as-seen for the validator
  RB_TG.add_task(task_GCV);
  RB_TG.add_task(task_GCV2); // TEMP diag: satbody degree/partners
  // AVBD body coloring (independent of the contact coloring above; entries are cheap and the
  // primal solve will only run when solver_type == AVBD)
  RB_TG.add_task(task_AVBD_CR);
  for (daxa_u32 rd = 0u; rd < BB_AVBD_COLOR_ROUNDS; ++rd)
  {
    RB_TG.add_task(task_AVBD_CRND_vec[rd]);
  }
  RB_TG.add_task(task_AVBD_CV);
  RB_TG.add_task(task_AVBD_PRE); // AVBD: save step-start pose + jump to the inertial target
  RB_TG.add_task(task_AVBD_WS);  // AVBD: lambda/k warm-start scaling
  for (daxa_u32 it = 0u; it < BB_AVBD_ITERATIONS; ++it)
  {
    for (daxa_u32 c = 0u; c < BB_AVBD_MAX_BODY_COLORS; ++c)
    {
      RB_TG.add_task(task_AVBD_PRIM_vec[c]);
    }
    RB_TG.add_task(task_AVBD_DUAL);
  }
  if (static_cast<daxa_u32>(sim_flags & SimFlag::USE_GRAPH_COLORING) != 0u)
  {
    // per-color solve (parallel: one dispatch per color, balanced, no atomics)
    // + serial overflow bucket after each sweep (uncolored / color>=MAX_COLORS_SOLVE manifolds)
    for (daxa_u32 c = 0u; c < MAX_COLORS_SOLVE; ++c)
      RB_TG.add_task(task_GCS_CPS_vec[c]);
    RB_TG.add_task(task_GCS_CPS_OV);
    for (auto i = 0u; i < iteration_count; ++i)
    {
      for (daxa_u32 c = 0u; c < MAX_COLORS_SOLVE; ++c)
        RB_TG.add_task(task_GCS_CS_vec[c]);
      RB_TG.add_task(task_GCS_CS_OV);
    }
    RB_TG.add_task(task_IP);
    for (auto i = 0u; i < iteration_count; ++i)
    {
      for (daxa_u32 c = 0u; c < MAX_COLORS_SOLVE; ++c)
        RB_TG.add_task(task_GCS_CSR_vec[c]);
      RB_TG.add_task(task_GCS_CSR_OV);
    }
  }
  else
  {
    // per-island solve (serial within each contact island)
    RB_TG.add_task(task_CPS);
    for (auto i = 0u; i < iteration_count; ++i)
      RB_TG.add_task(task_CS);
    RB_TG.add_task(task_IP);
    for (auto i = 0u; i < iteration_count; ++i)
      RB_TG.add_task(task_CSR);
  }
  RB_TG.add_task(task_AVBD_FIN); // AVBD: reconstruct velocities from the pose delta
  // AVBD post-stabilization (reference postStabilize): primal passes with alpha = 0
  // (full C0) AFTER velocities are reconstructed -> corrects pre-existing penetration
  // positionally without injecting momentum. The reference runs ONE sweep, which in a
  // DEEP pile propagates extraction roughly one contact layer per frame: the box pool
  // plateaued at 40-50mm standing depth (gravity re-compresses as fast as one sweep
  // extracts). Multiple sweeps per frame converge the pile to near-flush instead.
  for (daxa_u32 ps = 0u; ps < BB_AVBD_POST_STAB_SWEEPS; ++ps)
  {
    for (daxa_u32 c = 0u; c < BB_AVBD_MAX_BODY_COLORS; ++c)
    {
      RB_TG.add_task(task_AVBD_PRIM_PS_vec[c]);
    }
  }
  RB_TG.add_task(task_CP);
  RB_TG.add_task(task_update);

  // Bind backing resources to all task buffers to prevent unbound resource compilation crashes in Daxa 3.6
  task_sim_config_host.set_buffer(sim_config_host_buffer[0]);
  task_sim_config.set_buffer(sim_config[0]);
  task_old_sim_config.set_buffer(sim_config[1]);
  task_morton_codes.set_buffer(morton_codes);
  task_tmp_morton_codes.set_buffer(tmp_morton_codes);
  task_radix_sort_histograms.set_buffer(global_histograms[0]);
  task_previous_rigid_bodies.set_buffer(rigid_body_scratch); // Placeholder
  task_rigid_bodies.set_buffer(rigid_body_scratch); // Placeholder
  task_next_rigid_bodies.set_buffer(rigid_body_scratch); // Placeholder
  task_lbvh_nodes.set_buffer(lbvh_nodes[0]);
  task_lbvh_construction_info.set_buffer(lbvh_construction_info);
  task_active_rigid_bodies.set_buffer(active_rigid_bodies[0]);
  task_rigid_body_entries.set_buffer(rigid_body_entries[0]);
  task_previous_rigid_body_entries.set_buffer(rigid_body_entries[1]);
  task_broad_phase_collisions.set_buffer(broad_phase_collisions[0]);
  task_rigid_body_scratch.set_buffer(rigid_body_scratch);
  task_body_color_mask.set_buffer(body_color_mask);
  task_manifold_color.set_buffer(manifold_color);
  task_body_color_owner.set_buffer(body_color_owner);
  task_color_count.set_buffer(color_count);
  task_avbd_state.set_buffer(avbd_state);
  task_avbd_body_color.set_buffer(avbd_body_color);
  task_rigid_body_link_manifolds.set_buffer(rigid_body_link_manifolds[0]);
  task_collision_entries.set_buffer(collision_entries[0]);
  task_collisions.set_buffer(collisions[0]);
  task_collision_scratch.set_buffer(collision_scratch);
  task_previous_rigid_body_link_manifolds.set_buffer(rigid_body_link_manifolds[1]);
  task_collision_entries_previous.set_buffer(collision_entries[1]);
  task_old_collisions.set_buffer(collisions[1]);
  task_scratch_body_links.set_buffer(scratch_body_links[0]);
  task_body_links.set_buffer(body_links[0]);
  task_manifold_links.set_buffer(manifold_links[0]);
  task_islands.set_buffer(island_buffer[0]);
  task_previous_lbvh_nodes.set_buffer(lbvh_nodes[1]);
  task_previous_islands.set_buffer(island_buffer[1]);
  task_contact_islands.set_buffer(contact_island_buffer[0]);
  task_previous_contact_islands.set_buffer(contact_island_buffer[1]);

  // Placeholders for accel_struct_mngr task buffers which are compiled here but bound later
  accel_struct_mngr->task_dispatch_buffer.set_buffer(tmp_morton_codes);
  accel_struct_mngr->task_aabb_buffer.set_buffer(tmp_morton_codes);

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

void RigidBodyManager::record_read_back_sim_config_tasks(TaskGraph &out_readback_SC_TG)
{
  daxa::InlineTaskInfo task_readback_SC({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_old_sim_config),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_sim_config_host),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = ti.get(task_old_sim_config).id,
            .dst_buffer = ti.get(task_sim_config_host).id,
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

  out_readback_SC_TG = task_manager->create_task_graph("Read back Simulation Configuration", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, {}, {}, false, daxa::QUEUE_COMPUTE_0);
}

void RigidBodyManager::record_update_sim_config_tasks(TaskGraph &out_update_SC_TG)
{
  daxa::InlineTaskInfo task_update_SC({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_sim_config_host),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_sim_config),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = ti.get(task_sim_config_host).id,
            .dst_buffer = ti.get(task_sim_config).id,
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

  out_update_SC_TG = task_manager->create_task_graph("Update Simulation Configuration", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, {}, {}, false, daxa::QUEUE_COMPUTE_0);
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
  device.destroy_buffer(voxel_shapes);
  device.destroy_buffer(voxel_occupancy);
  device.destroy_buffer(voxel_surface);

  initialized = false;
}

bool RigidBodyManager::is_dirty()
{
  return sim_flag_dirty[renderer_manager->get_sim_frame_index()];
}

void RigidBodyManager::clean_dirty()
{
  // update_sim() now refreshes BOTH parities in one call, so clear all dirty flags
  for (auto i = 0u; i < DOUBLE_BUFFERING; ++i)
  {
    sim_flag_dirty[i] = false;
  }
}

daxa::BufferId RigidBodyManager::get_lbvh_node_buffer()
{
  return lbvh_nodes[renderer_manager->get_sim_frame_index()];
}

bool RigidBodyManager::simulate()
{
  if (!initialized)
  {
    return !initialized;
  }

  // advance the SIM clock (per-step double-buffer parity, decoupled from the render frame):
  // step K works on [parity K] and reads the previous step's output at [parity K^1]
  renderer_manager->begin_sim_step();

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

  task_rigid_bodies.set_buffer(accel_struct_mngr->get_rigid_body_buffer());

  return initialized;
}

// NOTE: this function reset simulation configuration
bool RigidBodyManager::update_sim()
{
  if (!initialized)
  {
    return !initialized;
  }

  // Populate BOTH double-buffer parities. The sim consumes alternating per-step buffers, and
  // since the per-step sim clock + catch-up bursts, the dirty path is no longer guaranteed to
  // run on each parity on consecutive iterations (the old per-render-frame clock was). A
  // single-parity update would leave the other half's sim config (and consumers like the
  // reset-body-links pass) stale or empty on every second step.
  for (daxa_u32 f = 0u; f < DOUBLE_BUFFERING; ++f)
  {
    *device.buffer_host_address_as<SimConfig>(sim_config_host_buffer[f]).value() = SimConfig{
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
        .g_c_info = GlobalCollisionInfo{
            .collision_count = 0,
            .collision_point_count = 0,
        },
        .frame_count = renderer_manager->get_frame_count(),
        .voxel_shapes_addr = device.device_address(voxel_shapes).value(),
        .voxel_occupancy_addr = device.device_address(voxel_occupancy).value(),
        .voxel_surface_addr = device.device_address(voxel_surface).value(),
    };

    update_buffers(f);
    update_SC_TG.execute();
  }

  update_buffers(); // restore current-parity bindings

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

  // both parities: see update_sim() — the active list is consumed per-step at alternating parity
  for (daxa_u32 f = 0u; f < DOUBLE_BUFFERING; ++f)
  {
    update_buffers(f);
    ARB_TG.execute();
  }

  update_buffers();

  return initialized;
}

void RigidBodyManager::update_buffers()
{
  update_buffers(renderer_manager->get_sim_frame_index());
}

void RigidBodyManager::update_buffers(daxa_u32 current_frame)
{
  daxa_u32 previous_frame = (current_frame + DOUBLE_BUFFERING - 1) % DOUBLE_BUFFERING;

  task_sim_config_host.set_buffer(sim_config_host_buffer[current_frame]);
  task_sim_config.set_buffer(sim_config[current_frame]);
  task_old_sim_config.set_buffer(sim_config[previous_frame]);
  task_morton_codes.set_buffer(morton_codes);
  task_tmp_morton_codes.set_buffer(tmp_morton_codes);
  task_radix_sort_histograms.set_buffer(global_histograms[current_frame]);
  task_previous_rigid_bodies.set_buffer(accel_struct_mngr->get_previous_rigid_body_buffer());
  task_rigid_bodies.set_buffer(accel_struct_mngr->get_rigid_body_buffer());
  task_next_rigid_bodies.set_buffer(accel_struct_mngr->get_next_rigid_body_buffer());
  task_lbvh_nodes.set_buffer(lbvh_nodes[current_frame]);
  task_previous_lbvh_nodes.set_buffer(lbvh_nodes[previous_frame]);
  task_lbvh_construction_info.set_buffer(lbvh_construction_info);
  task_active_rigid_bodies.set_buffer(active_rigid_bodies[current_frame]);
  task_rigid_body_entries.set_buffer(rigid_body_entries[current_frame]);
  task_previous_rigid_body_entries.set_buffer(rigid_body_entries[previous_frame]);
  task_broad_phase_collisions.set_buffer(broad_phase_collisions[current_frame]);
  task_rigid_body_scratch.set_buffer(rigid_body_scratch);
  task_rigid_body_link_manifolds.set_buffer(rigid_body_link_manifolds[current_frame]);
  task_collision_entries.set_buffer(collision_entries[current_frame]);
  task_collisions.set_buffer(collisions[current_frame]);
  task_collision_scratch.set_buffer(collision_scratch);
  task_previous_rigid_body_link_manifolds.set_buffer(rigid_body_link_manifolds[previous_frame]);
  task_collision_entries_previous.set_buffer(collision_entries[previous_frame]);
  task_old_collisions.set_buffer(collisions[previous_frame]);
  task_scratch_body_links.set_buffer(scratch_body_links[current_frame]);
  task_body_links.set_buffer(body_links[current_frame]);
  task_manifold_links.set_buffer(manifold_links[current_frame]);
  task_islands.set_buffer(island_buffer[current_frame]);
  task_previous_islands.set_buffer(island_buffer[previous_frame]);
  task_contact_islands.set_buffer(contact_island_buffer[current_frame]);
  task_previous_contact_islands.set_buffer(contact_island_buffer[previous_frame]);
}

BB_NAMESPACE_END
