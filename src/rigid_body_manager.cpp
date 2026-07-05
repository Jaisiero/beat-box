#include "rigid_body_manager.hpp"
#include "renderer_manager.hpp"

BB_NAMESPACE_BEGIN

// The collision/manifold buffers are sized by the HOST constant MAX_RIGID_BODY_COUNT (defines.hpp),
// but the narrow-phase overflow guards use the DEVICE constant BB_MAX_RIGID_BODY_COUNT (shared.inl).
// They are independent literals; if they diverge with the device value LARGER, the guard would admit
// an index past the host-sized buffer -> out-of-bounds GPU write. Keep them locked together.
static_assert(MAX_RIGID_BODY_COUNT == BB_MAX_RIGID_BODY_COUNT,
              "MAX_RIGID_BODY_COUNT (defines.hpp) must equal BB_MAX_RIGID_BODY_COUNT (shared.inl)");

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
    pipeline_SWS = task_manager->create_compute(SingleWorkgroupSortInfo{}.info);
    pipeline_VSB_INIT = task_manager->create_compute(VoxelSdfInitInfo{}.info);
    pipeline_VSB_AXIS = task_manager->create_compute(VoxelSdfAxisInfo{}.info);
    pipeline_VSB_FIN = task_manager->create_compute(VoxelSdfFinalizeInfo{}.info);
    pipeline_VSB_SURF = task_manager->create_compute(VoxelSurfaceBuildInfo{}.info);
    pipeline_VSB_INERTIA = task_manager->create_compute(VoxelInertiaReduceInfo{}.info);
    pipeline_VSB_PRIMS = task_manager->create_compute(VoxelPrimsBuildInfo{}.info);
    pipeline_VFR_CARVE = task_manager->create_compute(VoxelCarveInfo{}.info);
    pipeline_VFR_VORONOI = task_manager->create_compute(VoxelVoronoiAssignInfo{}.info);
    pipeline_VFR_FLOOD_INIT = task_manager->create_compute(VoxelFloodInitInfo{}.info);
    pipeline_VFR_FLOOD_STEP = task_manager->create_compute(VoxelFloodStepInfo{}.info);
    pipeline_RBLBVHGH = task_manager->create_compute(RigidBodyGenerateHierarchyLinearBVHInfo{}.info);
    pipeline_BBBLBVHGH = task_manager->create_compute(RigidBodyBuildBoundingBoxesLinearBVHInfo{}.info);
    pipeline_CBBLBVHGH = task_manager->create_compute(RigidBodyConvertBoundingBoxesLinearBVHInfo{}.info);
    pipeline_RBR = task_manager->create_compute(RigidBodyReorderingInfo{}.info);
    pipeline_RBL = task_manager->create_compute(ResetBodyLinksInfo{}.info);
    pipeline_BP = task_manager->create_compute(BroadPhaseInfo{}.info);
    pipeline_NPD = task_manager->create_compute(NarrowPhaseDispatcherInfo{}.info);
    pipeline_NP = task_manager->create_compute(NarrowPhaseInfo{}.info);
    pipeline_CHS = task_manager->create_compute(ChainSortInfo{}.info);
    pipeline_PS = task_manager->create_compute(PickSpringInfo{}.info);
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
    pipeline_GCSD = task_manager->create_compute(GraphColorSolveDispatcherInfo{}.info);
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
    pipeline_AVBD_CDISP = task_manager->create_compute(AvbdColorDispatcherInfo{}.info);
    pipeline_AVBD_MAXD = task_manager->create_compute(AvbdMaxDepthInfo{}.info);
    pipeline_AVBD_CASCD = task_manager->create_compute(AvbdCascadeDispatcherInfo{}.info);
    pipeline_AVBD_CMT = task_manager->create_compute(AvbdColorCommitInfo{}.info);
    pipeline_AVBD_PRE = task_manager->create_compute(AvbdPrepareInfo{}.info);
    pipeline_AVBD_FIN = task_manager->create_compute(AvbdFinalizeInfo{}.info);
    pipeline_AVBD_WS = task_manager->create_compute(AvbdWarmstartInfo{}.info);
    pipeline_AVBD_PRIM = task_manager->create_compute(AvbdPrimalInfo{}.info);
    pipeline_AVBD_DUAL = task_manager->create_compute(AvbdDualInfo{}.info);
    pipeline_AVBD_IMPJ = task_manager->create_compute(AvbdImpactJInfo{}.info);
    pipeline_AVBD_IMPA = task_manager->create_compute(AvbdImpactApplyInfo{}.info);
    pipeline_AVBD_PKTR = task_manager->create_compute(AvbdPocketTraceInfo{}.info);
    pipeline_AVBD_DRST = task_manager->create_compute(AvbdDepthResetInfo{}.info);
    pipeline_AVBD_DRLX = task_manager->create_compute(AvbdDepthRelaxInfo{}.info);
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

  renderer_manager = renderer.get();
  gui_manager = gui.get();
  iteration_count = iterations;

  // mouse pick-and-drag bridge: host writes the ray/buttons, the GPU pick pass writes the grab
  // state (disjoint halves). HOST_ACCESS_RANDOM = host-writable + device-readable/writable.
  pick_state_buffer = create_owned({
      .size = sizeof(PickState),
      .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
      .name = "pick_state",
  });
  {
    auto *ps = device.buffer_host_address_as<PickState>(pick_state_buffer).value();
    *ps = PickState{};
    ps->picked_id = MAX_U32;
  }
  task_pick_state.set_buffer(pick_state_buffer);

  for (auto i = 0u; i < DOUBLE_BUFFERING; ++i)
  {
    sim_config_host_buffer[i] = create_owned({
        .size = sizeof(SimConfig),
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "sim_config_host_" + std::to_string(i),
    });
    sim_config[i] = create_owned({
        .size = sizeof(SimConfig),
        .name = "sim_config_" + std::to_string(i),
    });
    // FIXME: think about a better way to handle this
    auto max_number_of_workgroups = (MAX_RIGID_BODY_COUNT + RADIX_SORT_WORKGROUP_SIZE - 1) / RADIX_SORT_WORKGROUP_SIZE;
    global_histograms[i] = create_owned({
        .size = sizeof(daxa_u32) * RADIX_SORT_BINS * max_number_of_workgroups,
        .name = "global_histograms" + std::to_string(i),
    });
    lbvh_nodes[i] = create_owned({
        .size = sizeof(LBVHNode) * MAX_LBVH_NODE_COUNT,
        .name = "lbvh_nodes" + std::to_string(i),
    });
    broad_phase_collisions[i] = create_owned({
        .size = sizeof(BroadPhaseCollision) * MAX_BROAD_PAIR_COUNT,
        .name = "broad_phase_collisions" + std::to_string(i),
    });
    collision_entries[i] = create_owned({
        .size = sizeof(CollisionEntry) * MAX_COLLISION_COUNT,
        .name = "collision_entries" + std::to_string(i),
    });
    collisions[i] = create_owned({
        .size = sizeof(Manifold) * MAX_COLLISION_COUNT,
        .name = "collisions" + std::to_string(i),
    });
    rigid_body_entries[i] = create_owned({
        .size = sizeof(RigidBodyEntry) * MAX_RIGID_BODY_COUNT,
        .name = "rigid_body_map" + std::to_string(i),
    });
    active_rigid_bodies[i] = create_owned({
        .size = sizeof(ActiveRigidBody) * MAX_RIGID_BODY_COUNT,
        .name = "active_rigid_bodies" + std::to_string(i),
    });
    rigid_body_link_manifolds[i] = create_owned({
        .size = sizeof(ManifoldNode) * BB_MAX_MANIFOLD_NODE_COUNT,
        .name = "rigid_body_link_manifolds" + std::to_string(i),
    });
    scratch_body_links[i] = create_owned({
        .size = sizeof(BodyLink) * MAX_RIGID_BODY_COUNT,
        .name = "scratch_body_links" + std::to_string(i),
    });
    body_links[i] = create_owned({
        .size = sizeof(BodyLinkIsland) * MAX_RIGID_BODY_COUNT,
        .name = "body_links" + std::to_string(i),
    });
    manifold_links[i] = create_owned({
        .size = sizeof(ManifoldLinkIsland) * MAX_COLLISION_COUNT,
        .name = "manifold_links" + std::to_string(i),
    });
    island_buffer[i] = create_owned({
        .size = sizeof(Island) * MAX_RIGID_BODY_COUNT,
        .name = "islands" + std::to_string(i),
    });
    contact_island_buffer[i] = create_owned({
        .size = sizeof(ContactIsland) * MAX_RIGID_BODY_COUNT,
        .name = "contact_islands" + std::to_string(i),
    });
    // voxel collision shape data (host-writable: filled once by the scene at load time;
    // static afterwards, addressed through SimConfig - the NP head is at the push limit)
    // voxel pools stay EXPLICIT (not create_owned): lazily created + is_empty()-guarded, so they own
    // their own lifecycle and are torn down with matching is_empty() guards in destroy().
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
      voxel_sdf = device.create_buffer({
          .size = sizeof(daxa_f32) * BB_MAX_VOXEL_SDF_F32S,
          .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
          .name = "voxel_sdf",
      });
      for (auto s = 0u; s < 2u; ++s)
      {
        voxel_sdf_scratch[s] = device.create_buffer({
            .size = sizeof(daxa_f32) * BB_MAX_VOXEL_SDF_F32S,
            .name = std::string("voxel_sdf_scratch") + std::to_string(s),
        });
      }
      voxel_derived = device.create_buffer({
          .size = sizeof(VoxelShapeDerived) * BB_MAX_VOXEL_SHAPE_COUNT,
          .name = "voxel_derived",
      });
      // FRACTURE event bridge (pick_state pattern): GPU-written by the impact pass,
      // host-read by the orchestrator; survives the per-step sim-config re-upload
      fracture_events_buffer = device.create_buffer({
          .size = sizeof(FractureEventBuffer),
          .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
          .name = "fracture_events",
      });
      *device.buffer_host_address_as<FractureEventBuffer>(fracture_events_buffer).value() = FractureEventBuffer{};
      // FRACTURE Voronoi sites: host writes up to BB_MAX_FRACTURE_SITES grid-space positions
      // per event; the voronoi-assign kernel reads them
      fracture_sites_buffer = device.create_buffer({
          .size = sizeof(daxa_f32vec4) * BB_MAX_FRACTURE_SITES,
          .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
          .name = "fracture_sites",
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
        .voxel_sdf_addr = device.device_address(voxel_sdf).value(),
        .fracture_events_addr = device.device_address(fracture_events_buffer).value(),
    };
  }
  tmp_morton_codes = create_owned({
      .size = sizeof(MortonCode) * MAX_RIGID_BODY_COUNT,
      .name = "tmp_morton_codes",
  });
  morton_codes = create_owned({
      .size = sizeof(MortonCode) * MAX_RIGID_BODY_COUNT,
      .name = "morton_codes",
  });
  lbvh_construction_info = create_owned({
      .size = sizeof(LBVHConstructionInfo) * MAX_LBVH_NODE_COUNT,
      .name = "lbvh_construction_info",
  });
  rigid_body_scratch = create_owned({
      .size = sizeof(RigidBody) * MAX_RIGID_BODY_COUNT,
      .name = "rigid_body_scratch",
  });
  collision_scratch = create_owned({
      .size = sizeof(Manifold) * MAX_COLLISION_COUNT,
      .name = "collision_scratch",
  });
  // graph coloring buffers (raw u32 arrays)
  body_color_mask = create_owned({
      .size = sizeof(daxa_u32) * MAX_RIGID_BODY_COUNT,
      .name = "body_color_mask",
  });
  manifold_color = create_owned({
      .size = sizeof(daxa_u32) * MAX_COLLISION_COUNT,
      .name = "manifold_color",
  });
  body_color_owner = create_owned({
      .size = sizeof(daxa_u32) * MAX_RIGID_BODY_COUNT * BB_MAX_COLORS,
      .name = "body_color_owner",
  });
  color_count = create_owned({
      .size = sizeof(daxa_u32) * (BB_MAX_COLORS + 1),
      .name = "color_count",
  });
  // AVBD buffers
  avbd_state = create_owned({
      .size = sizeof(AvbdBodyState) * MAX_RIGID_BODY_COUNT,
      .name = "avbd_state",
  });
  avbd_body_color = create_owned({
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
        auto reset_fresh = std::array<daxa_u32, 19>{}; // dbg_fresh..dbg_state_pad (per-frame block;
                                                      // dm_ids/walk_a/walk_b persist as the latch)
        allocate_fill_copy(ti, reset_fresh, ti.get(task_sim_config), offsetof(SimConfig, dbg_fresh));

        // DIAG: zero the explosion-latch fields once per config buffer (device memory starts
        // undefined; the latch CAS needs a 0 start). Two frames cover both double-buffered configs.
        static daxa_u32 dbg_latch_init_runs = 0;
        if (dbg_latch_init_runs < 2)
        {
          ++dbg_latch_init_runs;
          auto zeroes = std::array<daxa_u32, 8>{}; // dbg_ex_stage..dbg_id_sum are contiguous
          allocate_fill_copy(ti, zeroes, ti.get(task_sim_config), offsetof(SimConfig, dbg_ex_stage));
          auto zeroes_dm = std::array<daxa_u32, 3>{}; // persistent dm latch (dm_ids..dm_walk_b)
          allocate_fill_copy(ti, zeroes_dm, ti.get(task_sim_config), offsetof(SimConfig, dbg_dm_ids));
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

  // single-workgroup whole sort: one fixed 1-workgroup dispatch replaces the 12-task LSD
  // chain (BB_MAX_RIGID_BODY_COUNT = 1024 fits 128 threads x 8 items in groupshared).
  // Stable -> bit-identical permutation to the old chain (DET-hash verified).
  auto user_callback_SWS = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_SWS);
    ti.recorder.push_constant(RigidBodySingleRadixSortPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };
  using TTask_SWS = TaskTemplate<RigidBodySingleRadixSortTaskHead::Task, decltype(user_callback_SWS)>;
  TTask_SWS task_SWS(std::array{
                         daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                         daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.sim_config, task_sim_config),
                         daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.morton_codes_in, task_morton_codes),
                         daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.morton_codes_out, task_tmp_morton_codes),
                         daxa::attachment_view(RigidBodySingleRadixSortTaskHead::AT.global_histograms, task_radix_sort_histograms),
                     },
                     user_callback_SWS);

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

  // canonical chain sort (determinism): per-body pass right after the narrow phase
  // reorders each dynamic body's manifold chain by persistent pair key, so every
  // downstream chain walk (AVBD primal gather, coloring adjacency) accumulates in an
  // order that is a pure function of the contact graph, not of the atomic insertion race
  auto user_callback_CHS = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_CHS);
    ti.recorder.push_constant(NarrowPhasePushConstants{.task_head = ti.attachment_shader_blob});
    // review v2 #4: CHS was the last per-body pass still launching the fixed MAX_RIGID_BODY_COUNT
    // grid every frame; drive it off the live per-body count like every sibling (morton/reorder/
    // broad-phase above). Bitwise-identical (same threads run; only trailing idle workgroups drop).
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(NarrowPhaseTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };
  using TTask_CHS = TaskTemplate<NarrowPhaseTaskHead::Task, decltype(user_callback_CHS)>;
  TTask_CHS task_CHS(std::array{
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
                     user_callback_CHS);

  // mouse pick-and-drag spring: one thread at the START of every sim step (before any solver
  // pass) so the injected velocity flows through whichever solver is active
  auto user_callback_PS = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_PS);
    ti.recorder.push_constant(PickSpringPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };
  using TTask_PS = TaskTemplate<PickSpringTaskHead::Task, decltype(user_callback_PS)>;
  TTask_PS task_PS(std::array{
                       daxa::attachment_view(PickSpringTaskHead::AT.sim_config, task_sim_config),
                       daxa::attachment_view(PickSpringTaskHead::AT.rigid_body_map, task_rigid_body_entries),
                       daxa::attachment_view(PickSpringTaskHead::AT.rigid_bodies, task_rigid_bodies),
                       daxa::attachment_view(PickSpringTaskHead::AT.pick_state, task_pick_state),
                   },
                   user_callback_PS);

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

  // TGS sub-step velocity integrate (gravity * h): same pipeline, tgs_phase=1 (runs per sub-step, TGS only)
  auto user_callback_tgs_advect = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_advect);
    ti.recorder.push_constant(RigidBodySimPushConstants{.task_head = ti.attachment_shader_blob, .tgs_phase = 1});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(RigidBodySimTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };
  using TTaskAdvectTGS = TaskTemplate<RigidBodySimTaskHead::Task, decltype(user_callback_tgs_advect)>;
  TTaskAdvectTGS task_tgs_advect(std::array{
                              daxa::attachment_view(RigidBodySimTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                              daxa::attachment_view(RigidBodySimTaskHead::AT.sim_config, task_sim_config),
                              daxa::attachment_view(RigidBodySimTaskHead::AT.rigid_bodies, task_rigid_bodies),
                          },
                          user_callback_tgs_advect);

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

  // TGS sub-step position integrate (x += v*h): same pipeline, tgs_phase=1 (runs per sub-step, TGS only)
  auto user_callback_tgs_ip = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_IP);
    ti.recorder.push_constant(RigidBodyIntegratePositionsPushConstants{.task_head = ti.attachment_shader_blob, .tgs_phase = 1});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(IntegratePositionsTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * ACTIVE_RIGID_BODY_DISPATCH_COUNT_OFFSET});
  };
  using TTask_IP_TGS = TaskTemplate<IntegratePositionsTaskHead::Task, decltype(user_callback_tgs_ip)>;
  TTask_IP_TGS task_tgs_ip(std::array{
                       daxa::attachment_view(IntegratePositionsTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                       daxa::attachment_view(IntegratePositionsTaskHead::AT.sim_config, task_sim_config),
                       daxa::attachment_view(IntegratePositionsTaskHead::AT.rigid_bodies, task_rigid_bodies),
                   },
                   user_callback_tgs_ip);

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
                      daxa::attachment_view(CreatePointsTaskHead::AT.body_color, task_avbd_body_color),
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

  std::array<daxa::TaskBuffer, 39> buffers = {
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
      task_pick_state,
  };

  // the whole sim runs on the async compute queue; the render graph waits the sim timeline.
  // One graph per solver so each carries only its own passes (the AVBD vs PGS/TGS cross-overhead fix).
  std::string nm_pgs = std::string(name) + "_pgs", nm_avbd = std::string(name) + "_avbd", nm_tgs = std::string(name) + "_tgs";
  RB_TG_pgs  = task_manager->create_task_graph(nm_pgs.c_str(),  std::span<daxa::TaskBuffer>(buffers), {}, {}, {}, false, daxa::QUEUE_COMPUTE_0);
  RB_TG_avbd = task_manager->create_task_graph(nm_avbd.c_str(), std::span<daxa::TaskBuffer>(buffers), {}, {}, {}, false, daxa::QUEUE_COMPUTE_0);
  RB_TG_tgs  = task_manager->create_task_graph(nm_tgs.c_str(),  std::span<daxa::TaskBuffer>(buffers), {}, {}, {}, false, daxa::QUEUE_COMPUTE_0);

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

  // per-color solve dispatcher: after the validator, write per-color solve dispatch args
  // (used colors -> ceil(coll/X) workgroups, empty colors -> 0). Reuses the dispatcher head.
  auto user_callback_GCSD = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_GCSD);
    ti.recorder.push_constant(RigidBodyDispatcherPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };
  using TTask_GCSD = TaskTemplate<RigidBodyDispatcherTaskHead::Task, decltype(user_callback_GCSD)>;
  TTask_GCSD task_GCSD(std::array{
                           daxa::attachment_view(RigidBodyDispatcherTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                           daxa::attachment_view(RigidBodyDispatcherTaskHead::AT.sim_config, task_sim_config),
                       },
                       user_callback_GCSD);

  // AVBD per-color primal dispatcher (A1): after the AVBD body-color validator, write per-color
  // workgroup counts (used body colors -> ceil(rigid_body_count/X), empty -> 0). Reuses the same
  // dispatcher head as GCSD; the AVBD primal sweeps then dispatch_indirect per color (skip empty).
  auto user_callback_AVBD_CDISP = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_AVBD_CDISP);
    ti.recorder.push_constant(RigidBodyDispatcherPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };
  using TTask_AVBD_CDISP = TaskTemplate<RigidBodyDispatcherTaskHead::Task, decltype(user_callback_AVBD_CDISP)>;
  TTask_AVBD_CDISP task_AVBD_CDISP(std::array{
                           daxa::attachment_view(RigidBodyDispatcherTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                           daxa::attachment_view(RigidBodyDispatcherTaskHead::AT.sim_config, task_sim_config),
                       },
                       user_callback_AVBD_CDISP);

  // AVBD per-(layer,color) CASCADE dispatcher (A2): after entry_avbd_max_depth (avbd_max_support_depth
  // final), write per-(layer,color) workgroup counts so the post-stab cascade skips empty upper layers.
  auto user_callback_AVBD_CASCD = [this](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*pipeline_AVBD_CASCD);
    ti.recorder.push_constant(RigidBodyDispatcherPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
  };
  using TTask_AVBD_CASCD = TaskTemplate<RigidBodyDispatcherTaskHead::Task, decltype(user_callback_AVBD_CASCD)>;
  TTask_AVBD_CASCD task_AVBD_CASCD(std::array{
                           daxa::attachment_view(RigidBodyDispatcherTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
                           daxa::attachment_view(RigidBodyDispatcherTaskHead::AT.sim_config, task_sim_config),
                       },
                       user_callback_AVBD_CASCD);

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
  auto avbd_dispatch = [this](daxa::TaskInterface ti, std::shared_ptr<daxa::ComputePipeline> &pl, daxa_u32 pc_color, daxa_f32 stab_alpha, daxa_u32 dispatch_offset, daxa_u32 ps_depth = MAX_U32, daxa_f32 relax = 1.0f)
  {
    ti.recorder.set_pipeline(*pl);
    ti.recorder.push_constant(AvbdPushConstants{.task_head = ti.attachment_shader_blob, .color = pc_color, .stab_alpha = stab_alpha, .ps_depth = ps_depth, .relax = relax});
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

  // commit phase paired with each JP round (race-free body coloring). Vector of identical tasks so
  // each round gets its own committed-color barrier before the next round reads neighbor colors.
  auto user_callback_AVBD_CMT = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_CMT, 0u, 1.0f, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_CMT = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_CMT)>;
  std::vector<TTask_AVBD_CMT> task_AVBD_CMT_vec;
  task_AVBD_CMT_vec.reserve(BB_AVBD_COLOR_ROUNDS);
  for (daxa_u32 rd = 0u; rd < BB_AVBD_COLOR_ROUNDS; ++rd)
  {
    task_AVBD_CMT_vec.emplace_back(avbd_views, user_callback_AVBD_CMT);
  }

  auto user_callback_AVBD_PRE = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_PRE, 0u, 1.0f, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_PRE = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_PRE)>;
  TTask_AVBD_PRE task_AVBD_PRE(avbd_views, user_callback_AVBD_PRE);

  auto user_callback_AVBD_FIN = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_FIN, 0u, 1.0f, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_FIN = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_FIN)>;
  TTask_AVBD_FIN task_AVBD_FIN(avbd_views, user_callback_AVBD_FIN);

  auto user_callback_AVBD_WS = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_WS, 0u, 1.0f, COLLISION_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_WS = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_WS)>;
  TTask_AVBD_WS task_AVBD_WS(avbd_views, user_callback_AVBD_WS);

  auto make_avbd_primal = [this, avbd_dispatch](daxa_u32 c, daxa_f32 stab_alpha, daxa_u32 ps_depth, daxa_f32 relax = 1.0f)
  {
    // A1: dispatch_indirect at this color's own workgroup count (0 if the body color is unused) instead
    // of the full body grid for all 32 colors early-outing 25/32 of them. Written by task_AVBD_CDISP.
    // A2: for the SHOCK CASCADE (ps_depth != MAX_U32) use the per-(layer,color) count instead, so a
    // used color in an EMPTY layer (above the pile height) is also skipped. Written by task_AVBD_CASCD.
    daxa_u32 disp_off = (ps_depth == MAX_U32) ? (AVBD_COLOR_SOLVE_DISPATCH_OFFSET + c)
                                              : (AVBD_CASCADE_DISPATCH_OFFSET + ps_depth * BB_MAX_COLORS + c);
    return [this, avbd_dispatch, c, stab_alpha, ps_depth, disp_off, relax](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_PRIM, c, stab_alpha, disp_off, ps_depth, relax); };
  };
  using TTask_AVBD_PRIM = TaskTemplate<AvbdTaskHead::Task, decltype(make_avbd_primal(0u, 1.0f, MAX_U32))>;
  std::vector<TTask_AVBD_PRIM> task_AVBD_PRIM_vec;     // main sweeps: alpha = 1 (delta-only constraint)
  std::vector<TTask_AVBD_PRIM> task_AVBD_PRIM_PS_vec;  // post-stab cascade: alpha = 0 (full C0),
                                                       // ORDERED by support depth (Guendelman):
                                                       // entry [d * COLORS + c]
  std::vector<TTask_AVBD_PRIM> task_AVBD_PRIM_PS_plain_vec; // post-stab without the layer
                                                            // filter (symmetric polish sweeps)
  std::vector<TTask_AVBD_PRIM> task_AVBD_PRIM_PS_relax_vec; // EXTRA post-stab sweeps at
                                                            // BB_AVBD_PS_RELAX step scale
                                                            // (damped: more sweeps converge)
  task_AVBD_PRIM_vec.reserve(BB_AVBD_MAX_BODY_COLORS);
  task_AVBD_PRIM_PS_vec.reserve(BB_AVBD_SHOCK_LAYERS * BB_AVBD_MAX_BODY_COLORS);
  task_AVBD_PRIM_PS_plain_vec.reserve(BB_AVBD_MAX_BODY_COLORS);
  task_AVBD_PRIM_PS_relax_vec.reserve(BB_AVBD_MAX_BODY_COLORS);
  for (daxa_u32 c = 0u; c < BB_AVBD_MAX_BODY_COLORS; ++c)
  {
    task_AVBD_PRIM_vec.emplace_back(avbd_views, make_avbd_primal(c, 1.0f, MAX_U32));
    task_AVBD_PRIM_PS_plain_vec.emplace_back(avbd_views, make_avbd_primal(c, 0.0f, MAX_U32));
    task_AVBD_PRIM_PS_relax_vec.emplace_back(avbd_views, make_avbd_primal(c, 0.0f, MAX_U32, BB_AVBD_PS_RELAX));
  }
  for (daxa_u32 d = 0u; d < BB_AVBD_SHOCK_LAYERS; ++d)
  {
    for (daxa_u32 c = 0u; c < BB_AVBD_MAX_BODY_COLORS; ++c)
    {
      task_AVBD_PRIM_PS_vec.emplace_back(avbd_views, make_avbd_primal(c, 0.0f, d));
    }
  }

  // shock propagation: support-depth BFS over the contact graph (reset + N relax passes)
  auto user_callback_AVBD_DRST = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_DRST, 0u, 1.0f, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_DRST = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_DRST)>;
  TTask_AVBD_DRST task_AVBD_DRST(avbd_views, user_callback_AVBD_DRST);
  // A2: max support-depth reduction (per body, after the depth BFS). Feeds the cascade dispatcher.
  auto user_callback_AVBD_MAXD = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_MAXD, 0u, 1.0f, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_MAXD = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_MAXD)>;
  TTask_AVBD_MAXD task_AVBD_MAXD(avbd_views, user_callback_AVBD_MAXD);
  auto user_callback_AVBD_DRLX = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_DRLX, 0u, 1.0f, COLLISION_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_DRLX = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_DRLX)>;
  TTask_AVBD_DRLX task_AVBD_DRLX(avbd_views, user_callback_AVBD_DRLX);

  auto user_callback_AVBD_DUAL = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_DUAL, 0u, 1.0f, COLLISION_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_DUAL = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_DUAL)>;
  TTask_AVBD_DUAL task_AVBD_DUAL(avbd_views, user_callback_AVBD_DUAL);

  // inelastic impact treatment (e=0), post-FIN: J computes per-contact rebound-removal
  // impulses (per manifold, reads only), APPLY gathers each body's own share (per body)
  auto user_callback_AVBD_IMPJ = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_IMPJ, 0u, 1.0f, COLLISION_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_IMPJ = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_IMPJ)>;
  TTask_AVBD_IMPJ task_AVBD_IMPJ(avbd_views, user_callback_AVBD_IMPJ);
  auto user_callback_AVBD_IMPA = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_IMPA, 0u, 1.0f, RIGID_BODY_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_IMPA = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_IMPA)>;
  TTask_AVBD_IMPA task_AVBD_IMPA(avbd_views, user_callback_AVBD_IMPA);

  // deep-pocket oscillator trace (diagnostic): per manifold, latches the deepest awake contact
  auto user_callback_AVBD_PKTR = [this, avbd_dispatch](daxa::TaskInterface ti, auto &) { avbd_dispatch(ti, pipeline_AVBD_PKTR, 0u, 1.0f, COLLISION_DISPATCH_COUNT_OFFSET); };
  using TTask_AVBD_PKTR = TaskTemplate<AvbdTaskHead::Task, decltype(user_callback_AVBD_PKTR)>;
  TTask_AVBD_PKTR task_AVBD_PKTR(avbd_views, user_callback_AVBD_PKTR);

  // ---- per-color solver tasks (Phase 3): one dispatch per color, each filters manifold_color==color ----
  static const daxa_u32 MAX_COLORS_SOLVE = BB_MAX_COLORS_SOLVE; // shared.inl: per-color solver dispatch count; empty colors are cheap no-ops
  auto gc_solve_views = std::array{
      daxa::attachment_view(GraphColorSolveTaskHead::AT.dispatch_buffer, accel_struct_mngr->task_dispatch_buffer),
      daxa::attachment_view(GraphColorSolveTaskHead::AT.sim_config, task_sim_config),
      daxa::attachment_view(GraphColorSolveTaskHead::AT.collisions, task_collisions),
      daxa::attachment_view(GraphColorSolveTaskHead::AT.rigid_bodies, task_rigid_bodies),
      daxa::attachment_view(GraphColorSolveTaskHead::AT.manifold_color, task_manifold_color),
  };
  auto make_gcs = [this](std::shared_ptr<daxa::ComputePipeline> pl, daxa_u32 c, daxa_i32 tgs_phase = 0) {
    return [this, pl, c, tgs_phase](daxa::TaskInterface ti, auto &) {
      ti.recorder.set_pipeline(*pl);
      ti.recorder.push_constant(GraphColorSolvePushConstants{.task_head = ti.attachment_shader_blob, .color = c, .tgs_phase = tgs_phase});
      // per-color dispatch: this color's own workgroup count (0 if the color is unused) instead of
      // dispatching the full collision count for all 32 colors and early-outing 31/32 of the threads
      ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(GraphColorSolveTaskHead::AT.dispatch_buffer).id,
                                     .offset = sizeof(daxa_u32vec3) * (GRAPH_COLOR_SOLVE_DISPATCH_OFFSET + c)});
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

  // TGS_SOFT sub-step instances (same pipelines, tgs_phase=1 so the shader runs the TGS branch).
  // Reuses the per-color graph-coloring dispatch (incl. the empty-color skip) -> TGS stays parallel.
  std::vector<TTask_GCS> task_TGS_CPS_vec, task_TGS_CS_vec, task_TGS_CSR_vec;
  task_TGS_CPS_vec.reserve(MAX_COLORS_SOLVE);
  task_TGS_CS_vec.reserve(MAX_COLORS_SOLVE);
  task_TGS_CSR_vec.reserve(MAX_COLORS_SOLVE);
  for (daxa_u32 c = 0u; c < MAX_COLORS_SOLVE; ++c)
  {
    task_TGS_CPS_vec.emplace_back(gc_solve_views, make_gcs(pipeline_GCS_CPS, c, 1));
    task_TGS_CS_vec.emplace_back(gc_solve_views, make_gcs(pipeline_GCS_CS, c, 1));
    task_TGS_CSR_vec.emplace_back(gc_solve_views, make_gcs(pipeline_GCS_CSR, c, 1));
  }

  // overflow bucket: serial single-thread solve of manifolds the per-color dispatches skip
  // (uncolored / color>=MAX_COLORS_SOLVE). Early-outs on graph_color_overflow==0, so it is
  // free except on degenerate frames.
  auto make_gcs_ov = [this](std::shared_ptr<daxa::ComputePipeline> pl, daxa_i32 tgs_phase = 0) {
    return [this, pl, tgs_phase](daxa::TaskInterface ti, auto &) {
      ti.recorder.set_pipeline(*pl);
      ti.recorder.push_constant(GraphColorSolvePushConstants{.task_head = ti.attachment_shader_blob, .color = 0u, .tgs_phase = tgs_phase});
      ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
    };
  };
  using TTask_GCS_OV = TaskTemplate<GraphColorSolveTaskHead::Task, decltype(make_gcs_ov(pipeline_GCS_CS_OV))>;
  TTask_GCS_OV task_GCS_CPS_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CPS_OV));
  TTask_GCS_OV task_GCS_CS_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CS_OV));
  TTask_GCS_OV task_GCS_CSR_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CSR_OV));
  TTask_GCS_OV task_TGS_CPS_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CPS_OV, 1));
  TTask_GCS_OV task_TGS_CS_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CS_OV, 1));
  TTask_GCS_OV task_TGS_CSR_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CSR_OV, 1));

  // Per-solver task graph: shared setup (broad/narrow/islands/sleeping) + ONLY the active solver's
  // passes. simulate() runs the one matching solver_type, so PGS/TGS no longer pay AVBD's ~847
  // dispatches/frame (the cross-solver overhead that made them slow since AVBD landed).
  auto record_solve = [&](TaskGraph &G, SimSolverType solver)
  {
  G.add_task(task_PS); // mouse pick-and-drag spring (velocity injection BEFORE the step)
  G.add_task(task_RC);
  G.add_task(task_CRB);
  G.add_task(task_RBD);
  G.add_task(task_GMC);
  // single-workgroup whole sort: 1 task instead of the 12-task multi-dispatch LSD chain
  // (histogram/scatter/shift x4). Same stable permutation; ~11 fewer barriers per step.
  G.add_task(task_SWS);
  G.add_task(task_RBLBVHGH);
  G.add_task(task_BBBLBVHGH);
  G.add_task(task_CBBLBVHGH);
  G.add_task(task_RBR);
  G.add_task(task_RBL);
  G.add_task(task_BP);
  G.add_task(task_NPD);
  G.add_task(task_NP);
  G.add_task(task_CHS); // determinism: canonical chain sort (post-NP, before advect)
  G.add_task(task_advect);
  G.add_task(task_IC);
  G.add_task(task_CS_dispatcher);
  G.add_task(task_ID);
  G.add_task(task_IB);
  G.add_task(task_IPS);
  G.add_task(task_IBL);
  // neighborhood sleeping: decide sleep/wake BEFORE the solve so sleeping bodies skip it this step
  G.add_task(task_SLR);
  G.add_task(task_SLV);
  G.add_task(task_SLA);
  // FIXME: that's a really expensive sort
  // G.add_task(task_SBLI);
  G.add_task(task_MIB);
  G.add_task(task_CGI);
  G.add_task(task_CID);
  G.add_task(task_MIPS);
  G.add_task(task_IML);
  // FIXME: that's a really expensive sort too
  // G.add_task(task_SMLI);
  // graph coloring. MUST run after task_IML: the coloring, the validator AND the per-color solver
  // all index `task_collisions`, which task_IML (re)writes in island-sorted order THIS frame.
  // Coloring earlier (e.g. right after the narrow phase) reads the previous content of the buffer
  // (2 frames old under double buffering), so the colors describe a different manifold->body
  // mapping than the one the per-color solver dispatches over. Two same-body manifolds can then
  // land in one color and solve concurrently -> racy Jacobi-style overcorrection -> a resting
  // body gets ejected at thousands of m/s within one CS sweep (and the validator stays at 0
  // violations because it validated the SAME stale data the colorer saw).
  // contact coloring: PGS family + TGS use it (AVBD has its own body coloring)
  if (solver != SimSolverType::AVBD)
  {
  G.add_task(task_GCD);
  G.add_task(task_GCR);
  for (auto r = 0u; r < GRAPH_COLOR_MAX_ROUNDS; ++r)
  {
    G.add_task(task_GCOR);
    G.add_task(task_GCP1);
    G.add_task(task_GCP2);
  }
  G.add_task(task_GCOR); // reset owner-as-seen for the validator
  G.add_task(task_GCV);
  G.add_task(task_GCV2); // TEMP diag: satbody degree/partners
  G.add_task(task_GCSD); // per-color solve dispatch args (skip empty colors) — graph_color_count now final
  // AVBD body coloring (independent of the contact coloring above; entries are cheap and the
  // primal solve will only run when solver_type == AVBD)
  } // end contact coloring
  // AVBD: body coloring + prepare + shock + primal/dual + FIN + impact + post-stab + trace (AVBD only)
  if (solver == SimSolverType::AVBD)
  {
  G.add_task(task_AVBD_CR);
  for (daxa_u32 rd = 0u; rd < BB_AVBD_COLOR_ROUNDS; ++rd)
  {
    G.add_task(task_AVBD_CRND_vec[rd]); // propose (writes proposed_color, reads stable body_color)
    G.add_task(task_AVBD_CMT_vec[rd]);  // commit proposed_color -> body_color (barrier between rounds)
  }
  G.add_task(task_AVBD_CV);
  G.add_task(task_AVBD_CDISP); // A1: per-color primal dispatch args (skip empty body colors) — avbd_color_count now final
  G.add_task(task_AVBD_PRE); // AVBD: save step-start pose + jump to the inertial target
  G.add_task(task_AVBD_WS);  // AVBD: lambda/k warm-start scaling
  // shock propagation: support-depth BFS (statics/sleepers = 0; each pass relaxes
  // depth = min(depth, touching partner + 1)). Consumed by the ORDERED post-stab
  // cascade below.
  G.add_task(task_AVBD_DRST);
  for (daxa_u32 dr = 0u; dr < BB_AVBD_SHOCK_LAYERS; ++dr)
  {
    G.add_task(task_AVBD_DRLX);
  }
  G.add_task(task_AVBD_MAXD);  // A2: reduce max support-depth (BFS converged; support_depth not touched after this)
  G.add_task(task_AVBD_CASCD); // A2: per-(layer,color) cascade dispatch args (skip empty upper layers)
  for (daxa_u32 it = 0u; it < BB_AVBD_ITERATIONS; ++it)
  {
    for (daxa_u32 c = 0u; c < BB_AVBD_MAX_BODY_COLORS; ++c)
    {
      G.add_task(task_AVBD_PRIM_vec[c]);
    }
    G.add_task(task_AVBD_DUAL);
  }
  } // end AVBD primal/dual
  if (solver == SimSolverType::PGS || solver == SimSolverType::PGS_SOFT)
  {
  if (static_cast<daxa_u32>(sim_flags & SimFlag::USE_GRAPH_COLORING) != 0u)
  {
    // per-color solve (parallel: one dispatch per color, balanced, no atomics)
    // + serial overflow bucket after each sweep (uncolored / color>=MAX_COLORS_SOLVE manifolds)
    for (daxa_u32 c = 0u; c < MAX_COLORS_SOLVE; ++c)
      G.add_task(task_GCS_CPS_vec[c]);
    G.add_task(task_GCS_CPS_OV);
    for (auto i = 0u; i < iteration_count; ++i)
    {
      for (daxa_u32 c = 0u; c < MAX_COLORS_SOLVE; ++c)
        G.add_task(task_GCS_CS_vec[c]);
      G.add_task(task_GCS_CS_OV);
    }
    G.add_task(task_IP);
    for (auto i = 0u; i < iteration_count; ++i)
    {
      for (daxa_u32 c = 0u; c < MAX_COLORS_SOLVE; ++c)
        G.add_task(task_GCS_CSR_vec[c]);
      G.add_task(task_GCS_CSR_OV);
    }
  }
  else
  {
    // per-island solve (serial within each contact island)
    G.add_task(task_CPS);
    for (auto i = 0u; i < iteration_count; ++i)
      G.add_task(task_CS);
    G.add_task(task_IP);
    for (auto i = 0u; i < iteration_count; ++i)
      G.add_task(task_CSR);
  }
  } // end PGS solve
  if (solver == SimSolverType::AVBD)
  {
  G.add_task(task_AVBD_FIN); // AVBD: reconstruct velocities from the pose delta
  G.add_task(task_AVBD_IMPJ);  // inelastic impact (e=0): rebound-removal impulses
  G.add_task(task_AVBD_IMPA);  // inelastic impact (e=0): per-body application
  // AVBD post-stabilization (reference postStabilize): primal passes with alpha = 0
  // (full C0) AFTER velocities are reconstructed -> corrects pre-existing penetration
  // positionally without injecting momentum. Multiple sweeps converge deep piles, and
  // each sweep is a SHOCK-PROPAGATION CASCADE (Guendelman): layers are processed in
  // support-depth order, so every body extracts against supports that already settled
  // this sweep. (v1 tried row-skipping WITHOUT ordering and measured as a potential
  // energy pump - the ordering is the load-bearing part.)
  // HYBRID post-stab: the FIRST sweep is the shock-propagation cascade (ground-up layer
  // order tames the violent settling phase - measured: rains stop boiling), the rest are
  // plain symmetric sweeps (each pair revisited -> deeper standing extraction at rest:
  // cascade-only plateaued at 42mm vs 27-31mm for symmetric sweeps).
  for (daxa_u32 d = 0u; d < BB_AVBD_SHOCK_LAYERS; ++d)
  {
    for (daxa_u32 c = 0u; c < BB_AVBD_MAX_BODY_COLORS; ++c)
    {
      G.add_task(task_AVBD_PRIM_PS_vec[d * BB_AVBD_MAX_BODY_COLORS + c]);
    }
  }
  for (daxa_u32 ps = 1u; ps < BB_AVBD_POST_STAB_SWEEPS; ++ps)
  {
    for (daxa_u32 c = 0u; c < BB_AVBD_MAX_BODY_COLORS; ++c)
    {
      G.add_task(task_AVBD_PRIM_PS_plain_vec[c]);
    }
  }
  // EXTRA damped sweeps - knob currently 0 (MEASURED WORSE, see BB_AVBD_POST_STAB_RELAXED
  // in shared.inl); if constexpr keeps the falsified-but-kept mechanism from emitting the
  // always-false-loop warning (C4296) while it sits parked.
  if constexpr (BB_AVBD_POST_STAB_RELAXED > 0u)
  {
    for (daxa_u32 ps = 0u; ps < BB_AVBD_POST_STAB_RELAXED; ++ps)
    {
      for (daxa_u32 c = 0u; c < BB_AVBD_MAX_BODY_COLORS; ++c)
      {
        G.add_task(task_AVBD_PRIM_PS_relax_vec[c]);
      }
    }
  }
  } // end AVBD FIN/impact/post-stab
  if (solver == SimSolverType::TGS_SOFT)
  {
  // TGS_SOFT (Box2D v3 / solver2d): sub-stepped soft solver, integrated with graph coloring.
  // All tasks early-return unless solver_type==TGS_SOFT, so this block is free for the other solvers.
  // Prepare once (soft coeffs at sub-step h + local anchors), then BB_TGS_SUBSTEPS sub-steps of:
  // integrate velocity (gravity*h) -> per-color solve (bias) -> integrate positions (x+=v*h) ->
  // per-color relax (no bias). The separation is re-derived from the pose each sub-step (TGS temporal).
  for (daxa_u32 c = 0u; c < MAX_COLORS_SOLVE; ++c)
    G.add_task(task_TGS_CPS_vec[c]);
  G.add_task(task_TGS_CPS_OV);
  for (daxa_u32 s = 0u; s < BB_TGS_SUBSTEPS; ++s)
  {
    G.add_task(task_tgs_advect);
    for (daxa_u32 c = 0u; c < MAX_COLORS_SOLVE; ++c)
      G.add_task(task_TGS_CS_vec[c]);
    G.add_task(task_TGS_CS_OV);
    G.add_task(task_tgs_ip);
    for (daxa_u32 c = 0u; c < MAX_COLORS_SOLVE; ++c)
      G.add_task(task_TGS_CSR_vec[c]);
    G.add_task(task_TGS_CSR_OV);
  }
  } // end TGS sub-step loop
  if (solver == SimSolverType::AVBD)
    G.add_task(task_AVBD_PKTR); // diagnostic (AVBD only)
  G.add_task(task_CP);
  G.add_task(task_update);
  }; // end record_solve lambda
  record_solve(RB_TG_pgs,  SimSolverType::PGS_SOFT);
  record_solve(RB_TG_avbd, SimSolverType::AVBD);
  record_solve(RB_TG_tgs,  SimSolverType::TGS_SOFT);

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

  RB_TG_pgs.submit();  RB_TG_pgs.complete();
  RB_TG_avbd.submit(); RB_TG_avbd.complete();
  RB_TG_tgs.submit();  RB_TG_tgs.complete();


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

  // B2: free EXACTLY what create_owned() allocated (per-frame + scratch + color + AVBD buffers) by
  // iterating the ownership list — the hand-maintained parallel destroy list (which drifted once) is gone.
  for (auto b : owned_buffers)
  {
    device.destroy_buffer(b);
  }
  owned_buffers.clear();
  // voxel pools: explicit, is_empty()-guarded (lazily created, may not exist); reset so a re-create works.
  if (!voxel_shapes.is_empty())    { device.destroy_buffer(voxel_shapes);    voxel_shapes = {}; }
  if (!voxel_occupancy.is_empty()) { device.destroy_buffer(voxel_occupancy); voxel_occupancy = {}; }
  if (!voxel_surface.is_empty())   { device.destroy_buffer(voxel_surface);   voxel_surface = {}; }
  if (!voxel_sdf.is_empty())       { device.destroy_buffer(voxel_sdf);       voxel_sdf = {}; }
  for (auto s = 0u; s < 2u; ++s)
  {
    if (!voxel_sdf_scratch[s].is_empty()) { device.destroy_buffer(voxel_sdf_scratch[s]); voxel_sdf_scratch[s] = {}; }
  }
  if (!voxel_derived.is_empty()) { device.destroy_buffer(voxel_derived); voxel_derived = {}; }
  if (!fracture_events_buffer.is_empty()) { device.destroy_buffer(fracture_events_buffer); fracture_events_buffer = {}; }
  if (!fracture_sites_buffer.is_empty()) { device.destroy_buffer(fracture_sites_buffer); fracture_sites_buffer = {}; }

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

  // execute only the active solver's graph (each carries only its own passes -> no cross-overhead)
  TaskGraph &RB_TG_active = (solver_type == SimSolverType::AVBD)     ? RB_TG_avbd
                                : (solver_type == SimSolverType::TGS_SOFT) ? RB_TG_tgs
                                                                           : RB_TG_pgs;
  RB_TG_active.execute();

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
        .voxel_sdf_addr = device.device_address(voxel_sdf).value(),
        .fracture_events_addr = device.device_address(fracture_events_buffer).value(),
    };

    update_buffers(f);
    update_SC_TG.execute();
  }

  update_buffers(); // restore current-parity bindings

  return initialized;
}

void RigidBodyManager::build_voxel_pools_gpu(std::vector<VoxelShape> const &shapes,
                                             std::vector<daxa_f32> const &cpu_sdf_reference,
                                             std::vector<daxa_u32> const &cpu_surf_reference,
                                             std::vector<VoxelShapeDerived> const &cpu_derived_reference)
{
  if (!initialized || shapes.empty()) { return; }
  auto const occ_addr = device.device_address(voxel_occupancy).value();
  auto const sdf_addr = device.device_address(voxel_sdf).value();
  auto const sc0_addr = device.device_address(voxel_sdf_scratch[0]).value();
  auto const sc1_addr = device.device_address(voxel_sdf_scratch[1]).value();
  auto const shapes_addr = device.device_address(voxel_shapes).value();
  auto const surf_addr = device.device_address(voxel_surface).value();
  auto const derived_addr = device.device_address(voxel_derived).value();

  auto rec = device.create_command_recorder({});
  auto const barrier = [&rec]() {
    rec.pipeline_barrier({
        .src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
        .dst_access = daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE,
    });
  };
  for (daxa_u32 si = 0u; si < (daxa_u32)shapes.size(); ++si)
  {
    auto const &s = shapes[si];
    if (s.dims.x == 0u) { continue; } // retired/free shape slot (dims=0 sentinel) - skip
    daxa_u32 const nx = s.dims.x + 1u, ny = s.dims.y + 1u, nz = s.dims.z + 1u;
    daxa_u32 const nodes = nx * ny * nz;
    VoxelSdfBuildPushConstants pc = {
        .occupancy_addr = occ_addr,
        .sdf_addr = sdf_addr,
        .scratch_solid_addr = sc0_addr,
        .scratch_empty_addr = sc1_addr,
        .shapes_addr = shapes_addr,
        .surface_addr = surf_addr,
        .derived_addr = derived_addr,
        .prims_addr = 0u,
        .cell_dims = s.dims,
        .occ_offset = s.occ_offset,
        .sdf_offset = s.sdf_offset,
        .surf_offset = s.surf_offset,
        .shape_index = si,
        .prims_offset = 0u,
        .axis = 0u,
        .voxel_size = s.voxel_size,
    };
    rec.set_pipeline(*pipeline_VSB_INIT);
    rec.push_constant(pc);
    rec.dispatch({.x = (nodes + 63u) / 64u, .y = 1, .z = 1});
    barrier();
    daxa_u32 const nd[3] = {nx, ny, nz};
    for (daxa_u32 axis = 0u; axis < 3u; ++axis)
    {
      pc.axis = axis;
      daxa_u32 const columns = nd[(axis + 1u) % 3u] * nd[(axis + 2u) % 3u];
      rec.set_pipeline(*pipeline_VSB_AXIS);
      rec.push_constant(pc);
      rec.dispatch({.x = (columns + 63u) / 64u, .y = 1, .z = 1});
      barrier();
    }
    rec.set_pipeline(*pipeline_VSB_FIN);
    rec.push_constant(pc);
    rec.dispatch({.x = (nodes + 63u) / 64u, .y = 1, .z = 1});
    barrier();
    // surface list + mass-property reduce are independent of the EDT chain (they read
    // only the bitmask); single group each
    rec.set_pipeline(*pipeline_VSB_SURF);
    rec.push_constant(pc);
    rec.dispatch({.x = 1, .y = 1, .z = 1});
    rec.set_pipeline(*pipeline_VSB_INERTIA);
    rec.push_constant(pc);
    rec.dispatch({.x = 1, .y = 1, .z = 1});
    barrier();
  }
  auto cmds = rec.complete_current_commands();
  device.submit_commands({.command_lists = std::array{cmds}});
  device.wait_idle();

  // BB_SDF_VERIFY: read the GPU field back and compare against the CPU brute force (the
  // debug oracle the GPU-first directive keeps around). Exactness argument in voxel_sdf.slang;
  // only sqrt rounding may differ.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996) // read-only getenv is safe (same suppression as scene_manager)
#endif
  static bool const verify = std::getenv("BB_SDF_VERIFY") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
  if (verify && !cpu_sdf_reference.empty())
  {
    auto const size = cpu_sdf_reference.size() * sizeof(daxa_f32);
    daxa::BufferId staging = device.create_buffer({
        .size = size,
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "voxel_sdf_verify_staging",
    });
    auto rec2 = device.create_command_recorder({});
    rec2.copy_buffer_to_buffer({.src_buffer = voxel_sdf, .dst_buffer = staging, .size = size});
    auto cmds2 = rec2.complete_current_commands();
    device.submit_commands({.command_lists = std::array{cmds2}});
    device.wait_idle();
    daxa_f32 const *gpu = device.buffer_host_address_as<daxa_f32>(staging).value();
    double max_diff = 0.0;
    size_t worst = 0;
    for (size_t i = 0; i < cpu_sdf_reference.size(); ++i)
    {
      double const d = std::abs((double)gpu[i] - (double)cpu_sdf_reference[i]);
      if (d > max_diff) { max_diff = d; worst = i; }
    }
    std::cout << "[SDF-VERIFY] nodes=" << cpu_sdf_reference.size() << " max|gpu-cpu|=" << max_diff
              << " (worst node " << worst << ": gpu=" << gpu[worst] << " cpu=" << cpu_sdf_reference[worst] << ")"
              << ((max_diff < 1e-4) ? "  => MATCH" : "  => MISMATCH!") << std::endl;
    device.destroy_buffer(staging);
  }
  if (verify && !cpu_surf_reference.empty())
  {
    // surface list must be BYTE-IDENTICAL (same canonical order) and the GPU-patched
    // surf_count must equal the CPU count for every shape
    auto const surf_size = cpu_surf_reference.size() * sizeof(daxa_u32);
    auto const shapes_size = shapes.size() * sizeof(VoxelShape);
    daxa::BufferId staging = device.create_buffer({
        .size = surf_size + shapes_size,
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "voxel_surf_verify_staging",
    });
    auto rec2 = device.create_command_recorder({});
    rec2.copy_buffer_to_buffer({.src_buffer = voxel_surface, .dst_buffer = staging, .size = surf_size});
    rec2.copy_buffer_to_buffer({.src_buffer = voxel_shapes, .dst_buffer = staging, .dst_offset = surf_size, .size = shapes_size});
    auto cmds2 = rec2.complete_current_commands();
    device.submit_commands({.command_lists = std::array{cmds2}});
    device.wait_idle();
    daxa_u32 const *gpu_surf = device.buffer_host_address_as<daxa_u32>(staging).value();
    VoxelShape const *gpu_shapes = reinterpret_cast<VoxelShape const *>(gpu_surf + cpu_surf_reference.size());
    daxa_u32 entry_mismatches = 0u, count_mismatches = 0u, checked = 0u;
    for (size_t si = 0; si < shapes.size(); ++si)
    {
      if (gpu_shapes[si].surf_count != shapes[si].surf_count) { ++count_mismatches; }
      for (daxa_u32 i = 0u; i < shapes[si].surf_count; ++i, ++checked)
      {
        if (gpu_surf[shapes[si].surf_offset + i] != cpu_surf_reference[shapes[si].surf_offset + i]) { ++entry_mismatches; }
      }
    }
    std::cout << "[SURF-VERIFY] shapes=" << shapes.size() << " entries=" << checked
              << " entry_mismatches=" << entry_mismatches << " count_mismatches=" << count_mismatches
              << ((entry_mismatches + count_mismatches == 0u) ? "  => MATCH" : "  => MISMATCH!") << std::endl;
    device.destroy_buffer(staging);
  }
  if (verify && !cpu_derived_reference.empty())
  {
    // mass properties: counts must be exact; com/inertia within FP-order tolerance (the
    // GPU accumulates the CoM in f32 where the CPU authoring uses f64)
    auto const size = shapes.size() * sizeof(VoxelShapeDerived);
    daxa::BufferId staging = device.create_buffer({
        .size = size,
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "voxel_derived_verify_staging",
    });
    auto rec2 = device.create_command_recorder({});
    rec2.copy_buffer_to_buffer({.src_buffer = voxel_derived, .dst_buffer = staging, .size = size});
    auto cmds2 = rec2.complete_current_commands();
    device.submit_commands({.command_lists = std::array{cmds2}});
    device.wait_idle();
    VoxelShapeDerived const *gpu = device.buffer_host_address_as<VoxelShapeDerived>(staging).value();
    daxa_u32 count_mismatches = 0u;
    double max_rel = 0.0;
    for (size_t si = 0; si < shapes.size(); ++si)
    {
      auto const &c = cpu_derived_reference[si];
      auto const &g = gpu[si];
      if (g.count != c.count) { ++count_mismatches; }
      auto const rel = [&](double gv, double cv) {
        double const denom = std::max(std::abs(cv), 1e-6);
        max_rel = std::max(max_rel, std::abs(gv - cv) / denom);
      };
      auto const rel3 = [&](daxa_f32vec3 const &gv, daxa_f32vec3 const &cv) {
        rel(gv.x, cv.x); rel(gv.y, cv.y); rel(gv.z, cv.z);
      };
      rel3(g.com, c.com);
      rel3(g.unit_inertia.x, c.unit_inertia.x);
      rel3(g.unit_inertia.y, c.unit_inertia.y);
      rel3(g.unit_inertia.z, c.unit_inertia.z);
    }
    std::cout << "[INERTIA-VERIFY] shapes=" << shapes.size() << " count_mismatches=" << count_mismatches
              << " max_rel=" << max_rel
              << ((count_mismatches == 0u && max_rel < 1e-3) ? "  => MATCH" : "  => MISMATCH!") << std::endl;
    device.destroy_buffer(staging);
  }
}

void RigidBodyManager::build_voxel_prims_gpu(std::vector<VoxelShape> const &shapes,
                                             std::vector<std::pair<daxa_u32, daxa_u32>> const &bodies,
                                             daxa::BufferId prims_buffer,
                                             std::vector<Aabb> const &cpu_reference)
{
  if (!initialized || bodies.empty()) { return; }
  auto const occ_addr = device.device_address(voxel_occupancy).value();
  auto const shapes_addr = device.device_address(voxel_shapes).value();
  auto const prims_addr = device.device_address(prims_buffer).value();

  auto rec = device.create_command_recorder({});
  for (auto const &[shape_index, prim_offset] : bodies)
  {
    auto const &s = shapes[shape_index];
    daxa_u32 const cells = s.dims.x * s.dims.y * s.dims.z;
    VoxelSdfBuildPushConstants pc = {
        .occupancy_addr = occ_addr,
        .sdf_addr = 0u,
        .scratch_solid_addr = 0u,
        .scratch_empty_addr = 0u,
        .shapes_addr = shapes_addr,
        .surface_addr = 0u,
        .derived_addr = 0u,
        .prims_addr = prims_addr,
        .cell_dims = s.dims,
        .occ_offset = s.occ_offset,
        .sdf_offset = 0u,
        .surf_offset = 0u,
        .shape_index = shape_index,
        .prims_offset = prim_offset,
        .axis = 0u,
        .voxel_size = s.voxel_size,
    };
    rec.set_pipeline(*pipeline_VSB_PRIMS);
    rec.push_constant(pc);
    rec.dispatch({.x = (cells + 63u) / 64u, .y = 1, .z = 1});
  }
  // make the writes visible to the AS build that follows this call
  rec.pipeline_barrier({
      .src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
      .dst_access = daxa::AccessConsts::ACCELERATION_STRUCTURE_BUILD_READ,
  });
  auto cmds = rec.complete_current_commands();
  device.submit_commands({.command_lists = std::array{cmds}});
  device.wait_idle();

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
  static bool const verify = std::getenv("BB_SDF_VERIFY") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
  if (verify && !cpu_reference.empty())
  {
    auto const size = cpu_reference.size() * sizeof(Aabb);
    daxa::BufferId staging = device.create_buffer({
        .size = size,
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "voxel_prims_verify_staging",
    });
    auto rec2 = device.create_command_recorder({});
    rec2.copy_buffer_to_buffer({.src_buffer = prims_buffer, .dst_buffer = staging, .size = size});
    auto cmds2 = rec2.complete_current_commands();
    device.submit_commands({.command_lists = std::array{cmds2}});
    device.wait_idle();
    Aabb const *gpu = device.buffer_host_address_as<Aabb>(staging).value();
    daxa_u32 mismatches = 0u, checked = 0u;
    double max_diff = 0.0;
    // compare every entry: voxel ranges are GPU-written, cube bodies' single boxes are
    // CPU-written in both and match trivially
    for (size_t i = 0; i < cpu_reference.size(); ++i)
    {
      double const d = std::max({std::abs((double)gpu[i].minimum.x - (double)cpu_reference[i].minimum.x),
                                 std::abs((double)gpu[i].minimum.y - (double)cpu_reference[i].minimum.y),
                                 std::abs((double)gpu[i].minimum.z - (double)cpu_reference[i].minimum.z),
                                 std::abs((double)gpu[i].maximum.x - (double)cpu_reference[i].maximum.x),
                                 std::abs((double)gpu[i].maximum.y - (double)cpu_reference[i].maximum.y),
                                 std::abs((double)gpu[i].maximum.z - (double)cpu_reference[i].maximum.z)});
      if (d > 1e-5) { ++mismatches; }
      max_diff = std::max(max_diff, d);
      ++checked;
    }
    std::cout << "[PRIMS-VERIFY] entries=" << checked << " mismatches=" << mismatches
              << " max|gpu-cpu|=" << max_diff
              << ((mismatches == 0u) ? "  => MATCH" : "  => MISMATCH!") << std::endl;
    device.destroy_buffer(staging);
  }
}

void RigidBodyManager::carve_and_label(VoxelShape const &shape, daxa_f32vec3 carve_center_grid, daxa_f32 carve_radius_grid,
                                       std::vector<daxa_f32vec4> const &sites, daxa_f32 voronoi_radius_grid,
                                       std::vector<daxa_u32> &out_occ_words, std::vector<daxa_u32> &out_labels)
{
  if (!initialized) { return; }
  daxa_u32 const cells = shape.dims.x * shape.dims.y * shape.dims.z;
  daxa_u32 const words = (cells + 31u) / 32u;
  daxa_u32 const site_count = std::min<daxa_u32>((daxa_u32)sites.size(), BB_MAX_FRACTURE_SITES);
  bool const use_voronoi = site_count > 0u;
  if (use_voronoi)
  {
    std::memcpy(device.buffer_host_address_as<daxa_f32vec4>(fracture_sites_buffer).value(),
                sites.data(), site_count * sizeof(daxa_f32vec4));
  }
  VoxelFracturePushConstants pc = {
      .occupancy_addr = device.device_address(voxel_occupancy).value(),
      .labels_addr = device.device_address(voxel_sdf_scratch[0]).value(),
      .site_labels_addr = device.device_address(voxel_sdf_scratch[1]).value(),
      .site_pos_addr = device.device_address(fracture_sites_buffer).value(),
      .cell_dims = shape.dims,
      .occ_offset = shape.occ_offset,
      .carve_center = carve_center_grid,
      .carve_radius = carve_radius_grid,
      .voronoi_radius = voronoi_radius_grid,
      .site_count = site_count,
      .use_voronoi = use_voronoi ? 1u : 0u,
  };
  auto rec = device.create_command_recorder({});
  auto const barrier = [&rec]() {
    rec.pipeline_barrier({
        .src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
        .dst_access = daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE,
    });
  };
  daxa_u32 const groups = (cells + 63u) / 64u;
  if (carve_radius_grid > 0.0f)
  {
    rec.set_pipeline(*pipeline_VFR_CARVE);
    rec.push_constant(pc);
    rec.dispatch({.x = groups, .y = 1, .z = 1});
    barrier();
  }
  if (use_voronoi)
  {
    // nearest-site assignment (reads the post-carve occupancy) before the flood
    rec.set_pipeline(*pipeline_VFR_VORONOI);
    rec.push_constant(pc);
    rec.dispatch({.x = groups, .y = 1, .z = 1});
    barrier();
  }
  rec.set_pipeline(*pipeline_VFR_FLOOD_INIT);
  rec.push_constant(pc);
  rec.dispatch({.x = groups, .y = 1, .z = 1});
  barrier();
  // min-propagation + pointer jumping: reach roughly doubles per iteration, so
  // 2*ceil(log2(cells)) + slack converges for any component shape. FIXED count (no
  // early-out readback) keeps the pass deterministic and single-submit.
  daxa_u32 iters = 8u;
  for (daxa_u32 c = cells; c > 1u; c >>= 1u) { iters += 2u; }
  rec.set_pipeline(*pipeline_VFR_FLOOD_STEP);
  for (daxa_u32 it = 0u; it < iters; ++it)
  {
    rec.push_constant(pc);
    rec.dispatch({.x = groups, .y = 1, .z = 1});
    barrier();
  }
  auto cmds = rec.complete_current_commands();
  device.submit_commands({.command_lists = std::array{cmds}});
  device.wait_idle();

  // readbacks (fracture-rate one-offs): carved occupancy words + per-cell labels
  auto const occ_bytes = (u64)words * sizeof(daxa_u32);
  auto const lbl_bytes = (u64)cells * sizeof(daxa_u32);
  daxa::BufferId staging = device.create_buffer({
      .size = occ_bytes + lbl_bytes,
      .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
      .name = "fracture_readback_staging",
  });
  auto rec2 = device.create_command_recorder({});
  rec2.copy_buffer_to_buffer({.src_buffer = voxel_occupancy, .dst_buffer = staging,
                              .src_offset = (u64)shape.occ_offset * sizeof(daxa_u32), .size = occ_bytes});
  rec2.copy_buffer_to_buffer({.src_buffer = voxel_sdf_scratch[0], .dst_buffer = staging,
                              .dst_offset = occ_bytes, .size = lbl_bytes});
  auto cmds2 = rec2.complete_current_commands();
  device.submit_commands({.command_lists = std::array{cmds2}});
  device.wait_idle();
  daxa_u32 const *host = device.buffer_host_address_as<daxa_u32>(staging).value();
  out_occ_words.assign(host, host + words);
  out_labels.assign(host + words, host + words + cells);
  device.destroy_buffer(staging);
}

void RigidBodyManager::read_voxel_derived(daxa_u32 count, std::vector<VoxelShapeDerived> &out)
{
  out.clear();
  if (!initialized || count == 0u) { return; }
  auto const size = (u64)count * sizeof(VoxelShapeDerived);
  daxa::BufferId staging = device.create_buffer({
      .size = size,
      .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
      .name = "voxel_derived_readback_staging",
  });
  auto rec = device.create_command_recorder({});
  rec.copy_buffer_to_buffer({.src_buffer = voxel_derived, .dst_buffer = staging, .size = size});
  auto cmds = rec.complete_current_commands();
  device.submit_commands({.command_lists = std::array{cmds}});
  device.wait_idle();
  VoxelShapeDerived const *host = device.buffer_host_address_as<VoxelShapeDerived>(staging).value();
  out.assign(host, host + count);
  device.destroy_buffer(staging);
}

void RigidBodyManager::upload_voxel_shapes(std::vector<VoxelShape> const &shapes)
{
  if (!initialized || shapes.empty()) { return; }
  std::memcpy(device.buffer_host_address_as<VoxelShape>(voxel_shapes).value(),
              shapes.data(), shapes.size() * sizeof(VoxelShape));
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
  // NOTE: unlike every other binding here, these three resolve parity via the accel-struct
  // manager's GLOBAL get_sim_frame_index() and IGNORE the `current_frame` argument. Harmless today
  // (the graphs that call update_buffers(f) in a parity loop — update_SC_TG, ARB_TG — don't attach
  // these buffers), but a latent trap: extending either graph to touch the rigid-body buffers would
  // silently bind the global-current parity for the f=0 iteration. Add frame-indexed getter
  // overloads if that ever changes.
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
