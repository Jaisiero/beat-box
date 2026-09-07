#include "fragment_census_reference.hpp"
#include "fragment_plan_reference.hpp"
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
  if (const char *value = std::getenv("BB_TGS_SUBSTEPS"))
  {
    tgs_substep_count = static_cast<daxa_u32>(std::clamp(std::atoi(value), 1, 32));
  }
  if (device.is_valid())
  {
    narrow_phase_timing = std::getenv("BB_RESPAWN_TIMING") != nullptr || std::getenv("BB_FRAME_TIMING") != nullptr;
    if (narrow_phase_timing)
    {
      narrow_phase_queries = device.create_timeline_query_pool({.query_count = 2, .name = "fracture_narrow_phase"});
      solver_stage_queries = device.create_timeline_query_pool({.query_count = 6, .name = "solver_stages"});
    }
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
    pipeline_fragment_finalize = task_manager->create_compute(FragmentFinalizeInfo{}.info);
    pipeline_body_list = task_manager->create_compute(BodyListInfo{}.info);
    pipeline_fracture_scene_edit = task_manager->create_compute(FractureSceneEditInfo{}.info);
    pipeline_voxel_primitive_batch = task_manager->create_compute(VoxelPrimitiveBatchInfo{}.info);
    pipeline_fracture_layout = task_manager->create_compute(FractureLayoutInfo{}.info);
    pipeline_fracture_setup = task_manager->create_compute(FractureSetupInfo{}.info);
    pipeline_fracture_gather = task_manager->create_compute(FractureGatherInfo{}.info);
    pipeline_fragment_plan = task_manager->create_compute(FragmentPlanInfo{}.info);
    pipeline_impact_reset = task_manager->create_compute(FractureImpactInfo::make("entry_fracture_impact_reset"));
    pipeline_impact_select = task_manager->create_compute(FractureImpactInfo::make("entry_fracture_impact_select"));
    pipeline_impact_publish = task_manager->create_compute(FractureImpactInfo::make("entry_fracture_impact_publish"));
    pipeline_fracture_allocate = task_manager->create_compute(FractureAllocatorInfo{}.info);
    pipeline_fragment_batch_pack = task_manager->create_compute(FractureBatchPackingInfo{}.info);
    pipeline_census_init = task_manager->create_compute(FragmentCensusInitInfo{}.info);
    pipeline_census_accumulate = task_manager->create_compute(FragmentCensusAccumulateInfo{}.info);
    pipeline_census_compact = task_manager->create_compute(FragmentCensusCompactInfo{}.info);
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


void RigidBodyManager::record_active_rigid_body_list_tasks(TaskGraph &ARBL_TG)
{
  daxa::InlineTaskInfo task({
      .attachments={
          daxa::inl_attachment(daxa::TaskBufferAccess::COMPUTE_SHADER_READ,task_rigid_bodies),
          daxa::inl_attachment(daxa::TaskBufferAccess::COMPUTE_SHADER_READ_WRITE,task_active_rigid_bodies),
          daxa::inl_attachment(daxa::TaskBufferAccess::COMPUTE_SHADER_READ_WRITE,task_rigid_body_entries)},
      .task=[this](daxa::TaskInterface const &ti) {
        ti.recorder.set_pipeline(*pipeline_body_list);
        ti.recorder.push_constant(BodyListPushConstants{
            device.device_address(ti.get(task_rigid_bodies).id).value(),
            device.device_address(ti.get(task_active_rigid_bodies).id).value(),
            device.device_address(ti.get(task_rigid_body_entries).id).value(),
            renderer_manager->get_rigid_body_count()});
        ti.recorder.dispatch({.x=1u});
      },
      .name="Build active body list on GPU"});
  std::array<daxa::TaskBuffer,3> buffers{task_rigid_bodies,task_active_rigid_bodies,task_rigid_body_entries};
  std::array<daxa::InlineTaskInfo,1> tasks{task};
  ARBL_TG=task_manager->create_task_graph("GPU active body list",std::span<daxa::InlineTaskInfo>(tasks),
      std::span<daxa::TaskBuffer>(buffers),{},{},{},false,daxa::QUEUE_COMPUTE_0);
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
          .name = "voxel_occupancy",
      });
      voxel_surface = device.create_buffer({
          .size = sizeof(daxa_u32) * BB_MAX_VOXEL_SURF_COUNT,
          .name = "voxel_surface",
      });
      voxel_sdf = device.create_buffer({
          .size = sizeof(daxa_f32) * BB_MAX_VOXEL_SDF_F32S,
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
      fracture_impact_scratch = device.create_buffer({
          .size = sizeof(FractureImpactScratch), .name = "fracture_impact_scratch"});
      auto *events = device.buffer_host_address_as<FractureEventBuffer>(fracture_events_buffer).value();
      *events = FractureEventBuffer{};
      events->scratch_addr = device.device_address(fracture_impact_scratch).value();
      // FRACTURE Voronoi sites: host writes up to BB_MAX_FRACTURE_SITES grid-space positions
      // per event; the voronoi-assign kernel reads them
      fracture_remap_buffer = device.create_buffer({
          .size = sizeof(daxa_u32) * BB_MAX_VOXEL_SDF_F32S,
          .name = "fracture_label_remap",
      });
      fracture_plan_manifest = device.create_buffer({
          .size = sizeof(FragmentPlanManifest),
          .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
          .name = "fragment_build_manifest",
      });
      fracture_census_scratch = device.create_buffer({
          .size = sizeof(FragmentComponent) * BB_MAX_VOXEL_SDF_F32S,
          .name = "fracture_census_scratch",
      });
      fracture_census_output = device.create_buffer({
          .size = sizeof(FragmentCensusOutput),
          .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
          .name = "fracture_census_output",
      });
      fracture_scene_manifest=device.create_buffer({.size=sizeof(FractureSceneEditManifest),
          .memory_flags=daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,.name="fracture_scene_manifest"});
      fracture_spawn_templates=device.create_buffer({.size=sizeof(RigidBody)*BB_MAX_VOXEL_SHAPE_COUNT,.name="fracture_spawn_templates"});
      fracture_spawn_body=device.create_buffer({.size=sizeof(RigidBody),.name="fracture_spawn_body"});
      fracture_batch_manifest=device.create_buffer({.size=sizeof(FractureBatchManifest),
          .memory_flags=daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,.name="fracture_batch_manifest"});
      fracture_allocator = device.create_buffer({.size=sizeof(FractureAllocationState),.name="fracture_allocator"});
      fracture_allocations = device.create_buffer({.size=sizeof(FractureAllocationManifest),
          .memory_flags=daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,.name="fracture_allocations"});
      fracture_contexts = device.create_buffer({
          .size = sizeof(FractureParentContext) * BB_MAX_RIGID_BODY_COUNT,
          .name = "fracture_parent_contexts",
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
        .tgs_substeps = tgs_substep_count,
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
    if (narrow_phase_timing)
    {
      ti.recorder.reset_timestamps({.query_pool = narrow_phase_queries, .start_index = 0, .count = 2});
      ti.recorder.write_timestamp({.query_pool = narrow_phase_queries, .pipeline_stage = daxa::PipelineStageFlagBits::ALL_COMMANDS, .query_index = 0});
      narrow_phase_query_pending = true;
    }
    ti.recorder.set_pipeline(*pipeline_NP);
    ti.recorder.push_constant(NarrowPhasePushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(NarrowPhaseTaskHead::AT.dispatch_buffer).id, .offset = sizeof(daxa_u32vec3) * NARROW_PHASE_COLLISION_DISPATCH_COUNT_OFFSET});
    if (narrow_phase_timing)
      ti.recorder.write_timestamp({.query_pool = narrow_phase_queries, .pipeline_stage = daxa::PipelineStageFlagBits::ALL_COMMANDS, .query_index = 1});
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

  std::array<daxa::TaskBuffer, 40> buffers = {
      task_sim_config_host,
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
      daxa::attachment_view(SleepTaskHead::AT.islands, task_islands),
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

  // One recorded task per color sweep, rather than one task per color. The
  // dispatches and Gauss-Seidel order are unchanged; only host task overhead and
  // redundant pipeline binds are removed. The task head preserves dependencies
  // at sweep boundaries, including indirect-argument visibility.
  auto make_avbd_primal_sweep = [this](daxa_f32 stab_alpha, daxa_u32 ps_depth, daxa_f32 relax = 1.0f)
  {
    return [this, stab_alpha, ps_depth, relax](daxa::TaskInterface ti, auto &) {
      ti.recorder.set_pipeline(*pipeline_AVBD_PRIM);
      for (daxa_u32 c = 0u; c < BB_AVBD_MAX_BODY_COLORS; ++c)
      {
        daxa_u32 const offset = ps_depth == MAX_U32
            ? AVBD_COLOR_SOLVE_DISPATCH_OFFSET + c
            : AVBD_CASCADE_DISPATCH_OFFSET + ps_depth * BB_MAX_COLORS + c;
        ti.recorder.push_constant(AvbdPushConstants{.task_head = ti.attachment_shader_blob,
            .color = c, .stab_alpha = stab_alpha, .ps_depth = ps_depth, .relax = relax});
        ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(AvbdTaskHead::AT.dispatch_buffer).id,
            .offset = sizeof(daxa_u32vec3) * offset});
        if (c + 1u < BB_AVBD_MAX_BODY_COLORS)
        {
          // Later colors read positions/rotations written by earlier colors.
          // Global compute visibility also covers SimConfig residual writes.
          // Indirect arguments are read-only throughout this sweep.
          ti.recorder.pipeline_barrier({.src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
                                        .dst_access = daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
        }
      }
      // The task graph supplies the final dependency to the next sweep/dual.
    };
  };
  using TTask_AVBD_SWEEP = TaskTemplate<AvbdTaskHead::Task, decltype(make_avbd_primal_sweep(1.0f, MAX_U32))>;
  TTask_AVBD_SWEEP task_AVBD_PRIM(avbd_views, make_avbd_primal_sweep(1.0f, MAX_U32));
  TTask_AVBD_SWEEP task_AVBD_PRIM_PS_plain(avbd_views, make_avbd_primal_sweep(0.0f, MAX_U32));
  TTask_AVBD_SWEEP task_AVBD_PRIM_PS_relax(avbd_views, make_avbd_primal_sweep(0.0f, MAX_U32, BB_AVBD_PS_RELAX));
  std::vector<TTask_AVBD_SWEEP> task_AVBD_PRIM_PS_vec;
  task_AVBD_PRIM_PS_vec.reserve(BB_AVBD_SHOCK_LAYERS);
  for (daxa_u32 d = 0u; d < BB_AVBD_SHOCK_LAYERS; ++d)
    task_AVBD_PRIM_PS_vec.emplace_back(avbd_views, make_avbd_primal_sweep(0.0f, d));

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

  // TGS keeps the same color dispatches and temporal order, but records a whole
  // sweep as one task. Overflow remains a separate ordered task after each sweep.
  auto make_tgs_sweep = [this](std::shared_ptr<daxa::ComputePipeline> pl, daxa_i32 phase) {
    return [this, pl, phase](daxa::TaskInterface ti, auto &) {
      ti.recorder.set_pipeline(*pl);
      for (daxa_u32 c = 0u; c < BB_MAX_COLORS_SOLVE; ++c)
      {
        ti.recorder.push_constant(GraphColorSolvePushConstants{
            .task_head = ti.attachment_shader_blob, .color = c, .tgs_phase = phase});
        ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(GraphColorSolveTaskHead::AT.dispatch_buffer).id,
            .offset = sizeof(daxa_u32vec3) * (GRAPH_COLOR_SOLVE_DISPATCH_OFFSET + c)});
        if (c + 1u < BB_MAX_COLORS_SOLVE)
        {
          // Adjacent colors may share bodies. Publish velocities and manifold
          // impulses before the next color; arguments and color IDs are read-only.
          ti.recorder.pipeline_barrier({.src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
                                        .dst_access = daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
        }
      }
      // GraphColorSolveTaskHead supplies the final dependency to overflow,
      // integration or the following sweep, including indirect argument access.
    };
  };
  using TTask_TGS_SWEEP = TaskTemplate<GraphColorSolveTaskHead::Task, decltype(make_tgs_sweep(pipeline_GCS_CS, 1))>;
  TTask_TGS_SWEEP task_TGS_CPS(gc_solve_views, make_tgs_sweep(pipeline_GCS_CPS, 1));
  TTask_TGS_SWEEP task_TGS_WS(gc_solve_views, make_tgs_sweep(pipeline_GCS_CPS, 2));
  TTask_TGS_SWEEP task_TGS_CS(gc_solve_views, make_tgs_sweep(pipeline_GCS_CS, 1));
  TTask_TGS_SWEEP task_TGS_CSR(gc_solve_views, make_tgs_sweep(pipeline_GCS_CSR, 1));

  // overflow bucket: serial single-thread solve of manifolds the per-color dispatches skip
  // (uncolored / color>=MAX_COLORS_SOLVE). Early-outs on graph_color_overflow==0, so it is
  // free except on degenerate frames.
  auto make_gcs_ov = [this](std::shared_ptr<daxa::ComputePipeline> pl, daxa_i32 tgs_phase = 0) {
    return [this, pl, tgs_phase](daxa::TaskInterface ti, auto &) {
      ti.recorder.set_pipeline(*pl);
      ti.recorder.push_constant(GraphColorSolvePushConstants{.task_head = ti.attachment_shader_blob, .color = (tgs_phase != 0 && beat_box_diagnostics::options().tgs_serial) ? MAX_U32 : 0u, .tgs_phase = tgs_phase});
      ti.recorder.dispatch({.x = 1, .y = 1, .z = 1});
    };
  };
  using TTask_GCS_OV = TaskTemplate<GraphColorSolveTaskHead::Task, decltype(make_gcs_ov(pipeline_GCS_CS_OV))>;
  TTask_GCS_OV task_GCS_CPS_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CPS_OV));
  TTask_GCS_OV task_GCS_CS_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CS_OV));
  TTask_GCS_OV task_GCS_CSR_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CSR_OV));
  TTask_GCS_OV task_TGS_CPS_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CPS_OV, 1));
  TTask_GCS_OV task_TGS_WS_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CPS_OV, 2));
  TTask_GCS_OV task_TGS_CS_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CS_OV, 1));
  TTask_GCS_OV task_TGS_CSR_OV(gc_solve_views, make_gcs_ov(pipeline_GCS_CSR_OV, 1));

  // Raw scratch addresses are shared with both solvers. Explicit memory barriers
  // cover these accesses; attachments anchor the passes in the simulation chain.
  auto impact_task = [this](bool reset) {
    return daxa::InlineTaskInfo{
      .attachments = {
        daxa::inl_attachment(daxa::TaskBufferAccess::COMPUTE_SHADER_READ_WRITE, task_sim_config),
        daxa::inl_attachment(daxa::TaskBufferAccess::COMPUTE_SHADER_READ, task_rigid_bodies),
        daxa::inl_attachment(daxa::TaskBufferAccess::COMPUTE_SHADER_READ, task_rigid_body_link_manifolds),
        daxa::inl_attachment(daxa::TaskBufferAccess::COMPUTE_SHADER_READ, task_collision_entries),
        daxa::inl_attachment(daxa::TaskBufferAccess::COMPUTE_SHADER_READ, task_collisions),
      },
      .task = [this, reset](daxa::TaskInterface const &ti) {
        ti.recorder.pipeline_barrier({.src_access = daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE,
                                      .dst_access = daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
        FractureImpactPushConstants pc{
          .config_addr = device.device_address(ti.get(task_sim_config).id).value(),
          .bodies_addr = device.device_address(ti.get(task_rigid_bodies).id).value(),
          .nodes_addr = device.device_address(ti.get(task_rigid_body_link_manifolds).id).value(),
          .map_addr = device.device_address(ti.get(task_collision_entries).id).value(),
          .manifolds_addr = device.device_address(ti.get(task_collisions).id).value(),
        };
        ti.recorder.set_pipeline(*(reset ? pipeline_impact_reset : pipeline_impact_select));
        ti.recorder.push_constant(pc);
        ti.recorder.dispatch({.x = (reset ? BB_MAX_COLLISION_COUNT : BB_MAX_RIGID_BODY_COUNT) / 64u});
        ti.recorder.pipeline_barrier({.src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
                                      .dst_access = daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
        if (!reset) {
          ti.recorder.set_pipeline(*pipeline_impact_publish);
          ti.recorder.push_constant(pc);
          ti.recorder.dispatch({.x = 1u});
          ti.recorder.pipeline_barrier({.src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
                                        .dst_access = daxa::AccessConsts::HOST_READ});
        }
      },
      .name = reset ? "Reset fracture impacts" : "Select ordered fracture impacts",
    };
  };

  // Per-solver task graph: shared setup (broad/narrow/islands/sleeping) + ONLY the active solver's
  // passes. simulate() runs the one matching solver_type, so PGS/TGS no longer pay AVBD's ~847
  // dispatches/frame (the cross-solver overhead that made them slow since AVBD landed).
  auto record_solve = [&](TaskGraph &G, SimSolverType solver)
  {
  auto profile_point = [&](daxa_u32 index) {
    if (!narrow_phase_timing || (solver != SimSolverType::AVBD && solver != SimSolverType::TGS_SOFT)) return;
    G.add_task(daxa::InlineTaskInfo{
      // Anchor the marker to the ordered simulation chain. An attachment-free
      // task could be rescheduled and would not measure the intended boundary.
      .attachments = {daxa::inl_attachment(daxa::TaskBufferAccess::COMPUTE_SHADER_READ_WRITE, task_sim_config)},
      .task = [this, index, solver](daxa::TaskInterface const &ti) {
        if (index == 0u)
          ti.recorder.reset_timestamps({.query_pool = solver_stage_queries, .start_index = 0, .count = 6});
        ti.recorder.write_timestamp({.query_pool = solver_stage_queries, .pipeline_stage = daxa::PipelineStageFlagBits::ALL_COMMANDS, .query_index = index});
        if (index == 5u) { solver_stage_query_pending = true; stage_query_solver = solver; }
      },
      .name = "Solver profiling boundary",
    });
  };
  profile_point(0u);
  G.add_task(task_PS); // mouse pick-and-drag spring (velocity injection BEFORE the step)
  G.add_task(task_RC);
  G.add_task(impact_task(true));
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
  profile_point(1u);
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
  profile_point(2u);
  for (daxa_u32 it = 0u; it < BB_AVBD_ITERATIONS; ++it)
  {
    G.add_task(task_AVBD_PRIM);
    G.add_task(task_AVBD_DUAL);
  }
  } // end AVBD primal/dual
  if (solver == SimSolverType::AVBD) profile_point(3u);
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
    G.add_task(task_AVBD_PRIM_PS_vec[d]);
  }
  for (daxa_u32 ps = 1u; ps < BB_AVBD_POST_STAB_SWEEPS; ++ps)
  {
    G.add_task(task_AVBD_PRIM_PS_plain);
  }
  // EXTRA damped sweeps - knob currently 0 (MEASURED WORSE, see BB_AVBD_POST_STAB_RELAXED
  // in shared.inl); if constexpr keeps the falsified-but-kept mechanism from emitting the
  // always-false-loop warning (C4296) while it sits parked.
  if constexpr (BB_AVBD_POST_STAB_RELAXED > 0u)
  {
    for (daxa_u32 ps = 0u; ps < BB_AVBD_POST_STAB_RELAXED; ++ps)
    {
      G.add_task(task_AVBD_PRIM_PS_relax);
    }
  }
  } // end AVBD FIN/impact/post-stab
  if (solver == SimSolverType::AVBD) profile_point(4u);
  if (solver == SimSolverType::TGS_SOFT)
  {
  profile_point(2u);
  // Diagnostic: change convergence work while keeping dt, substeps and contact refresh fixed.
  daxa_u32 const tgs_sweeps = std::getenv("BB_TGS_SWEEPS")
      ? static_cast<daxa_u32>(std::clamp(std::atoi(std::getenv("BB_TGS_SWEEPS")), 1, 16)) : 2u;
  // TGS_SOFT (Box2D v3 / solver2d): sub-stepped soft solver, integrated with graph coloring.
  // This block is recorded only in the TGS graph.
  // Prepare once (soft coeffs at sub-step h + local anchors), then BB_TGS_SUBSTEPS sub-steps of:
  // integrate velocity (gravity*h) -> per-color WARM START -> per-color solve (bias) ->
  // integrate positions (x+=v*h) -> per-color relax (no bias). The separation is re-derived from
  // the pose each sub-step (TGS temporal). This mirrors Box2D v3's stage order exactly; the warm
  // start in particular belongs INSIDE the loop (b2WarmStartContactsTask runs per sub-step) and is
  // what carries the contact load across sub-steps.
  if (!beat_box_diagnostics::options().tgs_serial)
    G.add_task(task_TGS_CPS);
  G.add_task(task_TGS_CPS_OV);
  profile_point(3u);
  for (daxa_u32 s = 0u; s < tgs_substep_count; ++s)
  {
    G.add_task(task_tgs_advect);
    if (!beat_box_diagnostics::options().tgs_serial)
      G.add_task(task_TGS_WS);
    G.add_task(task_TGS_WS_OV);
    for (daxa_u32 sweep = 0u; sweep < tgs_sweeps; ++sweep)
    {
      if (!beat_box_diagnostics::options().tgs_serial)
        G.add_task(task_TGS_CS);
      G.add_task(task_TGS_CS_OV);
    }
    G.add_task(task_tgs_ip);
    for (daxa_u32 sweep = 0u; sweep < tgs_sweeps; ++sweep)
    {
      if (!beat_box_diagnostics::options().tgs_serial)
        G.add_task(task_TGS_CSR);
      G.add_task(task_TGS_CSR_OV);
    }
  }
  profile_point(4u);
  } // end TGS sub-step loop
  if (solver == SimSolverType::AVBD && std::getenv("BB_POCKET_TRACE"))
    G.add_task(task_AVBD_PKTR); // diagnostic (AVBD only)
  G.add_task(impact_task(false));
  G.add_task(task_CP);
  G.add_task(task_update);
  profile_point(5u);
  G.add_task(sim_config_readback_task());
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

  record_active_rigid_body_list_tasks(ARB_TG);
  ARB_TG.submit();
  ARB_TG.complete();

  return initialized = true;
}

daxa::InlineTaskInfo RigidBodyManager::sim_config_readback_task()
{
  return daxa::InlineTaskInfo({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_sim_config),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_sim_config_host),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        // Explicit device-to-transfer and transfer-to-host dependencies for readback.
        ti.recorder.pipeline_barrier({.src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
                                     .dst_access = daxa::AccessConsts::TRANSFER_READ});
        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = ti.get(task_sim_config).id,
            .dst_buffer = ti.get(task_sim_config_host).id,
            .size = sizeof(SimConfig),
        });
        ti.recorder.pipeline_barrier({.src_access = daxa::AccessConsts::TRANSFER_WRITE,
                                     .dst_access = daxa::AccessConsts::HOST_READ});
        ti.recorder.pipeline_barrier({.src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
                                     .dst_access = daxa::AccessConsts::HOST_READ});
      },
      .name = "read back sim config",
  });

}

void RigidBodyManager::record_read_back_sim_config_tasks(TaskGraph &out_readback_SC_TG)
{
  std::array<daxa::TaskBuffer, 2> buffers = {
      task_sim_config,
      task_sim_config_host,
  };

  std::array<daxa::InlineTaskInfo, 1> tasks = {
      sim_config_readback_task(),
  };

  out_readback_SC_TG = task_manager->create_task_graph("Read back Simulation Configuration", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, {}, {}, false, daxa::QUEUE_COMPUTE_0);
}

void RigidBodyManager::record_update_sim_config_tasks(TaskGraph &out_update_SC_TG)
{
  task_sim_config_upload.set_buffer(sim_config_host_buffer[0]); // compile-time placeholder
  daxa::InlineTaskInfo task_update_SC({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_sim_config_upload),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_sim_config),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        ti.recorder.copy_buffer_to_buffer({
            .src_buffer=ti.get(task_sim_config_upload).id,
            .dst_buffer=ti.get(task_sim_config).id,.size=sizeof(SimConfig)});
      },
      .name = "update sim config",
  });

  std::array<daxa::TaskBuffer, 2> buffers = {
      task_sim_config,
      task_sim_config_upload,
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
  if (!fracture_impact_scratch.is_empty()) { device.destroy_buffer(fracture_impact_scratch); fracture_impact_scratch = {}; }
  for (auto *buffer:{&fracture_scene_manifest,&fracture_spawn_templates,&fracture_spawn_body,&fracture_batch_manifest})
    if (!buffer->is_empty()) { device.destroy_buffer(*buffer);*buffer={}; }
  if (!fracture_allocator.is_empty()) { device.destroy_buffer(fracture_allocator);fracture_allocator={}; }
  if (!fracture_allocations.is_empty()) { device.destroy_buffer(fracture_allocations);fracture_allocations={}; }
  if (!fracture_contexts.is_empty()) { device.destroy_buffer(fracture_contexts); fracture_contexts = {}; }
  if (!fracture_plan_manifest.is_empty()) { device.destroy_buffer(fracture_plan_manifest); fracture_plan_manifest = {}; }
  if (!fracture_remap_buffer.is_empty()) { device.destroy_buffer(fracture_remap_buffer); fracture_remap_buffer = {}; }
  if (!fracture_census_scratch.is_empty()) { device.destroy_buffer(fracture_census_scratch); fracture_census_scratch = {}; }
  if (!fracture_census_output.is_empty()) { device.destroy_buffer(fracture_census_output); fracture_census_output = {}; }

  sim_config_uploads.clear();
  sim_config_upload_cursor = 0;
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

  if (task_rigid_bodies.id() != accel_struct_mngr->get_rigid_body_buffer()) { task_rigid_bodies.set_buffer(accel_struct_mngr->get_rigid_body_buffer()); }

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
        .tgs_substeps = tgs_substep_count,
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

    // Never overwrite staging consumed by an earlier publication. Slots are
    // reused only after the renderer's existing device completion boundary.
    if (sim_config_upload_cursor == sim_config_uploads.size())
      sim_config_uploads.push_back(create_owned({.size=sizeof(SimConfig),
          .memory_flags=daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,.name="Immutable simulation upload"}));
    auto const upload = sim_config_uploads[sim_config_upload_cursor++];
    *device.buffer_host_address_as<SimConfig>(upload).value() =
        *device.buffer_host_address_as<SimConfig>(sim_config_host_buffer[f]).value();
    task_sim_config_upload.set_buffer(upload);
    update_buffers(f);
    update_SC_TG.execute();
  }

  update_buffers(); // restore current-parity bindings

  return initialized;
}

void RigidBodyManager::build_voxel_pools_gpu(std::vector<VoxelShape> const &shapes,
                                             std::vector<daxa_f32> const &cpu_sdf_reference,
                                             std::vector<daxa_u32> const &cpu_surf_reference,
                                             std::vector<VoxelShapeDerived> const &cpu_derived_reference,
                                             std::vector<daxa_u32> const *dirty)
{
  if (!initialized || shapes.empty()) { return; }
  auto const occ_addr = device.device_address(voxel_occupancy).value();
  auto const sdf_addr = device.device_address(voxel_sdf).value();
  auto const sc0_addr = device.device_address(voxel_sdf_scratch[0]).value();
  auto const sc1_addr = device.device_address(voxel_sdf_scratch[1]).value();
  auto const shapes_addr = device.device_address(voxel_shapes).value();
  auto const surf_addr = device.device_address(voxel_surface).value();
  auto const derived_addr = device.device_address(voxel_derived).value();

  // INCREMENTAL (phase 2b-AS): rebuild only the shapes that changed this fracture, not all
  // of them - a fracture touches ~5 shapes out of dozens, and an unchanged shape's SDF /
  // surface / inertia are already correct on the GPU. dirty=nullptr => rebuild all (load).
  std::vector<daxa_u32> idx;
  if (dirty) { idx = *dirty; }
  else { idx.resize(shapes.size()); for (daxa_u32 i = 0u; i < (daxa_u32)shapes.size(); ++i) { idx[i] = i; } }
  if (idx.empty()) { return; }

  bool const profile_build = std::getenv("BB_RESPAWN_TIMING") != nullptr;
  daxa::TimelineQueryPool build_queries = {};
  if (profile_build)
    build_queries = device.create_timeline_query_pool({.query_count = 2, .name = "voxel_pool_build"});
  auto rec = device.create_command_recorder({});
  rec.pipeline_barrier({.src_access = daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE,
                       .dst_access = daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
  if (profile_build)
  {
    rec.reset_timestamps({.query_pool = build_queries, .start_index = 0, .count = 2});
    rec.write_timestamp({.query_pool = build_queries, .pipeline_stage = daxa::PipelineStageFlagBits::ALL_COMMANDS, .query_index = 0});
  }
  auto const barrier = [&rec]() {
    rec.pipeline_barrier({
        .src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
        .dst_access = daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE,
    });
  };
  for (daxa_u32 si : idx)
  {
    if (si >= shapes.size()) { continue; }
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
  rec.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE,
                        .dst_access = daxa::AccessConsts::READ});
  rec.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE,
                        .dst_access = daxa::AccessConsts::HOST_READ});
  if (profile_build)
    rec.write_timestamp({.query_pool = build_queries, .pipeline_stage = daxa::PipelineStageFlagBits::ALL_COMMANDS, .query_index = 1});
  auto cmds = rec.complete_current_commands();
  device.submit_commands({.command_lists = std::array{cmds}});
  // The following publication/build submissions use MAIN too. Only a host
  // timestamp read needs completion here; data dependencies stay on the GPU.
  if (profile_build)
  {
    device.wait_on_submit({.queue = daxa::QUEUE_MAIN,
        .queue_submit_index = device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
    auto const results = build_queries.get_query_results(0, 2);
    if (results[1] != 0u && results[3] != 0u)
    {
      double const ms = double(results[2] - results[0]) * device.properties().limits.timestamp_period / 1.0e6;
      std::cout << "[SDF-BUILD] gpu_ms=" << ms << " shapes=" << idx.size() << std::endl;
    }
  }

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
    rec2.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE,
                           .dst_access = daxa::AccessConsts::TRANSFER_READ});
    rec2.copy_buffer_to_buffer({.src_buffer = voxel_sdf, .dst_buffer = staging, .size = size});
    rec2.pipeline_barrier({.src_access = daxa::AccessConsts::TRANSFER_WRITE,
                           .dst_access = daxa::AccessConsts::HOST_READ});
    auto cmds2 = rec2.complete_current_commands();
    device.submit_commands({.command_lists = std::array{cmds2}});
    device.wait_on_submit({.queue = daxa::QUEUE_MAIN,
      .queue_submit_index = device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
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
    rec2.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE,
                           .dst_access = daxa::AccessConsts::TRANSFER_READ});
    rec2.copy_buffer_to_buffer({.src_buffer = voxel_surface, .dst_buffer = staging, .size = surf_size});
    rec2.copy_buffer_to_buffer({.src_buffer = voxel_shapes, .dst_buffer = staging, .dst_offset = surf_size, .size = shapes_size});
    rec2.pipeline_barrier({.src_access = daxa::AccessConsts::TRANSFER_WRITE,
                           .dst_access = daxa::AccessConsts::HOST_READ});
    auto cmds2 = rec2.complete_current_commands();
    device.submit_commands({.command_lists = std::array{cmds2}});
    device.wait_on_submit({.queue = daxa::QUEUE_MAIN,
      .queue_submit_index = device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
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
    rec2.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE,
                           .dst_access = daxa::AccessConsts::TRANSFER_READ});
    rec2.copy_buffer_to_buffer({.src_buffer = voxel_derived, .dst_buffer = staging, .size = size});
    rec2.pipeline_barrier({.src_access = daxa::AccessConsts::TRANSFER_WRITE,
                           .dst_access = daxa::AccessConsts::HOST_READ});
    auto cmds2 = rec2.complete_current_commands();
    device.submit_commands({.command_lists = std::array{cmds2}});
    device.wait_on_submit({.queue = daxa::QUEUE_MAIN,
      .queue_submit_index = device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
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
                                             std::vector<Aabb> const &cpu_reference,
                                             daxa::BufferId publication_bodies, daxa::BufferId publication_instances,
                                             daxa_u32 live_count)
{
  if (!initialized || (bodies.empty() && publication_bodies.is_empty())) { return; }
  auto const occ_addr = device.device_address(voxel_occupancy).value();
  auto const shapes_addr = device.device_address(voxel_shapes).value();
  auto const prims_addr = device.device_address(prims_buffer).value();

  auto rec = device.create_command_recorder({});
  if (!publication_bodies.is_empty())
  {
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
    rec.set_pipeline(*pipeline_fracture_gather);
    FractureGatherPushConstants gather{
        .source_addr=device.device_address(accel_struct_mngr->get_rigid_body_buffer()).value(),
        .target_addr=device.device_address(publication_bodies).value(),
        .instances_addr=device.device_address(publication_instances).value(),
        .source_count=live_count,.target_count=renderer_manager->get_rigid_body_count()};
    rec.push_constant(gather);rec.dispatch({.x=(live_count+63u)/64u});
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
    FracturePublicationPushConstants publication{
        .bodies_addr=gather.target_addr,.instances_addr=gather.instances_addr,
        .shapes_addr=shapes_addr,.derived_addr=device.device_address(voxel_derived).value(),
        .contexts_addr=device.device_address(fracture_contexts).value(),
        .allocator_addr=device.device_address(fracture_allocator).value(),
        .spawn_addr=device.device_address(fracture_spawn_body).value(),.primitives_addr=prims_addr,
        .body_count=gather.target_count};
    rec.set_pipeline(*pipeline_fragment_finalize);rec.push_constant(publication);
    rec.dispatch({.x=(publication.body_count+63u)/64u});
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
    rec.set_pipeline(*pipeline_fracture_layout);rec.push_constant(publication);rec.dispatch({.x=1u});
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
  }
  // Retain the per-body implementation for initial loading and the GPU oracle.
  auto record_reference = [&](auto &commands, daxa::BufferId target) {
    for (auto const &[shape_index, prim_offset] : bodies)
    {
      auto const &shape = shapes[shape_index];
      daxa_u32 const cells = shape.dims.x * shape.dims.y * shape.dims.z;
      VoxelSdfBuildPushConstants pc = {
          .occupancy_addr = occ_addr,
          .sdf_addr = 0u,
          .scratch_solid_addr = 0u,
          .scratch_empty_addr = 0u,
          .shapes_addr = shapes_addr,
          .surface_addr = 0u,
          .derived_addr = 0u,
          .prims_addr = device.device_address(target).value(),
          .cell_dims = shape.dims,
          .occ_offset = shape.occ_offset,
          .sdf_offset = 0u,
          .surf_offset = 0u,
          .shape_index = shape_index,
          .prims_offset = prim_offset,
          .axis = 0u,
          .voxel_size = shape.voxel_size,
      };
      commands.set_pipeline(*pipeline_VSB_PRIMS);
      commands.push_constant(pc);
      commands.dispatch({.x = (cells + 63u) / 64u, .y = 1, .z = 1});
    }
  };
  if (!publication_bodies.is_empty())
  {
    rec.set_pipeline(*pipeline_voxel_primitive_batch);
    rec.push_constant(VoxelPrimitiveBatchPushConstants{
        .bodies_addr=device.device_address(publication_bodies).value(),
        .shapes_addr=shapes_addr,.occupancy_addr=occ_addr,.primitives_addr=prims_addr,
        .body_count=renderer_manager->get_rigid_body_count()});
    rec.dispatch({.x=renderer_manager->get_rigid_body_count()});
  }
  else { record_reference(rec,prims_buffer); }
  // make the writes visible to the AS build that follows this call
  rec.pipeline_barrier({
      .src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
      .dst_access = daxa::AccessConsts::ACCELERATION_STRUCTURE_BUILD_READ,
  });
  rec.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE,
                        .dst_access = daxa::AccessConsts::READ});
  rec.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE,
                        .dst_access = daxa::AccessConsts::HOST_READ});
  auto cmds = rec.complete_current_commands();
  device.submit_commands({.command_lists = std::array{cmds}});
  if (!publication_bodies.is_empty() && std::getenv("BB_FRAGMENT_VERIFY"))
  {
    device.wait_on_submit({.queue=daxa::QUEUE_MAIN,.queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
    auto const *published=device.buffer_host_address_as<RigidBody>(publication_bodies).value();
    daxa_u32 count=0u;
    for (daxa_u32 id=0u;id<renderer_manager->get_rigid_body_count();++id)
      count=std::max(count,published[id].primitive_offset+published[id].primitive_count);
    if (count>0u)
    {
      auto const bytes=static_cast<daxa::usize>(count)*sizeof(Aabb);
      auto expected=device.create_buffer({.size=bytes,.memory_flags=daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,.name="primitive batch reference"});
      auto actual=device.create_buffer({.size=bytes,.memory_flags=daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,.name="primitive batch actual"});
      auto check=device.create_command_recorder({});
      check.pipeline_barrier({.src_access=daxa::AccessConsts::WRITE,.dst_access=daxa::AccessConsts::TRANSFER_READ});
      check.copy_buffer_to_buffer({.src_buffer=prims_buffer,.dst_buffer=expected,.size=bytes});
      check.copy_buffer_to_buffer({.src_buffer=prims_buffer,.dst_buffer=actual,.size=bytes});
      check.pipeline_barrier({.src_access=daxa::AccessConsts::TRANSFER_WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE});
      record_reference(check,expected);
      check.pipeline_barrier({.src_access=daxa::AccessConsts::WRITE,.dst_access=daxa::AccessConsts::HOST_READ});
      auto commands=check.complete_current_commands();device.submit_commands({.command_lists=std::array{commands}});
      device.wait_on_submit({.queue=daxa::QUEUE_MAIN,.queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
      bool same=std::memcmp(device.buffer_host_address(expected).value(),device.buffer_host_address(actual).value(),bytes)==0;
      device.destroy_buffer(expected);device.destroy_buffer(actual);
      if (!same) { std::cerr << "[PRIMITIVE-BATCH-VERIFY] FAILED" << std::endl;std::abort(); }
      std::cout << "[PRIMITIVE-BATCH-VERIFY] count=" << count << " exact MATCH" << std::endl;
    }
  }
  // AS_build_TG follows on MAIN. Host inspection is an opt-in oracle only.
  if (std::getenv("BB_FRAGMENT_VERIFY") || std::getenv("BB_CENSUS_VERIFY"))
    device.wait_on_submit({.queue=daxa::QUEUE_MAIN,
        .queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});

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
    rec2.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE,
                           .dst_access = daxa::AccessConsts::TRANSFER_READ});
    rec2.copy_buffer_to_buffer({.src_buffer = prims_buffer, .dst_buffer = staging, .size = size});
    rec2.pipeline_barrier({.src_access = daxa::AccessConsts::TRANSFER_WRITE,
                           .dst_access = daxa::AccessConsts::HOST_READ});
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

void RigidBodyManager::record_fragment_census(daxa::CommandRecorder &rec, daxa_u32vec3 dims,
    daxa_u64 labels_addr, bool compact, daxa_u32 body_id)
{
    FragmentCensusPushConstants census_pc = {
        .labels_addr = labels_addr,
        .scratch_addr = device.device_address(fracture_census_scratch).value(),
        .output_addr = device.device_address(fracture_census_output).value(),
        .dims = dims,
        .context_addr = body_id == MAX_U32 ? 0u : device.device_address(fracture_contexts).value()+sizeof(FractureParentContext)*body_id,
    };
    for (auto const &pipeline : {pipeline_census_init, pipeline_census_accumulate, pipeline_census_compact})
    {
      if (!compact && pipeline == pipeline_census_compact) continue;
      rec.set_pipeline(*pipeline);
      rec.push_constant(census_pc);
      if (body_id == MAX_U32)
        rec.dispatch({.x = (dims.x*dims.y*dims.z + 63u)/64u, .y = 1, .z = 1});
      else
        rec.dispatch_indirect({.indirect_buffer=fracture_contexts,
          .offset=sizeof(FractureParentContext)*body_id+offsetof(FractureParentContext,cell_dispatch)});
      rec.pipeline_barrier({.src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
                                 .dst_access = daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
    }
}

void RigidBodyManager::record_fracture_partition(daxa::CommandRecorder &rec,
    daxa_u32 body_id, daxa_u32 body_count, daxa_u64 batch_addr, daxa_u32 recorded_passes,
    bool compact, daxa::TimelineQueryPool *queries)
{
  VoxelFracturePushConstants pc = {
      .context_addr = device.device_address(fracture_contexts).value() + sizeof(FractureParentContext)*body_id,
      .occupancy_addr = device.device_address(voxel_occupancy).value(),
      .labels_addr = device.device_address(voxel_sdf_scratch[0]).value(),
      .site_labels_addr = device.device_address(voxel_sdf_scratch[1]).value(),
      .use_voronoi = 1u,
  };
  rec.pipeline_barrier({.src_access = daxa::AccessConsts::READ_WRITE,
                       .dst_access = daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
  auto const barrier = [&rec]() {
    rec.pipeline_barrier({
        .src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
        .dst_access = daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE,
    });
  };
  daxa_u64 const dispatch_offset=sizeof(FractureParentContext)*body_id;
  auto dispatch_cells=[&]() {
    rec.dispatch_indirect({.indirect_buffer=fracture_contexts,
      .offset=dispatch_offset+offsetof(FractureParentContext,cell_dispatch)});
  };
  rec.set_pipeline(*pipeline_fracture_setup);
  rec.push_constant(FractureSetupPushConstants{
      .bodies_addr = device.device_address(accel_struct_mngr->get_rigid_body_buffer()).value(),
      .shapes_addr = device.device_address(voxel_shapes).value(),
      .contexts_addr = device.device_address(fracture_contexts).value(),
      .events_addr = device.device_address(fracture_events_buffer).value(),
      .body_id = body_id, .body_count = body_count});
  rec.dispatch({.x=1u});
  // Setup produces shader inputs AND indirect dispatch arguments.
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,
                       .dst_access=daxa::AccessConsts::READ});
  rec.set_pipeline(*pipeline_VFR_VORONOI);
  rec.push_constant(pc);
  dispatch_cells();
  barrier();
  rec.set_pipeline(*pipeline_VFR_FLOOD_INIT);
  rec.push_constant(pc);
  dispatch_cells();
  barrier();
  // Bound command recording with the existing AS metadata. Recording the
  // scene-wide maximum adds measurable overhead for small cropped children.
  // GPU setup remains the source of every indirect dispatch argument.
  if (recorded_passes>BB_FRACTURE_FLOOD_PASSES) std::abort();
  rec.set_pipeline(*pipeline_VFR_FLOOD_STEP);
  for (daxa_u32 it = 0u; it < recorded_passes; ++it)
  {
    rec.push_constant(pc);
    dispatch_cells();
    barrier();
  }
  // Census shares the existing label submission and completion boundary.
  if (queries)
  {
    rec.reset_timestamps({.query_pool = *queries, .start_index = 0, .count = 2});
    rec.write_timestamp({.query_pool = *queries, .pipeline_stage = daxa::PipelineStageFlagBits::ALL_COMMANDS, .query_index = 0});
  }
  record_fragment_census(rec, {}, pc.labels_addr, compact, body_id);
  rec.set_pipeline(*pipeline_fragment_plan);
  rec.push_constant(FragmentPlanPushConstants{
      device.device_address(fracture_census_scratch).value(),
      device.device_address(fracture_remap_buffer).value(),
      device.device_address(fracture_plan_manifest).value(), 0u, pc.context_addr});
  rec.dispatch({.x=1,.y=1,.z=1});
  barrier();
  FractureAllocatorPushConstants allocation_pc{
      .state_addr=device.device_address(fracture_allocator).value(),
      .plan_addr=device.device_address(fracture_plan_manifest).value(),
      .allocation_addr=device.device_address(fracture_allocations).value(),
      .contexts_addr=device.device_address(fracture_contexts).value(),
      .shapes_addr=device.device_address(voxel_shapes).value(),.batch_addr=batch_addr,.parent_id=body_id};
  rec.set_pipeline(*pipeline_fracture_allocate);rec.push_constant(allocation_pc);rec.dispatch({.x=1u});
  // Reservation writes feed both shader reads and indirect command fetches.
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::READ});
  rec.set_pipeline(*pipeline_fragment_batch_pack);
  rec.push_constant(FractureBatchPackingPushConstants{
      .plan_addr=allocation_pc.plan_addr,.allocation_addr=allocation_pc.allocation_addr,
      .contexts_addr=allocation_pc.contexts_addr,.labels_addr=device.device_address(voxel_sdf_scratch[0]).value(),
      .remap_addr=device.device_address(fracture_remap_buffer).value(),
      .occupancy_addr=device.device_address(voxel_occupancy).value(),.parent_id=body_id});
  rec.dispatch_indirect({.indirect_buffer=fracture_allocations,.offset=offsetof(FractureAllocationManifest,dispatch_x)});
  barrier();
  allocation_pc.operation=1u;
  rec.set_pipeline(*pipeline_fracture_allocate);rec.push_constant(allocation_pc);rec.dispatch({.x=1u});
  if (queries)
    rec.write_timestamp({.query_pool = *queries, .pipeline_stage = daxa::PipelineStageFlagBits::ALL_COMMANDS, .query_index = 1});
}

void RigidBodyManager::carve_and_label(VoxelShape const &shape, daxa_u32 body_id,
                                       std::vector<daxa_u32> &out_labels, std::vector<FragmentComponent> &out_components,
                                       std::vector<FractureChildAllocation> &allocations)
{
  if (!initialized) { return; }
  static bool events_verified = false;
  if (!events_verified && std::getenv("BB_POOL_VERIFY")) {
    verify_fracture_events_gpu(); verify_fracture_allocator_gpu(); events_verified = true;
  }
  daxa_u32 const cells = shape.dims.x * shape.dims.y * shape.dims.z;
  auto const fracture_live_count = renderer_manager->get_rigid_body_count();
  static bool allocator_verified = false;
  if (!allocator_verified && std::getenv("BB_POOL_VERIFY") != nullptr)
  {
    auto pipeline = task_manager->create_compute(GpuPoolValidationInfo{}.info);
    auto buffer = device.create_buffer({.size = sizeof(GpuPoolTransaction)+sizeof(daxa_u32),
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM, .name = "gpu_pool_validation"});
    auto *tx = device.buffer_host_address_as<GpuPoolTransaction>(buffer).value();
    *tx = {};
    for (auto &pool : tx->committed) { pool.capacity=16; pool.allocate(8); }
    auto *failures = reinterpret_cast<daxa_u32 *>(tx+1);
    *failures = MAX_U32;
    auto commands = device.create_command_recorder({});
    commands.set_pipeline(*pipeline);
    auto const address = device.device_address(buffer).value();
    commands.push_constant(GpuPoolValidationPushConstants{address,address+sizeof(GpuPoolTransaction)});
    commands.dispatch({.x=1,.y=1,.z=1});
    commands.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,
                               .dst_access=daxa::AccessConsts::HOST_READ});
    auto list=commands.complete_current_commands();
    device.submit_commands({.command_lists=std::array{list}});
    device.wait_on_submit({.queue=daxa::QUEUE_MAIN,
        .queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
    bool valid=*failures==0 && tx->epoch==1 && tx->phase==3 && tx->failed_pool==GPU_POOL_COUNT-1;
    for (auto &pool : tx->committed)
      valid = valid && pool.valid() && pool.high_water==16 && pool.live_units==8 &&
              pool.range_count==1 && pool.ranges[0].offset==0 && pool.ranges[0].size==8;
    if (!valid) { std::cerr << "[GPU-POOL-VERIFY] FAILED" << std::endl; std::abort(); }
    std::cout << "[GPU-POOL-VERIFY] reservation, retirement and rollback MATCH" << std::endl;
    device.destroy_buffer(buffer);
    allocator_verified=true;
  }
  bool const verify_census = std::getenv("BB_CENSUS_VERIFY") != nullptr;
  // Always produce a compact manifest: downloading tiny grids would restore
  // the CPU packing dependency this path removes.
  bool const read_labels = verify_census || std::getenv("BB_SDF_VERIFY") != nullptr;
  daxa::TimelineQueryPool census_queries = {};
  if (verify_census)
    census_queries = device.create_timeline_query_pool({.query_count = 2, .name = "fragment_census"});
  auto rec = device.create_command_recorder({});
  record_fracture_partition(rec,body_id,fracture_live_count,0u,fracture_recorded_passes(shape),
      verify_census,verify_census ? &census_queries : nullptr);
  // Only verification or compact-manifest overflow downloads cell labels.
  daxa::BufferId staging = {};
  auto const lbl_bytes = (u64)cells * sizeof(daxa_u32);
  auto copy_labels = [&](auto &commands) {
    staging = device.create_buffer({.size = lbl_bytes,
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "fracture_label_oracle"});
    commands.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE,
                               .dst_access = daxa::AccessConsts::TRANSFER_READ});
    commands.copy_buffer_to_buffer({.src_buffer = voxel_sdf_scratch[0], .dst_buffer = staging, .size = lbl_bytes});
  };
  if (read_labels) copy_labels(rec);
  rec.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE,
                       .dst_access = daxa::AccessConsts::HOST_READ});
  auto cmds = rec.complete_current_commands();
  // Pending impact payloads were written on COMPUTE_0. Carry their memory
  // dependency onto MAIN explicitly, even though the CPU already read the summary.
  device.submit_commands({.command_lists = std::array{cmds},
      .wait_queue_submit_indices = std::array{std::pair{daxa::QUEUE_COMPUTE_0,
          device.latest_queue_submit_index(daxa::QUEUE_COMPUTE_0)}}});
  device.wait_on_submit({.queue = daxa::QUEUE_MAIN,
      .queue_submit_index = device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
  auto const *summary = device.buffer_host_address_as<FragmentCensusOutput>(fracture_census_output).value();
  out_labels.clear();
  if (!staging.is_empty())
  {
    auto const *host = device.buffer_host_address_as<daxa_u32>(staging).value();
    out_labels.assign(host, host + cells);
  }
  auto const *manifest=device.buffer_host_address_as<FragmentPlanManifest>(fracture_plan_manifest).value();
  auto const *allocation_manifest=device.buffer_host_address_as<FractureAllocationManifest>(fracture_allocations).value();
  out_components.clear();allocations.clear();
  if (allocation_manifest->status==0u && allocation_manifest->count>1u) {
    out_components.assign(manifest->components,manifest->components+allocation_manifest->count);
    allocations.assign(allocation_manifest->children,allocation_manifest->children+allocation_manifest->count);
  } else if (allocation_manifest->status!=0u) {
    std::cerr << "FRACTURE: GPU capacity/plan refusal; whole parent retained (body " << body_id << ")" << std::endl;
    if (allocation_manifest->status==3u) { std::cerr << "GPU allocator invariant FAILED" << std::endl;std::abort(); }
  }
  if (verify_census)
  {
    auto const cpu_start = std::chrono::steady_clock::now();
    auto reference = fragment_census_reference(out_labels, shape.dims);
    double const cpu_ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now() - cpu_start).count();
    auto const query = census_queries.get_query_results(0, 2);
    if (query[1] && query[3])
      std::cout << "[CENSUS-COST] cells=" << cells << " gpu_ms="
                << double(query[2]-query[0])*device.properties().limits.timestamp_period/1e6
                << " cpu_ms=" << cpu_ms << std::endl;
    auto actual_raw=decode_fragment_census(*summary,out_labels,shape.dims);
    if (reference.size() != actual_raw.size() ||
        (!reference.empty() && std::memcmp(reference.data(), actual_raw.data(), reference.size()*sizeof(FragmentComponent))))
    {
      std::cerr << "[CENSUS-VERIFY] FAILED" << std::endl;
      std::abort();
    }
    std::cout << "[CENSUS-VERIFY] cells=" << cells << " components=" << reference.size() << " MATCH" << std::endl;
    // One-time GPU edge cases, excluded from production and benchmark runs.
    static bool edge_cases_verified = false;
    if (!edge_cases_verified)
    {
      for (daxa_u32 test = 0; test < 4; ++test)
      {
        daxa_u32vec3 dims = test == 0 ? daxa_u32vec3(7,5,3) :
                            test == 1 ? daxa_u32vec3(257,1,1) :
                            test == 2 ? daxa_u32vec3(65536,1,1) : daxa_u32vec3(1,1,1);
        std::vector<daxa_u32> labels(dims.x*dims.y*dims.z, 0u);
        for (daxa_u32 c = 0; c < labels.size(); ++c)
          labels[c] = test == 0 ? (c%3 == 0 ? MAX_U32 : c%2) :
                      test == 1 ? c : test == 2 ? 0u : MAX_U32;
        auto input = device.create_buffer({.size = labels.size()*sizeof(daxa_u32),
            .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
            .name = "census_validation_labels"});
        std::memcpy(device.buffer_host_address_as<daxa_u32>(input).value(), labels.data(), labels.size()*sizeof(daxa_u32));
        auto test_rec = device.create_command_recorder({});
        record_fragment_census(test_rec, dims, device.device_address(input).value(),true);
        test_rec.pipeline_barrier({.src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
                                    .dst_access = daxa::AccessConsts::HOST_READ});
        auto test_cmds = test_rec.complete_current_commands();
        device.submit_commands({.command_lists = std::array{test_cmds}});
        device.wait_on_submit({.queue = daxa::QUEUE_MAIN,
            .queue_submit_index = device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
        auto expected = fragment_census_reference(labels, dims);
        auto actual = decode_fragment_census(*summary, labels, dims);
        if (summary->count != expected.size() || actual.size() != expected.size() ||
            (!actual.empty() && std::memcmp(actual.data(), expected.data(), actual.size()*sizeof(FragmentComponent))))
        {
          std::cerr << "[CENSUS-VERIFY] edge case FAILED: " << test << std::endl;
          std::abort();
        }
        std::cout << "[CENSUS-VERIFY] edge=" << test << " components=" << summary->count << " MATCH" << std::endl;
        device.destroy_buffer(input);
      }
      edge_cases_verified = true;
    }

  }
  if (read_labels)
  {
    auto expected=fragment_plan_reference(fragment_census_reference(out_labels,shape.dims));
    bool const overflow=expected.components.size()>FRAGMENT_PLAN_CAPACITY;
    bool const match=overflow ? manifest->status==1u :
        manifest->status==0u && expected.components.size()==manifest->count &&
        (manifest->count==0u || std::memcmp(expected.components.data(),manifest->components,manifest->count*sizeof(FragmentComponent))==0);
    if (!match) { std::cerr << "[PLAN-VERIFY] FAILED" << std::endl;std::abort(); }
    for (auto &label:out_labels) if (label!=MAX_U32) label=expected.remap.at(label);
    std::cout << "[PLAN-VERIFY] source=" << manifest->source_count << " children=" << manifest->count << " MATCH" << std::endl;
  }
  if (!staging.is_empty()) device.destroy_buffer(staging);
}

void RigidBodyManager::upload_voxel_occupancy(std::span<daxa_u32 const> occupancy)
{
  if (occupancy.empty()) return;
  // Scene creation only. Runtime fracture never uploads an occupancy mirror.
  auto staging=device.create_buffer({.size=occupancy.size_bytes(),
      .memory_flags=daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE, .name="initial_voxel_occupancy"});
  std::memcpy(device.buffer_host_address_as<daxa_u32>(staging).value(),occupancy.data(),occupancy.size_bytes());
  auto rec=device.create_command_recorder({});
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::READ_WRITE,
                       .dst_access=daxa::AccessConsts::TRANSFER_WRITE});
  rec.copy_buffer_to_buffer({.src_buffer=staging,.dst_buffer=voxel_occupancy,.size=occupancy.size_bytes()});
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::TRANSFER_WRITE,
                       .dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ});
  auto commands=rec.complete_current_commands();
  device.submit_commands({.command_lists=std::array{commands}});
  device.wait_on_submit({.queue=daxa::QUEUE_MAIN,
      .queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
  device.destroy_buffer(staging);
}

std::vector<daxa_u32> RigidBodyManager::read_voxel_occupancy(daxa_u32 count)
{
  if (count==0) return {};
  // Explicit diagnostic download; never part of production fracture.
  auto staging=device.create_buffer({.size=sizeof(daxa_u32)*count,
      .memory_flags=daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,.name="occupancy_oracle_readback"});
  auto rec=device.create_command_recorder({});
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::WRITE,
                       .dst_access=daxa::AccessConsts::TRANSFER_READ});
  rec.copy_buffer_to_buffer({.src_buffer=voxel_occupancy,.dst_buffer=staging,.size=sizeof(daxa_u32)*count});
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::TRANSFER_WRITE,
                       .dst_access=daxa::AccessConsts::HOST_READ});
  auto commands=rec.complete_current_commands();
  device.submit_commands({.command_lists=std::array{commands}});
  device.wait_on_submit({.queue=daxa::QUEUE_MAIN,
      .queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
  auto const *host=device.buffer_host_address_as<daxa_u32>(staging).value();
  std::vector<daxa_u32> result(host,host+count);
  device.destroy_buffer(staging);
  return result;
}

void RigidBodyManager::initialize_fracture_allocator(std::array<daxa_u32,GPU_POOL_COUNT> const &high_water,
                                                     std::span<RigidBody const> spawn_templates)
{
  fracture_spawn_template_count=static_cast<daxa_u32>(spawn_templates.size());
  if (fracture_spawn_template_count>BB_MAX_VOXEL_SHAPE_COUNT) std::abort();
  auto staging=device.create_buffer({.size=sizeof(FractureAllocationState)+spawn_templates.size_bytes(),
      .memory_flags=daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,.name="fracture_allocator_initialization"});
  auto *state=device.buffer_host_address_as<FractureAllocationState>(staging).value();
  *state={};state->spawn_seed=0x50a4b0c5u;
  if (!spawn_templates.empty()) std::memcpy(reinterpret_cast<std::byte*>(state)+sizeof(*state),spawn_templates.data(),spawn_templates.size_bytes());
  std::array<daxa_u32,GPU_POOL_COUNT> capacities{BB_MAX_VOXEL_OCC_U32S,BB_MAX_VOXEL_SDF_F32S,
      BB_MAX_VOXEL_SURF_COUNT,0u,BB_MAX_VOXEL_SHAPE_COUNT,BB_MAX_RIGID_BODY_COUNT};
  for (size_t i=0;i<GPU_POOL_COUNT;++i) {
    state->pools[i].capacity=capacities[i];state->pools[i].high_water=high_water[i];state->pools[i].live_units=high_water[i];
  }
  auto rec=device.create_command_recorder({});
  rec.copy_buffer_to_buffer({.src_buffer=staging,.dst_buffer=fracture_allocator,.size=sizeof(FractureAllocationState)});
  if (!spawn_templates.empty()) rec.copy_buffer_to_buffer({.src_buffer=staging,.dst_buffer=fracture_spawn_templates,
      .src_offset=sizeof(FractureAllocationState),.size=spawn_templates.size_bytes()});
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::TRANSFER_WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
  auto list=rec.complete_current_commands();device.submit_commands({.command_lists=std::array{list}});
  device.wait_on_submit({.queue=daxa::QUEUE_MAIN,.queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
  device.destroy_buffer(staging);
}

std::vector<GpuFreeList> RigidBodyManager::read_fracture_pools()
{
  auto staging=device.create_buffer({.size=sizeof(FractureAllocationState),
      .memory_flags=daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,.name="fracture_allocator_oracle"});
  auto rec=device.create_command_recorder({});
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::TRANSFER_READ});
  rec.copy_buffer_to_buffer({.src_buffer=fracture_allocator,.dst_buffer=staging,.size=sizeof(FractureAllocationState)});
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::TRANSFER_WRITE,.dst_access=daxa::AccessConsts::HOST_READ});
  auto list=rec.complete_current_commands();device.submit_commands({.command_lists=std::array{list}});
  device.wait_on_submit({.queue=daxa::QUEUE_MAIN,.queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
  auto *state=device.buffer_host_address_as<FractureAllocationState>(staging).value();
  if (state->active || state->failures) { std::cerr << "[GPU-ALLOCATOR-VERIFY] FAILED" << std::endl;std::abort(); }
  std::vector<GpuFreeList> result(state->pools,state->pools+GPU_POOL_COUNT);
  device.destroy_buffer(staging);return result;
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
  rec.pipeline_barrier({.src_access = daxa::AccessConsts::COMPUTE_SHADER_WRITE,
                        .dst_access = daxa::AccessConsts::TRANSFER_READ});
  rec.copy_buffer_to_buffer({.src_buffer = voxel_derived, .dst_buffer = staging, .size = size});
  rec.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE,
                        .dst_access = daxa::AccessConsts::READ});
  rec.pipeline_barrier({.src_access = daxa::AccessConsts::WRITE,
                        .dst_access = daxa::AccessConsts::HOST_READ});
  auto cmds = rec.complete_current_commands();
  device.submit_commands({.command_lists = std::array{cmds}});
  device.wait_on_submit({.queue = daxa::QUEUE_MAIN,
      .queue_submit_index = device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
  VoxelShapeDerived const *host = device.buffer_host_address_as<VoxelShapeDerived>(staging).value();
  out.assign(host, host + count);
  device.destroy_buffer(staging);
}

bool RigidBodyManager::read_back_sim_config(bool completed_simulation_snapshot)
{
  if (!initialized)
  {
    return !initialized;
  }

  // Every solver publishes SimConfig at its graph tail. Callers that already
  // completed that simulation can consume the current parity without another
  // submission or CPU wait. Keep the standalone path for non-stepped updates.
  if (!completed_simulation_snapshot)
  {
    update_buffers();
    readback_SC_TG.execute();
    device.wait_on_submit({
        .queue = daxa::QUEUE_COMPUTE_0,
        .queue_submit_index = device.latest_queue_submit_index(daxa::QUEUE_COMPUTE_0),
    });
  }

  if (narrow_phase_timing && narrow_phase_query_pending)
  {
    // Read after the existing COMPUTE_0 completion wait; no additional stall.
    auto const results = narrow_phase_queries.get_query_results(0, 2);
    if (results[1] != 0u && results[3] != 0u)
    {
      double const ms = double(results[2] - results[0]) * device.properties().limits.timestamp_period / 1.0e6;
      std::cout << "[FRACTURE-NP] gpu_ms=" << ms << std::endl;
    }
    narrow_phase_query_pending = false;
  }

  if (narrow_phase_timing && solver_stage_query_pending)
  {
    auto const results = solver_stage_queries.get_query_results(0, 6);
    bool available = true;
    for (daxa_u32 i = 0u; i < 6u; ++i) available &= results[2u*i+1u] != 0u;
    if (available)
    {
      static constexpr char const *avbd_names[] = {"setup_ms", "prepare_ms", "main_ms", "post_ms", "finalize_ms"};
      static constexpr char const *tgs_names[] = {"setup_ms", "prepare_ms", "contacts_ms", "substeps_ms", "finalize_ms"};
      bool const tgs = stage_query_solver == SimSolverType::TGS_SOFT;
      auto const *names = tgs ? tgs_names : avbd_names;
      std::cout << (tgs ? "[TGS-STAGES]" : "[AVBD-STAGES]");
      for (daxa_u32 i = 0u; i < 5u; ++i)
        std::cout << ' ' << names[i] << '=' << double(results[2u*(i+1u)] - results[2u*i]) * device.properties().limits.timestamp_period / 1.0e6;
      std::cout << std::endl;
    }
    solver_stage_query_pending = false;
  }

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

  // Daxa 3.6 set_buffer clears queue history. When ping-pong roles reverse,
  // swap IDs AND their history, then avoid resetting unchanged bindings below.
  auto swap_roles = [](daxa::TaskBuffer &a, daxa::TaskBuffer &b,
                       daxa::BufferId next_a, daxa::BufferId next_b) {
    if (next_a != next_b && a.id() == next_b && b.id() == next_a) { a.swap_buffers(b); }
  };
  swap_roles(task_sim_config, task_old_sim_config, sim_config[current_frame], sim_config[previous_frame]);
  swap_roles(task_lbvh_nodes, task_previous_lbvh_nodes, lbvh_nodes[current_frame], lbvh_nodes[previous_frame]);
  swap_roles(task_rigid_body_entries, task_previous_rigid_body_entries, rigid_body_entries[current_frame], rigid_body_entries[previous_frame]);
  swap_roles(task_rigid_body_link_manifolds, task_previous_rigid_body_link_manifolds, rigid_body_link_manifolds[current_frame], rigid_body_link_manifolds[previous_frame]);
  swap_roles(task_collision_entries, task_collision_entries_previous, collision_entries[current_frame], collision_entries[previous_frame]);
  swap_roles(task_collisions, task_old_collisions, collisions[current_frame], collisions[previous_frame]);
  swap_roles(task_islands, task_previous_islands, island_buffer[current_frame], island_buffer[previous_frame]);
  swap_roles(task_contact_islands, task_previous_contact_islands, contact_island_buffer[current_frame], contact_island_buffer[previous_frame]);


  if (task_sim_config_host.id() != sim_config_host_buffer[current_frame]) { task_sim_config_host.set_buffer(sim_config_host_buffer[current_frame]); }
  if (task_sim_config.id() != sim_config[current_frame]) { task_sim_config.set_buffer(sim_config[current_frame]); }
  if (task_old_sim_config.id() != sim_config[previous_frame]) { task_old_sim_config.set_buffer(sim_config[previous_frame]); }
  if (task_morton_codes.id() != morton_codes) { task_morton_codes.set_buffer(morton_codes); }
  if (task_tmp_morton_codes.id() != tmp_morton_codes) { task_tmp_morton_codes.set_buffer(tmp_morton_codes); }
  if (task_radix_sort_histograms.id() != global_histograms[current_frame]) { task_radix_sort_histograms.set_buffer(global_histograms[current_frame]); }
  // NOTE: unlike every other binding here, these three resolve parity via the accel-struct
  // manager's GLOBAL get_sim_frame_index() and IGNORE the `current_frame` argument. Harmless today
  // (the graphs that call update_buffers(f) in a parity loop — update_SC_TG, ARB_TG — don't attach
  // these buffers), but a latent trap: extending either graph to touch the rigid-body buffers would
  // silently bind the global-current parity for the f=0 iteration. Add frame-indexed getter
  // overloads if that ever changes.
  if (task_previous_rigid_bodies.id() != accel_struct_mngr->get_previous_rigid_body_buffer()) { task_previous_rigid_bodies.set_buffer(accel_struct_mngr->get_previous_rigid_body_buffer()); }
  if (task_rigid_bodies.id() != accel_struct_mngr->get_rigid_body_buffer()) { task_rigid_bodies.set_buffer(accel_struct_mngr->get_rigid_body_buffer()); }
  if (task_next_rigid_bodies.id() != accel_struct_mngr->get_next_rigid_body_buffer()) { task_next_rigid_bodies.set_buffer(accel_struct_mngr->get_next_rigid_body_buffer()); }
  if (task_lbvh_nodes.id() != lbvh_nodes[current_frame]) { task_lbvh_nodes.set_buffer(lbvh_nodes[current_frame]); }
  if (task_previous_lbvh_nodes.id() != lbvh_nodes[previous_frame]) { task_previous_lbvh_nodes.set_buffer(lbvh_nodes[previous_frame]); }
  if (task_lbvh_construction_info.id() != lbvh_construction_info) { task_lbvh_construction_info.set_buffer(lbvh_construction_info); }
  if (task_active_rigid_bodies.id() != active_rigid_bodies[current_frame]) { task_active_rigid_bodies.set_buffer(active_rigid_bodies[current_frame]); }
  if (task_rigid_body_entries.id() != rigid_body_entries[current_frame]) { task_rigid_body_entries.set_buffer(rigid_body_entries[current_frame]); }
  if (task_previous_rigid_body_entries.id() != rigid_body_entries[previous_frame]) { task_previous_rigid_body_entries.set_buffer(rigid_body_entries[previous_frame]); }
  if (task_broad_phase_collisions.id() != broad_phase_collisions[current_frame]) { task_broad_phase_collisions.set_buffer(broad_phase_collisions[current_frame]); }
  if (task_rigid_body_scratch.id() != rigid_body_scratch) { task_rigid_body_scratch.set_buffer(rigid_body_scratch); }
  if (task_rigid_body_link_manifolds.id() != rigid_body_link_manifolds[current_frame]) { task_rigid_body_link_manifolds.set_buffer(rigid_body_link_manifolds[current_frame]); }
  if (task_collision_entries.id() != collision_entries[current_frame]) { task_collision_entries.set_buffer(collision_entries[current_frame]); }
  if (task_collisions.id() != collisions[current_frame]) { task_collisions.set_buffer(collisions[current_frame]); }
  if (task_collision_scratch.id() != collision_scratch) { task_collision_scratch.set_buffer(collision_scratch); }
  if (task_previous_rigid_body_link_manifolds.id() != rigid_body_link_manifolds[previous_frame]) { task_previous_rigid_body_link_manifolds.set_buffer(rigid_body_link_manifolds[previous_frame]); }
  if (task_collision_entries_previous.id() != collision_entries[previous_frame]) { task_collision_entries_previous.set_buffer(collision_entries[previous_frame]); }
  if (task_old_collisions.id() != collisions[previous_frame]) { task_old_collisions.set_buffer(collisions[previous_frame]); }
  if (task_scratch_body_links.id() != scratch_body_links[current_frame]) { task_scratch_body_links.set_buffer(scratch_body_links[current_frame]); }
  if (task_body_links.id() != body_links[current_frame]) { task_body_links.set_buffer(body_links[current_frame]); }
  if (task_manifold_links.id() != manifold_links[current_frame]) { task_manifold_links.set_buffer(manifold_links[current_frame]); }
  if (task_islands.id() != island_buffer[current_frame]) { task_islands.set_buffer(island_buffer[current_frame]); }
  if (task_previous_islands.id() != island_buffer[previous_frame]) { task_previous_islands.set_buffer(island_buffer[previous_frame]); }
  if (task_contact_islands.id() != contact_island_buffer[current_frame]) { task_contact_islands.set_buffer(contact_island_buffer[current_frame]); }
  if (task_previous_contact_islands.id() != contact_island_buffer[previous_frame]) { task_previous_contact_islands.set_buffer(contact_island_buffer[previous_frame]); }
}

BB_NAMESPACE_END

std::vector<FractureParentContext> beatbox::RigidBodyManager::read_fracture_contexts()
{
  auto size = sizeof(FractureParentContext)*BB_MAX_RIGID_BODY_COUNT;
  auto staging = device.create_buffer({.size=size,
      .memory_flags=daxa::MemoryFlagBits::HOST_ACCESS_RANDOM, .name="fracture_context_oracle"});
  auto rec = device.create_command_recorder({});
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,
                        .dst_access=daxa::AccessConsts::TRANSFER_READ});
  rec.copy_buffer_to_buffer({.src_buffer=fracture_contexts,.dst_buffer=staging,.size=size});
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::TRANSFER_WRITE,.dst_access=daxa::AccessConsts::HOST_READ});
  auto commands=rec.complete_current_commands();
  device.submit_commands({.command_lists=std::array{commands}});
  device.wait_on_submit({.queue=daxa::QUEUE_MAIN,.queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
  auto *data=device.buffer_host_address_as<FractureParentContext>(staging).value();
  std::vector<FractureParentContext> result(data,data+BB_MAX_RIGID_BODY_COUNT);
  device.destroy_buffer(staging);
  return result;
}

void beatbox::RigidBodyManager::verify_fracture_events_gpu()
{
  auto make = [&](daxa::usize size, char const *name) {
    return device.create_buffer({.size=size,.memory_flags=daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,.name=name});
  };
  auto scratch_buffer=make(sizeof(FractureImpactScratch),"impact_test_scratch");
  auto events_buffer=make(sizeof(FractureEventBuffer),"impact_test_events");
  auto config_buffer=make(sizeof(SimConfig),"impact_test_config");
  auto *scratch=device.buffer_host_address_as<FractureImpactScratch>(scratch_buffer).value();
  auto *events=device.buffer_host_address_as<FractureEventBuffer>(events_buffer).value();
  auto *config=device.buffer_host_address_as<SimConfig>(config_buffer).value();
  *scratch={}; *events={}; *config={};
  events->scratch_addr=device.device_address(scratch_buffer).value();
  config->fracture_events_addr=device.device_address(events_buffer).value();
  auto submit = [&](bool reset) {
    auto rec=device.create_command_recorder({});
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
    FractureImpactPushConstants pc{.config_addr=device.device_address(config_buffer).value()};
    if (reset) {
      rec.set_pipeline(*pipeline_impact_reset);rec.push_constant(pc);
      rec.dispatch({.x=BB_MAX_COLLISION_COUNT/64u});
      rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
    }
    rec.set_pipeline(*pipeline_impact_publish);rec.push_constant(pc);rec.dispatch({.x=1u});
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::HOST_READ});
    auto list=rec.complete_current_commands();device.submit_commands({.command_lists=std::array{list}});
    device.wait_on_submit({.queue=daxa::QUEUE_MAIN,.queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
  };
  auto check = [](bool valid) {
    if (!valid) { std::cerr << "[IMPACT-VERIFY] FAILED" << std::endl;std::abort(); }
  };
  for (daxa_u32 id=0;id<BB_MAX_RIGID_BODY_COUNT;++id)
    scratch->selected[id]={.body_id=id,.impulse=float(id+1u),.position={float(id),0,0}};
  submit(false);
  check(events->count==BB_MAX_RIGID_BODY_COUNT && events->serial==1u);
  for (daxa_u32 id=0;id<BB_MAX_RIGID_BODY_COUNT;++id)
    check(events->events[id].body_id==id && events->events[id].impulse==float(id+1u));
  // An empty catch-up step must not erase unconsumed events.
  submit(true);check(events->count==BB_MAX_RIGID_BODY_COUNT && events->serial==1u);
  scratch->selected[50]={.body_id=50,.impulse=4096,.position={1,0,0}};
  scratch->selected[51]={.body_id=51,.impulse=1};
  submit(false);check(events->count==BB_MAX_RIGID_BODY_COUNT && events->serial==2u);
  check(events->events[50].impulse==4096 && events->events[51].impulse==52);
  // Equal impulses use the contact coordinates as a stable tie breaker.
  scratch->selected[50].position={0,0,0};submit(false);
  check(events->serial==3u && scratch->pending[50].position.x==0);
  scratch->selected[50].normal={-1,0,0};submit(false);
  check(events->serial==4u && scratch->pending[50].normal.x==-1);
  events->consumed_serial=events->serial;
  submit(true);check(events->count==0u && events->serial==4u);
  scratch->selected[7]={.body_id=7,.impulse=1};submit(false);
  check(events->count==1u && events->serial==5u && events->events[0].body_id==7u);
  device.destroy_buffer(config_buffer);device.destroy_buffer(events_buffer);device.destroy_buffer(scratch_buffer);
  std::cout << "[IMPACT-VERIFY] full capacity, catch-up, maximum, ties and acknowledgement MATCH" << std::endl;
}

void beatbox::RigidBodyManager::verify_fracture_allocator_gpu()
{
  std::vector<daxa::BufferId> buffers;
  auto make=[&](daxa::usize size) {
    auto buffer=device.create_buffer({.size=size,.memory_flags=daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,.name="allocation_oracle"});
    buffers.push_back(buffer);return buffer;
  };
  auto batch_buffer=make(sizeof(FractureBatchManifest));
  auto *batch=device.buffer_host_address_as<FractureBatchManifest>(batch_buffer).value();
  *batch={};
  auto state_buffer=make(sizeof(FractureAllocationState));
  auto plan_buffer=make(sizeof(FragmentPlanManifest));
  auto output_buffer=make(sizeof(FractureAllocationManifest));
  auto context_buffer=make(sizeof(FractureParentContext));
  auto shapes_buffer=make(sizeof(VoxelShape)*4u);
  auto labels_buffer=make(sizeof(daxa_u32)*8u);
  auto remap_buffer=make(sizeof(daxa_u32)*8u);
  auto occupancy_buffer=make(sizeof(daxa_u32)*16u);
  auto *state=device.buffer_host_address_as<FractureAllocationState>(state_buffer).value();
  auto *plan=device.buffer_host_address_as<FragmentPlanManifest>(plan_buffer).value();
  auto *output=device.buffer_host_address_as<FractureAllocationManifest>(output_buffer).value();
  auto *context=device.buffer_host_address_as<FractureParentContext>(context_buffer).value();
  auto *shapes=device.buffer_host_address_as<VoxelShape>(shapes_buffer).value();
  auto *labels=device.buffer_host_address_as<daxa_u32>(labels_buffer).value();
  auto *remap=device.buffer_host_address_as<daxa_u32>(remap_buffer).value();
  auto *occupancy=device.buffer_host_address_as<daxa_u32>(occupancy_buffer).value();
  *state={};*plan={};*output={};
  *context={.parent={.primitive_count=8u,.shape_index=1u,.rotation=Quaternion(0,0,0,1)},
            .shape={.dims={2,2,2}}};
  std::fill_n(shapes,4u,VoxelShape{});shapes[0]=context->shape;
  std::array<daxa_u32,6> capacity{16,128,32,0,4,2}, initial{1,27,8,0,1,1};
  for (size_t i=0;i<6;++i) { state->pools[i].capacity=capacity[i];state->pools[i].allocate(initial[i]); }
  state->private_shapes[0]=1u;
  plan->count=2u;plan->solid_count=8u;
  for (daxa_u32 i=0;i<2;++i) {
    plan->components[i]={.label=i,.count=4u,.lo_x=i,.hi_x=i,.hi_y=1u,.hi_z=1u};
  }
  for (daxa_u32 i=0;i<8;++i) { labels[i]=i%2u;remap[i]=i; }
  std::fill_n(occupancy,16u,0xdeadbeefu);occupancy[0]=255u;
  auto address=[&](daxa::BufferId buffer) { return device.device_address(buffer).value(); };
  FractureAllocatorPushConstants pc{address(state_buffer),address(plan_buffer),address(output_buffer),
      address(context_buffer),address(shapes_buffer),address(batch_buffer)};
  bool reset_batch=true;
  auto run=[&](bool pack) {
    auto rec=device.create_command_recorder({});
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
    rec.set_pipeline(*pipeline_fracture_allocate);
    if (reset_batch) {
      auto reset=pc;reset.operation=2u;rec.push_constant(reset);rec.dispatch({.x=1u});
      rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
      reset_batch=false;
    }
    rec.push_constant(pc);rec.dispatch({.x=1u});
    if (pack) {
      rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::READ});
      rec.set_pipeline(*pipeline_fragment_batch_pack);
      rec.push_constant(FractureBatchPackingPushConstants{address(plan_buffer),address(output_buffer),
          address(context_buffer),address(labels_buffer),address(remap_buffer),address(occupancy_buffer)});
      rec.dispatch_indirect({.indirect_buffer=output_buffer,.offset=offsetof(FractureAllocationManifest,dispatch_x)});
      rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
      auto finish=pc;finish.operation=1u;
      rec.set_pipeline(*pipeline_fracture_allocate);rec.push_constant(finish);rec.dispatch({.x=1u});
    }
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::HOST_READ});
    auto commands=rec.complete_current_commands();device.submit_commands({.command_lists=std::array{commands}});
    device.wait_on_submit({.queue=daxa::QUEUE_MAIN,.queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
  };
  auto check=[](bool ok) { if (!ok) { std::cerr << "[ALLOCATION-VERIFY] FAILED" << std::endl;std::abort(); } };
  run(true);
  check(output->status==0u && output->count==2u && state->active==0u && state->failures==0u);
  check(batch->status==0u && batch->count==2u && batch->refused==0u);
  for (daxa_u32 i=0;i<2;++i) {
    check(batch->children[i].parent_id==0u && batch->children[i].component.count==4u);
    check(std::memcmp(&batch->children[i].allocation,&output->children[i],sizeof(FractureChildAllocation))==0);
  }
  check(occupancy[0]==255u && occupancy[1]==15u && occupancy[2]==15u && occupancy[3]==0xdeadbeefu);
  check(output->children[0].offsets[5]==0u && output->children[1].offsets[5]==1u);
  check(state->private_shapes[0]==0u && state->private_shapes[1]==1u && state->private_shapes[2]==1u);
  for (auto &pool:state->pools) check(pool.valid());
  check(state->pools[0].live_units==2u && state->pools[5].live_units==2u);
  // Force failure in the LAST pool after earlier tentative reservations succeeded.
  // Neither allocation ownership nor any visible shape may change on rejection.
  std::vector<GpuFreeList> before(state->pools,state->pools+6);
  std::vector<VoxelShape> shapes_before(shapes,shapes+4);
  context->shape=shapes[1];context->parent.shape_index=2u;context->parent.primitive_count=4u;
  plan->solid_count=4u;
  for (daxa_u32 i=0;i<2;++i) plan->components[i]={.label=i,.count=2u,.lo_y=i,.hi_y=i,.hi_z=1u};
  run(true);
  check(output->status==2u && output->count==0u && state->active==0u && state->failures==0u);
  check(batch->status==0u && batch->count==2u && batch->refused==1u);
  check(std::memcmp(before.data(),state->pools,sizeof(state->pools))==0);
  check(std::memcmp(shapes_before.data(),shapes,sizeof(VoxelShape)*4u)==0);
  check(occupancy[0]==255u && occupancy[1]==15u && occupancy[2]==15u);
  // Exercise plan boundaries independently of the current scene's topology.
  constexpr daxa_u32 test_cells=2048u;
  auto census_buffer=make(sizeof(FragmentComponent)*test_cells);
  auto plan_remap_buffer=make(sizeof(daxa_u32)*test_cells);
  auto *census=device.buffer_host_address_as<FragmentComponent>(census_buffer).value();
  auto *plan_remap=device.buffer_host_address_as<daxa_u32>(plan_remap_buffer).value();
  for (daxa_u32 roots : {0u,16u,131u,FRAGMENT_PLAN_CAPACITY+1u}) {
    std::fill_n(census,test_cells,FragmentComponent{});
    for (daxa_u32 i=0;i<roots;++i) {
      daxa_u32 count=roots==16u ? 1u+i%2u : 3u;
      census[i]={.label=i,.count=count,.sum_x=i*count,.lo_x=i,.hi_x=i};
    }
    auto rec=device.create_command_recorder({});
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
    rec.set_pipeline(*pipeline_fragment_plan);
    rec.push_constant(FragmentPlanPushConstants{address(census_buffer),address(plan_remap_buffer),address(plan_buffer),test_cells});
    rec.dispatch({.x=1u});
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::HOST_READ});
    auto commands=rec.complete_current_commands();device.submit_commands({.command_lists=std::array{commands}});
    device.wait_on_submit({.queue=daxa::QUEUE_MAIN,.queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
    if (roots>FRAGMENT_PLAN_CAPACITY) { check(plan->status==1u);continue; }
    auto reference=fragment_plan_reference(std::span<FragmentComponent const>(census,roots));
    check(plan->status==0u && plan->count==reference.components.size());
    if (plan->count) check(std::memcmp(plan->components,reference.components.data(),plan->count*sizeof(FragmentComponent))==0);
    for (auto const &[label,target]:reference.remap) check(plan_remap[label]==target);
  }
  std::cout << "[PLAN-VERIFY] empty, all-slivers, 131 roots and capacity rejection MATCH" << std::endl;
  // Independent breadth-first reference for the single-dispatch small-grid
  // connectivity path, including empty cells and disconnected same-site regions.
  auto small_context_buffer=make(sizeof(FractureParentContext));
  auto small_labels_buffer=make(sizeof(daxa_u32)*64u);
  auto small_sites_buffer=make(sizeof(daxa_u32)*64u);
  auto *small_context=device.buffer_host_address_as<FractureParentContext>(small_context_buffer).value();
  auto *small_labels=device.buffer_host_address_as<daxa_u32>(small_labels_buffer).value();
  auto *small_sites=device.buffer_host_address_as<daxa_u32>(small_sites_buffer).value();
  daxa_u32 random=0x843175bu;
  for (daxa_u32 fixture=0u;fixture<24u;++fixture) {
    daxa_u32vec3 dims=fixture%3u==0u ? daxa_u32vec3{4u,4u,4u} :
                     fixture%3u==1u ? daxa_u32vec3{1u,1u,64u} : daxa_u32vec3{3u,3u,7u};
    daxa_u32 cells=dims.x*dims.y*dims.z;
    small_context->shape.dims=dims;
    std::array<daxa_u32,64> expected;expected.fill(MAX_U32);
    std::array<bool,64> solid{};
    for (daxa_u32 i=0u;i<cells;++i) {
      random=random*1664525u+1013904223u;
      solid[i]=fixture<3u || (fixture>=6u && (random>>29u)!=0u);
      small_labels[i]=solid[i] ? i : MAX_U32;
      small_sites[i]=fixture<9u ? 0u : (random>>16u)%3u;
    }
    for (daxa_u32 root=0u;root<cells;++root) {
      if (!solid[root] || expected[root]!=MAX_U32) continue;
      expected[root]=root;std::vector<daxa_u32> queue{root};
      for (size_t q=0u;q<queue.size();++q) {
        auto i=queue[q];auto x=i%dims.x,y=(i/dims.x)%dims.y,z=i/(dims.x*dims.y);
        std::array<daxa_u32,6> neighbors{x>0u ? i-1u : MAX_U32,x+1u<dims.x ? i+1u : MAX_U32,
          y>0u ? i-dims.x : MAX_U32,y+1u<dims.y ? i+dims.x : MAX_U32,
          z>0u ? i-dims.x*dims.y : MAX_U32,z+1u<dims.z ? i+dims.x*dims.y : MAX_U32};
        for (auto n:neighbors) if (n!=MAX_U32 && solid[n] && expected[n]==MAX_U32 && small_sites[n]==small_sites[i]) {
          expected[n]=root;queue.push_back(n);
        }
      }
    }
    auto rec=device.create_command_recorder({});
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
    rec.set_pipeline(*pipeline_VFR_FLOOD_STEP);
    rec.push_constant(VoxelFracturePushConstants{.context_addr=address(small_context_buffer),
      .labels_addr=address(small_labels_buffer),.site_labels_addr=address(small_sites_buffer),.use_voronoi=1u});
    rec.dispatch({.x=1u});
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::HOST_READ});
    auto commands=rec.complete_current_commands();device.submit_commands({.command_lists=std::array{commands}});
    device.wait_on_submit({.queue=daxa::QUEUE_MAIN,.queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
    for (daxa_u32 i=0u;i<cells;++i) check(small_labels[i]==expected[i]);
  }
  std::cout << "[SMALL-FLOOD-VERIFY] 24 grids, empty cells, chains and site boundaries MATCH" << std::endl;
  // A full body pool must reject a standalone spawn, but a combined edit
  // can retire a body and reuse that exact slot before any AS publication.
  auto edit_bodies_buffer=make(sizeof(RigidBody)*2u);
  auto edit_template_buffer=make(sizeof(RigidBody));
  auto edit_spawn_buffer=make(sizeof(RigidBody));
  auto edit_output_buffer=make(sizeof(FractureSceneEditManifest));
  auto *edit_bodies=device.buffer_host_address_as<RigidBody>(edit_bodies_buffer).value();
  auto *edit_template=device.buffer_host_address_as<RigidBody>(edit_template_buffer).value();
  auto *edit_spawn=device.buffer_host_address_as<RigidBody>(edit_spawn_buffer).value();
  auto *edit_output=device.buffer_host_address_as<FractureSceneEditManifest>(edit_output_buffer).value();
  *state={};state->spawn_seed=123u;
  std::array<daxa_u32,6> edit_initial{2,16,2,0,2,2};
  for (size_t i=0;i<6;++i) { state->pools[i].capacity=capacity[i];state->pools[i].allocate(edit_initial[i]); }
  state->private_shapes[0]=1u;
  shapes[0]={.dims={1,1,1}};
  shapes[1]={.dims={1,1,1},.occ_offset=1u,.surf_offset=1u,.sdf_offset=8u};
  edit_bodies[0]={.id=0u,.position={0,-10,0},.rotation=Quaternion(0,0,0,1)}; // a static below the kill plane must survive
  edit_bodies[1]={.id=1u,.flags=RigidBodyFlag::DYNAMIC,.primitive_count=1u,.shape_index=1u,
                  .position={0,-1,0},.rotation=Quaternion(0,0,0,1)};
  *edit_template={.flags=RigidBodyFlag::DYNAMIC | RigidBodyFlag::GRAVITY,
                  .primitive_count=1u,.shape_index=2u,.rotation=Quaternion(0,0,0,1)};
  auto run_edit=[&](daxa_u32 operation) {
    auto rec=device.create_command_recorder({});
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
    rec.set_pipeline(*pipeline_fracture_scene_edit);
    rec.push_constant(FractureSceneEditPushConstants{address(state_buffer),address(edit_bodies_buffer),
        address(shapes_buffer),address(edit_template_buffer),address(edit_spawn_buffer),address(edit_output_buffer),2u,1u,operation,0.0f});
    rec.dispatch({.x=1u});
    rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::HOST_READ});
    auto commands=rec.complete_current_commands();device.submit_commands({.command_lists=std::array{commands}});
    device.wait_on_submit({.queue=daxa::QUEUE_MAIN,.queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
  };
  run_edit(FRACTURE_EDIT_SPAWN);
  check(edit_output->status==2u && edit_output->retired_count==0u && edit_output->spawn_id==MAX_U32);
  check(state->spawn_seed==123u && state->pools[5].live_units==2u && state->private_shapes[0]==1u);
  run_edit(FRACTURE_EDIT_RETIRE | FRACTURE_EDIT_SPAWN);
  check(edit_output->status==0u && edit_output->retired_count==1u && edit_output->ids[0]==1u);
  check(edit_output->spawn_id==1u && edit_output->spawn_template==0u);
  check(edit_spawn->id==1u && edit_spawn->shape_index==2u && edit_spawn->position.y>=10.6f && edit_spawn->position.y<=11.4f);
  check(state->pools[5].live_units==2u && state->pools[4].live_units==1u && state->private_shapes[0]==0u);
  check(state->body_edits[1].kind==3u && state->body_edits[0].kind==0u);
  check(edit_bodies[0].position.y==-10.0f && edit_bodies[1].shape_index==1u);
  for (auto &pool:state->pools) check(pool.valid());
  run_edit(0u);
  check(edit_output->status==0u && edit_output->retired_count==0u && edit_output->spawn_id==MAX_U32);
  std::cout << "[SCENE-EDIT-VERIFY] full-pool refusal, retire-and-reuse, static preservation and no-op MATCH" << std::endl;
  for (auto buffer:buffers) device.destroy_buffer(buffer);
  std::cout << "[ALLOCATION-VERIFY] reservation, indirect packing, retirement and last-pool rollback MATCH" << std::endl;
}

FractureSceneEditManifest beatbox::RigidBodyManager::edit_fracture_scene_gpu(bool cull, bool spawn, daxa_f32 kill_y)
{
  // Submit after previous MAIN readers instead of waiting before recording.
  // The result wait below still completes them before host AS retirement.
  auto rec=device.create_command_recorder({});
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::READ_WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
  rec.set_pipeline(*pipeline_fracture_scene_edit);
  rec.push_constant(FractureSceneEditPushConstants{
      .allocator_addr=device.device_address(fracture_allocator).value(),
      .bodies_addr=device.device_address(accel_struct_mngr->get_rigid_body_buffer()).value(),
      .shapes_addr=device.device_address(voxel_shapes).value(),
      .templates_addr=device.device_address(fracture_spawn_templates).value(),
      .spawn_addr=device.device_address(fracture_spawn_body).value(),
      .output_addr=device.device_address(fracture_scene_manifest).value(),
      .body_count=renderer_manager->get_rigid_body_count(),.template_count=fracture_spawn_template_count,
      .operation=(cull ? FRACTURE_EDIT_RETIRE : 0u) | (spawn ? FRACTURE_EDIT_SPAWN : 0u),.kill_y=kill_y});
  rec.dispatch({.x=1u});
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::HOST_READ});
  auto list=rec.complete_current_commands();device.submit_commands({.command_lists=std::array{list}});
  device.wait_on_submit({.queue=daxa::QUEUE_MAIN,.queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
  auto result=*device.buffer_host_address_as<FractureSceneEditManifest>(fracture_scene_manifest).value();
  if (result.status==3u || result.retired_count>BB_MAX_RIGID_BODY_COUNT) { std::cerr << "GPU scene edit FAILED" << std::endl;std::abort(); }
  return result;
}

std::vector<FractureBatchChild> beatbox::RigidBodyManager::fracture_batch_gpu(std::span<FracturePartitionInput const> inputs)
{
  if (inputs.empty()) return {};
  auto rec=device.create_command_recorder({});
  auto batch_addr=device.device_address(fracture_batch_manifest).value();
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::READ_WRITE,.dst_access=daxa::AccessConsts::COMPUTE_SHADER_READ_WRITE});
  rec.set_pipeline(*pipeline_fracture_allocate);
  rec.push_constant(FractureAllocatorPushConstants{.state_addr=device.device_address(fracture_allocator).value(),
      .batch_addr=batch_addr,.operation=2u});rec.dispatch({.x=1u});
  auto body_count=renderer_manager->get_rigid_body_count();
  for (auto const &input:inputs)
    record_fracture_partition(rec,input.body_id,body_count,batch_addr,input.recorded_passes,false);
  rec.pipeline_barrier({.src_access=daxa::AccessConsts::COMPUTE_SHADER_WRITE,.dst_access=daxa::AccessConsts::HOST_READ});
  auto commands=rec.complete_current_commands();
  device.submit_commands({.command_lists=std::array{commands},
      .wait_queue_submit_indices=std::array{std::pair{daxa::QUEUE_COMPUTE_0,
          device.latest_queue_submit_index(daxa::QUEUE_COMPUTE_0)}}});
  device.wait_on_submit({.queue=daxa::QUEUE_MAIN,.queue_submit_index=device.latest_queue_submit_index(daxa::QUEUE_MAIN)});
  auto const *batch=device.buffer_host_address_as<FractureBatchManifest>(fracture_batch_manifest).value();
  if (batch->status!=0u || batch->count>FRAGMENT_PLAN_CAPACITY) {
    std::cerr << "GPU fracture batch FAILED" << std::endl;std::abort();
  }
  if (batch->refused) std::cout << "[FRACTURE] capacity refusals=" << batch->refused << "; parents retained" << std::endl;
  return {batch->children,batch->children+batch->count};
}
