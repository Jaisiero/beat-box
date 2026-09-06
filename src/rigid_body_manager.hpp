#pragma once

#include "defines.hpp"
#include "task_manager.hpp"
#include "acceleration_structure_manager.hpp"

BB_NAMESPACE_BEGIN

struct RendererManager;
struct GUIManager;

struct RigidBodyManager{

  explicit RigidBodyManager(daxa::Device& device, 
  std::shared_ptr<TaskManager> task_manager, std::shared_ptr<AccelerationStructureManager> accel_struct_mngr);
  ~RigidBodyManager();

  bool create(char const* name, std::shared_ptr<RendererManager> renderer, std::shared_ptr<GUIManager> gui, daxa_u32 iterations = DEFAULT_ITERATION_COUNT);
  void destroy();

  bool simulate();
  // true requires wait_for_simulation() after the latest simulate(), with no subsequent reset or parity change.
  bool read_back_sim_config(bool completed_simulation_snapshot = false);
  SimConfig& get_sim_config_reference();

  bool update();
  bool update_resources();
  // NOTE: this function reset simulation configuration
  bool update_sim();
  void skip_warm_starting_once() {
    suppress_warm_starting_once = true;
  }
  
  bool update_active_rigid_body_list();

  SimFlag get_sim_flags() const {
    return sim_flags;
  }

  // Flag changes do NOT mark the sim dirty: the per-step "reset sim config" task
  // already uploads `sim_flags` every step, so the toggle propagates within a frame.
  // Marking dirty ran update_sim(), whose FULL SimConfig rewrite zeroes the previous
  // step's g_c_info.collision_count - the next warm-start pass then found ZERO old
  // manifolds, wiping every persisted lambda. Visible effect (user-found): pressing
  // TAB (debug overlay) made the whole settled pile twitch and reposition.
  SimFlag clear_sim_flags(SimFlag flags) {
    sim_flags &= ~flags;
    return sim_flags;
  }

  SimFlag set_sim_flags(SimFlag flags) {
    sim_flags |= flags;
    return sim_flags;
  }

  void set_sim_type(SimSolverType type) {
    solver_type = type;
    for(auto i = 0u; i < DOUBLE_BUFFERING; ++i) {
      sim_flag_dirty[i] = true;
    }
  }

  SimSolverType get_sim_type() const {
    return solver_type;
  }

  daxa::BufferId get_sim_config_host_buffer();
  daxa::BufferId get_lbvh_node_buffer();

  bool is_dirty();
  void clean_dirty();

  // Task graph information for rigid body simulation
  daxa::TaskBuffer task_sim_config_host{{.buffer = {}, .name = "RB_sim_config_host_task"}};
  daxa::TaskBuffer task_sim_config{{.buffer = {}, .name = "RB_sim_config_task"}};
  daxa::TaskBuffer task_old_sim_config{{.buffer = {}, .name = "RB_old_sim_config_task"}};
  daxa::TaskBuffer task_morton_codes{{.buffer = {}, .name = "RB_morton_code_task"}};
  daxa::TaskBuffer task_tmp_morton_codes{{.buffer = {}, .name = "RB_sorted_morton_code_task"}};
  daxa::TaskBuffer task_radix_sort_histograms{{.buffer = {}, .name = "RB_radix_sort_histogram_task"}};
  daxa::TaskBuffer task_rigid_bodies{{.buffer = {}, .name = "RB_task"}};
  daxa::TaskBuffer task_lbvh_nodes{{.buffer = {}, .name = "RB_lbvh_node_task"}};
  daxa::TaskBuffer task_previous_lbvh_nodes{{.buffer = {}, .name = "RB_previous_lbvh_node_task"}};
  daxa::TaskBuffer task_lbvh_construction_info{{.buffer = {}, .name = "RB_lbvh_construction_info_task"}};
  daxa::TaskBuffer task_rigid_body_entries{{.buffer = {}, .name = "RB_rigid_body_entry_task"}};
  daxa::TaskBuffer task_previous_rigid_body_entries{{.buffer = {}, .name = "RB_previous_rigid_body_entry_task"}};
  daxa::TaskBuffer task_broad_phase_collisions{{.buffer = {}, .name = "RB_broad_phase_collision_task"}};
  daxa::TaskBuffer task_rigid_body_scratch{{.buffer = {}, .name = "RB_sorted_task"}};
  daxa::TaskBuffer task_previous_rigid_bodies{{.buffer = {}, .name = "RB_previous_task"}};
  daxa::TaskBuffer task_rigid_body_link_manifolds{{.buffer = {}, .name = "rigid_body_link_manifold_task"}};
  daxa::TaskBuffer task_previous_rigid_body_link_manifolds{{.buffer = {}, .name = "previous_rigid_body_link_manifold_task"}};
  daxa::TaskBuffer task_next_rigid_bodies{{.buffer = {}, .name = "RB_next_task"}};
  daxa::TaskBuffer task_collision_entries{{.buffer = {}, .name = "RB_collision_entry_task"}};
  daxa::TaskBuffer task_collisions{{.buffer = {}, .name = "RB_collision_task"}};
  daxa::TaskBuffer task_collision_scratch{{.buffer = {}, .name = "RB_collision_scratch_task"}};
  daxa::TaskBuffer task_collision_entries_previous{{.buffer = {}, .name = "RB_previous_collision_entry_task"}};
  daxa::TaskBuffer task_old_collisions{{.buffer = {}, .name = "RB_old_collision_task"}};
  daxa::TaskBuffer task_active_rigid_bodies{{.buffer = {}, .name = "RB_active_rigid_body_task"}};
  daxa::TaskBuffer task_scratch_body_links{{.buffer = {}, .name = "RB_scratch_body_link_task"}};
  daxa::TaskBuffer task_body_links{{.buffer = {}, .name = "RB_body_link_task"}};
  daxa::TaskBuffer task_manifold_links{{.buffer = {}, .name = "RB_manifold_link_task"}};
  daxa::TaskBuffer task_islands{{.buffer = {}, .name = "RB_island_task"}};
  daxa::TaskBuffer task_previous_islands{{.buffer = {}, .name = "RB_previous_island_task"}};
  daxa::TaskBuffer task_contact_islands{{.buffer = {}, .name = "RB_contact_island_task"}};
  daxa::TaskBuffer task_previous_contact_islands{{.buffer = {}, .name = "RB_previous_contact_island_task"}};
  // graph coloring
  daxa::TaskBuffer task_body_color_mask{{.buffer = {}, .name = "RB_body_color_mask_task"}};
  daxa::TaskBuffer task_manifold_color{{.buffer = {}, .name = "RB_manifold_color_task"}};
  daxa::TaskBuffer task_body_color_owner{{.buffer = {}, .name = "RB_body_color_owner_task"}};
  daxa::TaskBuffer task_color_count{{.buffer = {}, .name = "RB_color_count_task"}};
  // AVBD
  daxa::TaskBuffer task_avbd_state{{.buffer = {}, .name = "RB_avbd_state_task"}};
  daxa::TaskBuffer task_avbd_body_color{{.buffer = {}, .name = "RB_avbd_body_color_task"}};
  // mouse pick-and-drag bridge (host-visible; input half host-written, state half GPU-written)
  daxa::TaskBuffer task_pick_state{{.buffer = {}, .name = "RB_pick_state_task"}};

  // Mouse pick input, called once per render frame from the render loop: the camera ray under the
  // cursor + button edges. `request` grabs (ray-cast) on the left-press edge; `dragging` keeps
  // the spring alive while held. The GPU consumes REQUEST; the existing post-simulation wait protects host access.
  void set_pick_input(daxa_f32vec3 ray_origin, daxa_f32vec3 ray_dir, bool request, bool dragging)
  {
    if (!initialized) { return; }
    auto *ps = device.buffer_host_address_as<PickState>(pick_state_buffer).value();
    ps->ray_origin = ray_origin;
    ps->ray_dir = ray_dir;
    // Render frames can outnumber physics steps: retain an unconsumed edge while
    // held. Release cancels it. The pick pass clears REQUEST after one raycast.
    bool const pending = dragging && (request || (ps->flags & BB_PICK_REQUEST) != 0u);
    ps->flags = (pending ? BB_PICK_REQUEST : 0u) | (dragging ? BB_PICK_DRAGGING : 0u);
  }

  // The currently grabbed body's persistent id (MAX_U32 = none) — a host read of the GPU-written
  // half of the pick bridge (single u32: tear-free). Lets the render loop suppress camera rotation
  // while the left button is dragging a body instead of orbiting.
  daxa_u32 get_picked_body()
  {
    if (!initialized) { return MAX_U32; }
    return device.buffer_host_address_as<PickState>(pick_state_buffer).value()->picked_id;
  }
  daxa_u32 get_grab_count() // diagnostic (BB_PICK_TRACE)
  {
    if (!initialized) { return 0u; }
    return device.buffer_host_address_as<PickState>(pick_state_buffer).value()->grab_count;
  }
  // voxel collision shape pools (static after scene load; host-writable, filled by the
  // SceneManager and addressed through SimConfig - no task-graph attachments needed)
  daxa::BufferId get_voxel_shapes_buffer() const { return voxel_shapes; }
  daxa::BufferId get_voxel_occupancy_buffer() const { return voxel_occupancy; }
  daxa::BufferId get_voxel_surface_buffer() const { return voxel_surface; }
  daxa::BufferId get_voxel_sdf_buffer() const { return voxel_sdf; }
  // GPU build of the derived voxel-shape pools (GPU-first): node SDF (exact separable EDT)
  // + packed surface-voxel list (canonical cell order, byte-identical to the CPU oracle's),
  // both from the occupancy bitmask; see voxel_sdf.slang. Called by the scene after
  // uploading shapes+occupancy; re-callable on future runtime shape edits (destruction).
  // cpu_*_reference: when BB_SDF_VERIFY is set, GPU results are read back and compared
  // against the CPU brute force (kept as the debug oracle per the GPU-first directive).
  // dirty (phase 2b-AS): when non-null, rebuild ONLY those shape indices (a fracture touches
  // a handful; the rest are already correct on the GPU). nullptr = rebuild all (load path).
  void build_voxel_pools_gpu(std::vector<VoxelShape> const &shapes,
                             std::vector<daxa_f32> const &cpu_sdf_reference,
                             std::vector<daxa_u32> const &cpu_surf_reference,
                             std::vector<VoxelShapeDerived> const &cpu_derived_reference,
                             std::vector<daxa_u32> const *dirty = nullptr);
  // GPU build of every voxel BODY's BLAS AABB range, written straight into the AS
  // manager's primitive scratch buffer (called between its host upload and the BLAS
  // build). bodies = (shape index, first-Aabb offset) per voxel body, in body order.
  void build_voxel_prims_gpu(std::vector<VoxelShape> const &shapes,
                             std::vector<std::pair<daxa_u32, daxa_u32>> const &bodies,
                             daxa::BufferId prims_buffer,
                             std::vector<Aabb> const &cpu_reference,
                             daxa::BufferId publication_bodies = {}, daxa::BufferId publication_instances = {},
                             daxa_u32 live_count = 0u);
  // GPU material seeding, partitioning and component planning. Only the compact
  // build manifest leaves the device; labels are downloaded only for verification.
  void carve_and_label(VoxelShape const &shape, daxa_u32 body_id,
                       std::vector<daxa_u32> &out_labels, std::vector<FragmentComponent> &out_components,
                       std::vector<FractureChildAllocation> &allocations);
  std::vector<FractureParentContext> read_fracture_contexts();
  void verify_fracture_events_gpu();
  void verify_fracture_allocator_gpu();
  // Initial authored occupancy upload; runtime occupancy remains GPU-owned.
  void upload_voxel_occupancy(std::span<daxa_u32 const> occupancy);
  std::vector<daxa_u32> read_voxel_occupancy(daxa_u32 count);
  void initialize_fracture_allocator(std::array<daxa_u32, GPU_POOL_COUNT> const &high_water,
                                     std::span<RigidBody const> spawn_templates = {});
  FractureSceneEditManifest edit_fracture_scene_gpu(bool cull, bool spawn, daxa_f32 kill_y);
  std::vector<FractureBatchChild> fracture_batch_gpu(std::span<FracturePartitionInput const> inputs);
  std::vector<GpuFreeList> read_fracture_pools();
  void read_voxel_derived(daxa_u32 count, std::vector<VoxelShapeDerived> &out);
  // Compact FRACTURE event bridge: geometric payloads stay in GPU pending storage.
  // Read/ack only after simulation completion publishes the host-visible summary.
  // A scalar generation alone is not a synchronization primitive.
  void acknowledge_fracture_events(daxa_u32 serial)
  {
    device.buffer_host_address_as<FractureEventBuffer>(fracture_events_buffer).value()->consumed_serial = serial;
  }
  FractureEventBuffer const *get_fracture_events()
  {
    if (!initialized || fracture_events_buffer.is_empty()) { return nullptr; }
    return device.buffer_host_address_as<FractureEventBuffer>(fracture_events_buffer).value();
  }
  void reset_fracture_events()
  {
    if (!initialized || fracture_events_buffer.is_empty()) { return; }
    auto *events = device.buffer_host_address_as<FractureEventBuffer>(fracture_events_buffer).value();
    *events = FractureEventBuffer{};
    events->scratch_addr = device.device_address(fracture_impact_scratch).value();
  }

private:
  daxa::InlineTaskInfo sim_config_readback_task();
  void record_read_back_sim_config_tasks(TaskGraph &out_readback_SC_TG);
  void record_update_sim_config_tasks(TaskGraph &out_update_SC_TG);
  void record_active_rigid_body_list_tasks(TaskGraph &ARB_TG);
  void update_buffers();
  void update_buffers(daxa_u32 current_frame);
  
  // Device reference
  daxa::Device& device;
  // B2: every GPU buffer this manager owns is created through create_owned() and registered here,
  // so destroy() frees EXACTLY what create() allocated by iterating this list — no more hand-edited
  // parallel destroy list that can drift (it did once: the graph-coloring/AVBD buffers were missing).
  // The lazily-created voxel pools stay explicit (is_empty()-guarded) as they have their own lifecycle.
  std::vector<daxa::BufferId> owned_buffers;
  daxa::BufferId create_owned(daxa::BufferInfo const &info)
  {
    auto id = device.create_buffer(info);
    owned_buffers.push_back(id);
    return id;
  }
  // Initialization flag
  bool initialized = false;
  // iteration count
  daxa_u32 iteration_count = DEFAULT_ITERATION_COUNT;
  daxa_u32 tgs_substep_count = BB_TGS_SUBSTEPS;
  // Task manager reference
  std::shared_ptr<TaskManager> task_manager;
  // Back-references wired in create() — RAW on purpose (review v3): shared_ptr back-refs formed
  // reference cycles with RendererManager (which owns this manager), so no destructor ever ran.
  // All managers live strictly within main()'s scope; non-owning pointers are safe.
  RendererManager *renderer_manager = nullptr;
  GUIManager *gui_manager = nullptr;
  // Acceleration Structure manager reference
  std::shared_ptr<AccelerationStructureManager> accel_struct_mngr;
  // Simulation flags
  SimFlag sim_flags = SimFlag::ADVECTION
  | SimFlag::DEBUG_INFO
  | SimFlag::FRICTION
  | SimFlag::ACCUM_IMPULSE
  | SimFlag::WARM_STARTING
  | SimFlag::USE_GRAPH_COLORING
  // SLEEPING_ENABLED back ON by default (user decision 2026-07-03), closing the 2026-06-15
  // "dejemos de hacer trampas" era. The original objection was that sleeping froze visibly
  // interpenetrated states; since then the honest rest was actually built: exact SDF+SAT
  // voxel narrow phase, post-stab slop, convergence early-out - canonical scenes settle at
  // pen 0-5mm with fresh=0, so what sleeping freezes now is a CORRECT state. scene_7's
  // slow-relaxing internal debt (accepted as torture-test behavior, see the settle-tail
  // notes) sleeps once quiet instead of chewing itself for minutes. Press O to toggle off.
  | SimFlag::SLEEPING_ENABLED;
  // simulating flag update 
  bool sim_flag_dirty[DOUBLE_BUFFERING] = {};
  bool suppress_warm_starting_once = false;

  // Compute pipeline reference
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBD;
  std::shared_ptr<daxa::ComputePipeline> pipeline_GMC;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBRSH;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBSRS;
  std::shared_ptr<daxa::ComputePipeline> pipeline_SWS;
  std::shared_ptr<daxa::ComputePipeline> pipeline_VSB_INIT;
  std::shared_ptr<daxa::ComputePipeline> pipeline_VSB_AXIS;
  std::shared_ptr<daxa::ComputePipeline> pipeline_VSB_FIN;
  std::shared_ptr<daxa::ComputePipeline> pipeline_VSB_SURF;
  std::shared_ptr<daxa::ComputePipeline> pipeline_VSB_INERTIA;
  std::shared_ptr<daxa::ComputePipeline> pipeline_VSB_PRIMS;
  std::shared_ptr<daxa::ComputePipeline> pipeline_fragment_finalize;
  std::shared_ptr<daxa::ComputePipeline> pipeline_census_init, pipeline_census_accumulate, pipeline_census_compact;
  daxa::BufferId fracture_census_scratch{}, fracture_census_output{}, fracture_remap_buffer{};
  std::shared_ptr<daxa::ComputePipeline> pipeline_fragment_plan, pipeline_fracture_allocate, pipeline_fragment_batch_pack;
  std::shared_ptr<daxa::ComputePipeline> pipeline_impact_reset, pipeline_impact_select, pipeline_impact_publish;
  void record_fragment_census(daxa::CommandRecorder &rec, daxa_u32vec3 dims, daxa_u64 labels_addr, bool compact, daxa_u32 body_id = MAX_U32);
  void record_fracture_partition(daxa::CommandRecorder &rec,
      daxa_u32 body_id, daxa_u32 body_count, daxa_u64 batch_addr, daxa_u32 recorded_passes,
      bool compact, daxa::TimelineQueryPool *queries = nullptr);
  std::shared_ptr<daxa::ComputePipeline> pipeline_body_list;
  std::shared_ptr<daxa::ComputePipeline> pipeline_fracture_setup, pipeline_fracture_gather, pipeline_fracture_scene_edit, pipeline_fracture_layout;
  daxa::BufferId fracture_plan_manifest{};
  std::shared_ptr<daxa::ComputePipeline> pipeline_VFR_CARVE;
  std::shared_ptr<daxa::ComputePipeline> pipeline_VFR_VORONOI;
  std::shared_ptr<daxa::ComputePipeline> pipeline_VFR_FLOOD_INIT;
  std::shared_ptr<daxa::ComputePipeline> pipeline_VFR_FLOOD_STEP;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBLBVHGH;
  std::shared_ptr<daxa::ComputePipeline> pipeline_BBBLBVHGH;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CBBLBVHGH;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBL;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBR;
  std::shared_ptr<daxa::ComputePipeline> pipeline_BP;
  std::shared_ptr<daxa::ComputePipeline> pipeline_NPD;
  std::shared_ptr<daxa::ComputePipeline> pipeline_NP;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CHS; // canonical chain sort (determinism)
  std::shared_ptr<daxa::ComputePipeline> pipeline_PS;  // mouse pick-and-drag spring
  std::shared_ptr<daxa::ComputePipeline> pipeline_CS_dispatcher;
  std::shared_ptr<daxa::ComputePipeline> pipeline_ID;
  std::shared_ptr<daxa::ComputePipeline> pipeline_IC;
  std::shared_ptr<daxa::ComputePipeline> pipeline_IB;
  std::shared_ptr<daxa::ComputePipeline> pipeline_IPS;
  std::shared_ptr<daxa::ComputePipeline> pipeline_IBL;
  std::shared_ptr<daxa::ComputePipeline> pipeline_SBLI;
  std::shared_ptr<daxa::ComputePipeline> pipeline_MIB;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CIG;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CID;
  std::shared_ptr<daxa::ComputePipeline> pipeline_MIPS;
  std::shared_ptr<daxa::ComputePipeline> pipeline_IML;
  std::shared_ptr<daxa::ComputePipeline> pipeline_SMLI;
  std::shared_ptr<daxa::ComputePipeline> pipeline_advect;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CPS;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CS;
  std::shared_ptr<daxa::ComputePipeline> pipeline_IP;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CSR;
  // graph coloring
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCD;  // dispatcher
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCSD; // per-color solve dispatcher (skips empty colors)
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCR;  // reset
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCOR; // owner reset
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCP1; // assign phase 1
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCP2; // assign phase 2
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCV;  // validate
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCV2; // validate2 (satbody diag, TEMP)
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCS_CPS; // per-color pre-solver
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCS_CS;  // per-color solver
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCS_CSR; // per-color relax
  std::shared_ptr<daxa::ComputePipeline> pipeline_SLR; // sleep reduce (per-body quiet timer)
  std::shared_ptr<daxa::ComputePipeline> pipeline_SLV; // sleep veto (loud contact partners veto sleeping)
  std::shared_ptr<daxa::ComputePipeline> pipeline_SLA; // sleep apply (quiet + un-vetoed -> SLEEPING flag)
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_CR;   // AVBD body-color reset
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_CRND; // AVBD body-color JP round
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_CV;   // AVBD body-color validate
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_CDISP; // AVBD per-color primal dispatch args (skip empty body colors)
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_MAXD;  // AVBD max support-depth reduction (A2)
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_CASCD; // AVBD per-(layer,color) cascade dispatch args (skip empty layers) (A2)
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_CMT;  // AVBD body-color commit (race-free split)
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_PRE;  // AVBD prepare (inertial target)
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_FIN;  // AVBD finalize (velocity reconstruction)
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_WS;   // AVBD lambda/k warm-start scaling
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_PRIM; // AVBD per-color primal 6x6 block solve
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_DUAL; // AVBD dual lambda/penalty updates
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_IMPJ; // inelastic impact: per-contact impulse
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_IMPA; // inelastic impact: per-body apply
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_PKTR; // deep-pocket oscillator trace (diagnostic)
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_DRST; // shock propagation: depth reset
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_DRLX; // shock propagation: depth BFS relax
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCS_CPS_OV; // overflow pre-solver (serial)
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCS_CS_OV;  // overflow solver (serial)
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCS_CSR_OV; // overflow relax (serial)
  std::shared_ptr<daxa::ComputePipeline> create_points_pipeline;
  std::shared_ptr<daxa::ComputePipeline> update_pipeline;

  // Per-solver sim task graphs: shared setup + only the active solver's passes, so PGS/TGS no longer
  // pay AVBD's ~847 dispatches/frame (and vice versa). simulate() executes the one matching solver_type.
  TaskGraph RB_TG_pgs;   // PGS / PGS_SOFT
  TaskGraph RB_TG_avbd;  // AVBD
  TaskGraph RB_TG_tgs;   // TGS_SOFT

  // TaskGraph for read-back of simulation configuration
  TaskGraph readback_SC_TG;

  // TaskGraph to update simulation configuration
  TaskGraph update_SC_TG;

  // Task graph for uploading active rigid body list
  TaskGraph ARB_TG;

  daxa::BufferId sim_config_host_buffer[DOUBLE_BUFFERING] = {};
  daxa::BufferId sim_config[DOUBLE_BUFFERING] = {};
  daxa::BufferId pick_state_buffer = {}; // host-visible bridge (see task_pick_state)
  bool narrow_phase_timing = false;
  bool narrow_phase_query_pending = false;
  daxa::TimelineQueryPool narrow_phase_queries = {};
  daxa::TimelineQueryPool solver_stage_queries = {};
  bool solver_stage_query_pending = false;
  SimSolverType stage_query_solver = SimSolverType::AVBD;
  daxa::BufferId morton_codes = {};
  daxa::BufferId tmp_morton_codes = {};
  daxa::BufferId lbvh_nodes[DOUBLE_BUFFERING] = {};
  daxa::BufferId lbvh_construction_info = {};
  daxa::BufferId broad_phase_collisions[DOUBLE_BUFFERING] = {};
  daxa::BufferId global_histograms[DOUBLE_BUFFERING] = {};
  daxa::BufferId collision_entries[DOUBLE_BUFFERING] = {};
  daxa::BufferId collisions[DOUBLE_BUFFERING] = {};
  daxa::BufferId collision_scratch = {};
  daxa::BufferId rigid_body_entries[DOUBLE_BUFFERING] = {};
  daxa::BufferId rigid_body_scratch = {};
  daxa::BufferId active_rigid_bodies[DOUBLE_BUFFERING] = {};
  daxa::BufferId rigid_body_link_manifolds[DOUBLE_BUFFERING] = {};
  daxa::BufferId body_links[DOUBLE_BUFFERING] = {};
  daxa::BufferId manifold_links[DOUBLE_BUFFERING] = {};
  daxa::BufferId scratch_body_links[DOUBLE_BUFFERING] = {};
  daxa::BufferId island_buffer[DOUBLE_BUFFERING] = {};
  daxa::BufferId contact_island_buffer[DOUBLE_BUFFERING] = {};
  // graph coloring
  daxa::BufferId body_color_mask = {};
  daxa::BufferId manifold_color = {};
  daxa::BufferId body_color_owner = {};
  daxa::BufferId color_count = {};
  // AVBD
  daxa::BufferId avbd_state = {};
  daxa::BufferId avbd_body_color = {};
  // voxel collision shapes
  daxa::BufferId voxel_shapes = {};
  daxa::BufferId voxel_occupancy = {};
  daxa::BufferId voxel_surface = {};
  daxa::BufferId voxel_sdf = {};
  daxa::BufferId voxel_sdf_scratch[2] = {}; // squared-distance fields (solid/empty) for the GPU EDT
  daxa::BufferId voxel_derived = {};        // VoxelShapeDerived per shape (GPU mass-property reduce)
  daxa::BufferId fracture_allocator = {}, fracture_allocations = {}, fracture_batch_manifest = {};
  daxa::BufferId fracture_scene_manifest = {}, fracture_spawn_templates = {}, fracture_spawn_body = {};
  daxa_u32 fracture_spawn_template_count = 0u;
  daxa::BufferId fracture_contexts = {};
  daxa::BufferId fracture_impact_scratch = {};
  daxa::BufferId fracture_events_buffer = {}; // host-visible impact->host fracture bridge

  // Simulation configuration. AVBD is the default solver (user decision after the A/B
  // campaign: rests flush at pen~0 vs 13mm Baumgarte sink, true zero residual velocity,
  // ~40% cheaper, and it holds towers/piles PGS_SOFT cannot); PGS_SOFT stays on key 2.
  SimSolverType solver_type = SimSolverType::AVBD;

  daxa_u32 shift = 0;
};

BB_NAMESPACE_END
