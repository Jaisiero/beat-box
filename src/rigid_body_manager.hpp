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
  bool read_back_sim_config();
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
  // voxel collision shape pools (static after scene load; host-writable, filled by the
  // SceneManager and addressed through SimConfig - no task-graph attachments needed)
  daxa::BufferId get_voxel_shapes_buffer() const { return voxel_shapes; }
  daxa::BufferId get_voxel_occupancy_buffer() const { return voxel_occupancy; }
  daxa::BufferId get_voxel_surface_buffer() const { return voxel_surface; }

private:
  void record_read_back_sim_config_tasks(TaskGraph &out_readback_SC_TG);
  void record_update_sim_config_tasks(TaskGraph &out_update_SC_TG);
  void record_active_rigid_body_list_upload_tasks(TaskGraph &ARB_TG);
  void update_buffers();
  void update_buffers(daxa_u32 current_frame);
  
  // Device reference
  daxa::Device& device;
  // Initialization flag
  bool initialized = false;
  // iteration count
  daxa_u32 iteration_count = DEFAULT_ITERATION_COUNT;
  // Task manager reference
  std::shared_ptr<TaskManager> task_manager;
  // Renderer manager reference
  std::shared_ptr<RendererManager> renderer_manager;
  // GUI manager reference
  std::shared_ptr<GUIManager> gui_manager;
  // Acceleration Structure manager reference
  std::shared_ptr<AccelerationStructureManager> accel_struct_mngr;
  // Simulation flags
  SimFlag sim_flags = SimFlag::ADVECTION
  | SimFlag::DEBUG_INFO
  | SimFlag::FRICTION
  | SimFlag::ACCUM_IMPULSE
  | SimFlag::WARM_STARTING
  | SimFlag::USE_GRAPH_COLORING
  | SimFlag::SLEEPING_ENABLED
  ;
  // simulating flag update 
  bool sim_flag_dirty[DOUBLE_BUFFERING] = {};
  bool suppress_warm_starting_once = false;

  // Compute pipeline reference
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBD;
  std::shared_ptr<daxa::ComputePipeline> pipeline_GMC;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBRSH;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBSRS;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBLBVHGH;
  std::shared_ptr<daxa::ComputePipeline> pipeline_BBBLBVHGH;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CBBLBVHGH;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBL;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBR;
  std::shared_ptr<daxa::ComputePipeline> pipeline_BP;
  std::shared_ptr<daxa::ComputePipeline> pipeline_NPD;
  std::shared_ptr<daxa::ComputePipeline> pipeline_NP;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CHS; // canonical chain sort (determinism)
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
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_PRE;  // AVBD prepare (inertial target)
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_FIN;  // AVBD finalize (velocity reconstruction)
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_WS;   // AVBD lambda/k warm-start scaling
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_PRIM; // AVBD per-color primal 6x6 block solve
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_DUAL; // AVBD dual lambda/penalty updates
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_IMPJ; // inelastic impact: per-contact impulse
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_IMPA; // inelastic impact: per-body apply
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_DRST; // shock propagation: depth reset
  std::shared_ptr<daxa::ComputePipeline> pipeline_AVBD_DRLX; // shock propagation: depth BFS relax
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCS_CPS_OV; // overflow pre-solver (serial)
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCS_CS_OV;  // overflow solver (serial)
  std::shared_ptr<daxa::ComputePipeline> pipeline_GCS_CSR_OV; // overflow relax (serial)
  std::shared_ptr<daxa::ComputePipeline> create_points_pipeline;
  std::shared_ptr<daxa::ComputePipeline> update_pipeline;

  // TaskGraph for rigid body simulation
  TaskGraph RB_TG;

  // TaskGraph for read-back of simulation configuration
  TaskGraph readback_SC_TG;

  // TaskGraph to update simulation configuration
  TaskGraph update_SC_TG;

  // Task graph for uploading active rigid body list
  TaskGraph ARB_TG;

  daxa::BufferId sim_config_host_buffer[DOUBLE_BUFFERING] = {};
  daxa::BufferId sim_config[DOUBLE_BUFFERING] = {};
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

  // Simulation configuration. AVBD is the default solver (user decision after the A/B
  // campaign: rests flush at pen~0 vs 13mm Baumgarte sink, true zero residual velocity,
  // ~40% cheaper, and it holds towers/piles PGS_SOFT cannot); PGS_SOFT stays on key 2.
  SimSolverType solver_type = SimSolverType::AVBD;

  daxa_u32 shift = 0;
};

BB_NAMESPACE_END
