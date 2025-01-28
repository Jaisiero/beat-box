#pragma once

#include "defines.hpp"
#include "task_manager.hpp"
#include "acceleration_structure_manager.hpp"

BB_NAMESPACE_BEGIN

class RendererManager;
class GUIManager;

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
  
  bool update_active_rigid_body_list();

  SimFlag get_sim_flags() const {
    return sim_flags;
  }

  SimFlag clear_sim_flags(SimFlag flags) {
    sim_flags &= ~flags;
    for(auto i = 0u; i < DOUBLE_BUFFERING; ++i) {
      sim_flag_dirty[i] = true;
    }
    return sim_flags;
  }

  SimFlag set_sim_flags(SimFlag flags) {
    sim_flags |= flags;
    for(auto i = 0u; i < DOUBLE_BUFFERING; ++i) {
      sim_flag_dirty[i] = true;
    }
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
  daxa::TaskBuffer task_sim_config_host{{.initial_buffers = {}, .name = "RB_sim_config_host_task"}};
  daxa::TaskBuffer task_sim_config{{.initial_buffers = {}, .name = "RB_sim_config_task"}};
  daxa::TaskBuffer task_old_sim_config{{.initial_buffers = {}, .name = "RB_old_sim_config_task"}};
  daxa::TaskBuffer task_morton_codes{{.initial_buffers = {}, .name = "RB_morton_code_task"}};
  daxa::TaskBuffer task_tmp_morton_codes{{.initial_buffers = {}, .name = "RB_sorted_morton_code_task"}};
  daxa::TaskBuffer task_radix_sort_histograms{{.initial_buffers = {}, .name = "RB_radix_sort_histogram_task"}};
  daxa::TaskBuffer task_rigid_bodies{{.initial_buffers = {}, .name = "RB_task"}};
  daxa::TaskBuffer task_lbvh_nodes{{.initial_buffers = {}, .name = "RB_lbvh_node_task"}};
  daxa::TaskBuffer task_lbvh_construction_info{{.initial_buffers = {}, .name = "RB_lbvh_construction_info_task"}};
  daxa::TaskBuffer task_rigid_body_entries{{.initial_buffers = {}, .name = "RB_rigid_body_entry_task"}};
  daxa::TaskBuffer task_broad_phase_collisions{{.initial_buffers = {}, .name = "RB_broad_phase_collision_task"}};
  daxa::TaskBuffer task_rigid_body_sorted{{.initial_buffers = {}, .name = "RB_sorted_task"}};
  daxa::TaskBuffer task_previous_rigid_bodies{{.initial_buffers = {}, .name = "RB_previous_task"}};
  daxa::TaskBuffer task_rigid_body_link_manifolds{{.initial_buffers = {}, .name = "rigid_body_link_manifold_task"}};
  daxa::TaskBuffer task_previous_rigid_body_link_manifolds{{.initial_buffers = {}, .name = "previous_rigid_body_link_manifold_task"}};
  daxa::TaskBuffer task_next_rigid_bodies{{.initial_buffers = {}, .name = "RB_next_task"}};
  daxa::TaskBuffer task_collisions{{.initial_buffers = {}, .name = "RB_collision_task"}};
  daxa::TaskBuffer task_old_collisions{{.initial_buffers = {}, .name = "RB_old_collision_task"}};
  daxa::TaskBuffer task_active_rigid_bodies{{.initial_buffers = {}, .name = "RB_active_rigid_body_task"}};
  daxa::TaskBuffer task_scratch_body_links{{.initial_buffers = {}, .name = "RB_scratch_body_link_task"}};
  daxa::TaskBuffer task_body_links{{.initial_buffers = {}, .name = "RB_body_link_task"}};
  daxa::TaskBuffer task_manifold_links{{.initial_buffers = {}, .name = "RB_manifold_link_task"}};
  daxa::TaskBuffer task_islands{{.initial_buffers = {}, .name = "RB_island_task"}};
  daxa::TaskBuffer task_contact_islands{{.initial_buffers = {}, .name = "RB_contact_island_task"}};

private: 
  void record_read_back_sim_config_tasks(TaskGraph &readback_SC_TG);
  void record_update_sim_config_tasks(TaskGraph &update_SC_TG);
  void record_active_rigid_body_list_upload_tasks(TaskGraph &ARB_TG);
  void update_buffers();
  
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
  ;
  // simulating flag update 
  bool sim_flag_dirty[DOUBLE_BUFFERING] = {};

  // Compute pipeline reference
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBD;
  std::shared_ptr<daxa::ComputePipeline> pipeline_GMC;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBRSH;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBSRS;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBLBVHGH;
  std::shared_ptr<daxa::ComputePipeline> pipeline_BBBLBVHGH;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBL;
  std::shared_ptr<daxa::ComputePipeline> pipeline_RBR;
  std::shared_ptr<daxa::ComputePipeline> pipeline_BP;
  std::shared_ptr<daxa::ComputePipeline> pipeline_NPD;
  std::shared_ptr<daxa::ComputePipeline> pipeline_NP;
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
  daxa::BufferId collisions[DOUBLE_BUFFERING] = {};
  daxa::BufferId rigid_body_entries[DOUBLE_BUFFERING] = {};
  daxa::BufferId rigid_body_sorted = {};
  daxa::BufferId active_rigid_bodies[DOUBLE_BUFFERING] = {};
  daxa::BufferId rigid_body_link_manifolds[DOUBLE_BUFFERING] = {};
  daxa::BufferId body_links[DOUBLE_BUFFERING] = {};
  daxa::BufferId manifold_links[DOUBLE_BUFFERING] = {};
  daxa::BufferId scratch_body_links[DOUBLE_BUFFERING] = {};
  daxa::BufferId island_buffer[DOUBLE_BUFFERING] = {};
  daxa::BufferId contact_island_buffer[DOUBLE_BUFFERING] = {};

  // Simulation configuration
  SimSolverType solver_type = SimSolverType::PGS_SOFT;

  daxa_u32 shift = 0;
};

BB_NAMESPACE_END