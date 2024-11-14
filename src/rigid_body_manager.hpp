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

  daxa::BufferId get_sim_config_buffer();
  daxa::BufferId get_previous_sim_config_buffer();
  daxa::BufferId get_collision_buffer();

  bool create(char const* name, std::shared_ptr<RendererManager> renderer, std::shared_ptr<GUIManager> gui, daxa_u32 iterations = DEFAULT_ITERATION_COUNT);
  void destroy();

  bool simulate();
  bool read_back_sim_config();
  SimConfig& get_sim_config_reference() {
    return *device.buffer_host_address_as<SimConfig>(sim_config_host_buffer).value();
  }

  bool update_resources(daxa::BufferId dispatch_buffer, daxa::BufferId rigid_bodies, daxa::BufferId aabbs, daxa::BufferId points_buffer);
  // NOTE: this function reset simulation configuration
  bool update_sim(daxa_u32 rigid_body_count);

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

  daxa::BufferId get_sim_config_host_buffer() {
    return sim_config_host_buffer;
  }

  bool is_dirty();
  void clean_dirty();

  // Task graph information for rigid body simulation
  daxa::TaskBuffer task_dispatch_buffer{{.initial_buffers = {}, .name = "RB_dispatch"}};
  daxa::TaskBuffer task_sim_config{{.initial_buffers = {}, .name = "RB_sim_config"}};
  daxa::TaskBuffer task_old_sim_config{{.initial_buffers = {}, .name = "RB_old_sim_config"}};
  daxa::TaskBuffer task_rigid_bodies{{.initial_buffers = {}, .name = "RB_task"}};
  daxa::TaskBuffer task_next_rigid_bodies{{.initial_buffers = {}, .name = "RB_previous_task"}};
  daxa::TaskBuffer task_aabbs{{.initial_buffers = {}, .name = "RB_aabb_task"}};
  daxa::TaskBuffer task_collisions{{.initial_buffers = {}, .name = "RB_collisions"}};
  daxa::TaskBuffer task_old_collisions{{.initial_buffers = {}, .name = "RB_old_collisions"}};
  daxa::TaskBuffer task_points{{.initial_buffers = {}, .name = "RB_points"}};

private: 
  void record_read_back_sim_config_tasks(TaskGraph &readback_SC_TG);
  void record_update_sim_config_tasks(TaskGraph &update_SC_TG);
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
  SimFlag sim_flags = SimFlag::ADVECTION;
  // simulating flag update 
  bool sim_flag_dirty[DOUBLE_BUFFERING] = {};

  // Compute pipeline reference
  std::shared_ptr<daxa::ComputePipeline> pipeline_RC;
  std::shared_ptr<daxa::ComputePipeline> pipeline_BP;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CS_dispatcher;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CPS;
  std::shared_ptr<daxa::ComputePipeline> pipeline_CS;
  std::shared_ptr<daxa::ComputePipeline> pipeline;
  std::shared_ptr<daxa::ComputePipeline> create_points_pipeline;
  std::shared_ptr<daxa::ComputePipeline> update_pipeline;

  // TaskGraph for rigid body simulation
  TaskGraph RB_TG;

  // TaskGraph for read-back of simulation configuration
  TaskGraph readback_SC_TG;

  // TaskGraph to update simulation configuration
  TaskGraph update_SC_TG;

  daxa::BufferId sim_config_host_buffer;
  daxa::BufferId sim_config[DOUBLE_BUFFERING] = {};
  daxa::BufferId collisions[DOUBLE_BUFFERING] = {};
};

BB_NAMESPACE_END