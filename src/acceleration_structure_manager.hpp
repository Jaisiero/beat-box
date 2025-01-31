#pragma once

#include "defines.hpp"
#include "task_manager.hpp"

BB_NAMESPACE_BEGIN

class RendererManager;
class RigidBodyManager;
class GUIManager;

struct AccelerationStructureManager
{
  // task for the AABB buffer
  daxa::TaskBuffer task_aabb_buffer{{.name = "aabb_buffer_task"}};
  // task for points AABB buffer
  // task for the BLAS
  daxa::TaskBlas task_blas{{.name = "blas_task"}};
  // task for the TLAS
  daxa::TaskTlas task_tlas{{.name = "tlas_task"}};

  // TaskGraph for building acceleration structures
  TaskGraph AS_build_TG;
  // TaskGraph for updating acceleration structures
  TaskGraph TLAS_update_TG;
  // TaskGraph for updating acceleration structures
  TaskGraph AS_update_buffers_TG;

  daxa::TaskBuffer task_dispatch_buffer{{.name = "dispatch_buffer"}};
  daxa::TaskBuffer task_blas_instance_data{{.name = "blas_instance_data"}};

  explicit AccelerationStructureManager(daxa::Device &device, std::shared_ptr<TaskManager> task_manager);
  ~AccelerationStructureManager();

  bool create(std::shared_ptr<RendererManager> renderer, std::shared_ptr<RigidBodyManager> rigid_body, std::shared_ptr<GUIManager> gui);
  void destroy();
  void free_accel_structs();

  daxa::TlasId get_tlas();
  daxa::BufferId get_previous_rigid_body_buffer();
  daxa::BufferId get_rigid_body_buffer();
  daxa::BufferId get_next_rigid_body_buffer();


  // NOTE: queue sync assures double buffering is filled
  void build_AS();
  bool build_accel_structs(std::vector<RigidBody> &rigid_bodies, std::vector<Aabb> const &primitives);
  void update_TLAS();
  bool update_TLAS_resources(daxa::BufferId dispatch_buffer);
  void update_AS_buffers();

private:
  // TODO: temporary
  static constexpr u32 MAX_ACCELERATION_STRUCTURE_COUNT = 1024;
  static constexpr u32 AVERAGE_AS_SIZE = 1024 * 1024;
  
  // Alignment of the acceleration structure build offset
  static constexpr u64 ACCELERATION_STRUCTURE_BUILD_OFFSET_ALIGMENT = 256;
  // Daxa device
  daxa::Device &device;
  // Initialization flag
  bool initialized = false;
  // Task manager reference
  std::shared_ptr<TaskManager> task_manager;
  // Renderer manager reference
  std::shared_ptr<RendererManager> renderer_manager;
  // Rigid body manager reference
  std::shared_ptr<RigidBodyManager> rigid_body_manager;
  // GUI manager reference
  std::shared_ptr<GUIManager> gui_manager;
  // Compute pipeline for updating acceleration structures
  std::shared_ptr<daxa::ComputePipeline> update_pipeline;
  // Alignment of the scratch buffer
  u32 acceleration_structure_scratch_offset_alignment = 0;

  // Offset for the scratch buffer for RigidBodies
  u32 rigid_body_scratch_offset = 0;
  // Scratch buffer for RigidBodies
  daxa::BufferId rigid_body_scratch_buffer = {};
  // Rigid body count
  u32 current_rigid_body_count = 0;
  // Rigid body previous count
  u32 previous_rigid_body_count = 0; 
  // Buffer for the RigidBodies
  daxa::BufferId rigid_body_buffer[DOUBLE_BUFFERING] = {};

  // Offset for the scratch buffer for primitives
  u32 primitive_scratch_offset = 0;
  // Scratch buffer for primitives
  daxa::BufferId primitive_scratch_buffer = {};
  // Primitive count
  u32 current_primitive_count = 0;
  // Previous primitive count
  u32 previous_primitive_count = 0;
  // Buffer for the primitives
  daxa::BufferId primitive_buffer = {};

  // Offset for the BLAS scratch buffer
  u32 proc_blas_scratch_offset = 0;
  // Scratch buffer for the BLAS
  daxa::BufferId proc_blas_scratch_buffer = {};
  // Offset for the BLAS buffer
  u32 proc_blas_buffer_offset = 0;
  // Buffer for the BLAS
  daxa::BufferId proc_blas_buffer = {};
  // Sub-allocated buffer for the BLAS
  std::vector<daxa::BlasId> proc_blas = {};

  // Buffer for the LBVH BLAS
  daxa::BlasId lbvh_blas[DOUBLE_BUFFERING] = {};

  // Buffer for the TLAS
  daxa::BufferId proc_tlas_buffer = {};
  // Scratch buffer for the TLAS
  daxa::BufferId proc_tlas_scratch_buffer = {};

  // Build BlAS info
  std::vector<daxa::BlasBuildInfo> blas_build_infos = {};
  // Build sizes for the BLAS
  std::vector<daxa::AccelerationStructureBuildSizesInfo> blas_build_sizes = {};
  // Build geometry for the BLAS
  std::vector<std::vector<daxa::BlasAabbGeometryInfo>> blas_geometries = {};

  // Build TLAS info
  daxa::AccelerationStructureBuildSizesInfo tlas_build_sizes = {};
  // Build TLAS info
  daxa::TlasBuildInfo tlas_build_info = {};
  // BLAS instances
  daxa::BufferId blas_instances_buffer = {};
  // BLAS instances data
  std::array<daxa::TlasInstanceInfo, 1> tlas_info = {};
  // BLAS instances data
  daxa_BlasInstanceData *blas_instances_data = nullptr;
  // TLAS
  daxa::TlasId tlas[DOUBLE_BUFFERING] = {};

  bool update();
  void record_accel_struct_tasks(TaskGraph &AS_TG);
  void record_update_TLAS_tasks(TaskGraph &TLAS_TG, std::shared_ptr<daxa::ComputePipeline> update_AS_pipeline);
  void record_update_AS_buffers_tasks(TaskGraph &AS_buffers_TG);
  void update_buffers();

}; // struct AccelerationStructureManager

BB_NAMESPACE_END