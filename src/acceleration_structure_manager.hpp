#pragma once

#include "defines.hpp"
#include "task_manager.hpp"

BB_NAMESPACE_BEGIN

class RendererManager;

struct AccelerationStructureManager
{
  // Daxa device
  daxa::Device &device;
  // Initialization flag
  bool initialized = false;
  // Task manager reference
  std::shared_ptr<TaskManager> task_manager;
  // Renderer manager reference
  std::shared_ptr<RendererManager> renderer_manager;
  // Compute pipeline for updating acceleration structures
  std::shared_ptr<daxa::ComputePipeline> update_pipeline;

  // TODO: temporary
  static constexpr u32 MAX_ACCELERATION_STRUCTURE_COUNT = 1024;
  static constexpr u32 AVERAGE_AS_SIZE = 1024 * 1024;

  // Alignment of the acceleration structure build offset
  static constexpr u64 ACCELERATION_STRUCTURE_BUILD_OFFSET_ALIGMENT = 256;
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
  // Buffer for the points
  daxa::BufferId points_buffer[DOUBLE_BUFFERING] = {};

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

  // Offset for point BLAS buffer
  u32 point_blas_buffer_offset = 0;
  // Buffer for point BLAS
  daxa::BufferId point_blas_buffer = {};
  // Sub-allocated buffer for point BLAS
  std::vector<daxa::BlasId> point_blas = {};

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

  // task for the RigidBodies
  daxa::TaskBuffer task_rigid_body_buffer{{.name = "rigid_body_buffer_task"}};
  // task for the AABB buffer
  daxa::TaskBuffer task_aabb_buffer{{.name = "aabb_buffer_task"}};
  // task for the points AABB buffer
  daxa::TaskBuffer task_points_aabb_buffer{{.name = "points_aabb_buffer_task"}};
  // task for the BLAS
  daxa::TaskBlas task_blas{{.name = "blas_task"}};
  // task for the TLAS
  daxa::TaskTlas task_tlas{{.name = "tlas_task"}};

  // TaskGraph for building acceleration structures
  TaskGraph AS_build_TG;
  // TaskGraph for updating acceleration structures
  TaskGraph TLAS_update_TG;

  daxa::TaskBuffer task_dispatch_buffer{{.name = "dispatch_buffer"}};
  daxa::TaskBuffer task_sim_config{{.name = "sim_config"}};
  daxa::TaskBuffer task_blas_instance_data{{.name = "blas_instance_data"}};
  daxa::TaskBuffer task_collisions{{.name = "RB_collisions"}};

  // TODO: is this the right way to do this?
  daxa::BufferId sim_host_buffer;

  explicit AccelerationStructureManager(daxa::Device &device, std::shared_ptr<TaskManager> task_manager);
  ~AccelerationStructureManager();

  bool create(std::shared_ptr<RendererManager> renderer);
  void destroy();
  void free_accel_structs();

  daxa::TlasId get_tlas();
  daxa::BufferId get_rigid_body_buffer();
  daxa::BufferId get_points_buffer();


  // NOTE: queue sync assures double buffering is filled
  void build_AS();
  bool build_accel_structs(std::vector<RigidBody> &rigid_bodies, std::vector<Aabb> const &primitives);
  void update_TLAS(daxa::BufferId sim_config);
  bool update();
  bool update_TLAS_resources(daxa::BufferId dispatch_buffer, daxa::BufferId collisions, daxa::BufferId sim_config_host_buffer);

private:
  void record_accel_struct_tasks(TaskGraph &AS_TG);
  void record_update_TLAS_tasks(TaskGraph &TLAS_TG, std::shared_ptr<daxa::ComputePipeline> update_AS_pipeline);
  void update_buffers(daxa::BufferId sim_config);

}; // struct AccelerationStructureManager

BB_NAMESPACE_END