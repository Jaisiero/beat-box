#pragma once

#include "defines.hpp"
#include "task_manager.hpp"
#include "free_list_pool.hpp" // incremental-AS BLAS/prim region allocator
#include <functional>

BB_NAMESPACE_BEGIN

struct RendererManager;
struct RigidBodyManager;
struct GUIManager;

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
  // TaskGraph for rebuilding the TLAS after instance data has been updated
  TaskGraph TLAS_build_TG;
  // TaskGraph for updating acceleration structures
  TaskGraph AS_update_buffers_TG;

  daxa::TaskBuffer task_dispatch_buffer{{.name = "dispatch_buffer"}};
  daxa::TaskBuffer task_blas_instance_data{{.name = "blas_instance_data"}};

  explicit AccelerationStructureManager(daxa::Device &device, std::shared_ptr<TaskManager> task_manager);
  ~AccelerationStructureManager();

  bool create(std::shared_ptr<RendererManager> renderer, std::shared_ptr<RigidBodyManager> rigid_body, std::shared_ptr<GUIManager> gui);
  void destroy();

  daxa::TlasId get_tlas();
  daxa::BufferId get_previous_rigid_body_buffer();
  daxa::BufferId get_rigid_body_buffer();
  daxa::BufferId get_next_rigid_body_buffer();


  // NOTE: queue sync assures double buffering is filled
  void build_AS();
  // post_primitive_upload: optional hook run between the host primitive upload and the
  // BLAS build - the GPU voxel-prims pass writes each voxel body's AABB range into the
  // scratch buffer here (GPU-first; the CPU-filled ranges are only the verify oracle).
  bool build_accel_structs(std::vector<RigidBody> &rigid_bodies, std::vector<Aabb> const &primitives,
                           std::function<void(daxa::BufferId)> const &post_primitive_upload = {});
  void update_TLAS();
  // Zero the incremental upload counters so the next build_accel_structs() re-fills from offset 0
  // (used by scene reset/reload). Without this the counts accumulate and the 2nd reload exceeds the
  // max rigid-body count. Does NOT destroy buffers/AS.
  void reset_for_reload()
  {
    current_rigid_body_count = 0;
    previous_rigid_body_count = 0;
    current_primitive_count = 0;
    primitive_scratch_offset = 0;
    previous_primitive_count = 0;
    proc_blas_buffer_offset = 0;
  }
  bool update_TLAS_resources(daxa::BufferId dispatch_buffer);
  void update_AS_buffers();

private:
  // TODO: temporary
  static constexpr u32 MAX_ACCELERATION_STRUCTURE_COUNT = 1024;
  static constexpr u32 AVERAGE_AS_SIZE = 1024 * 1024; // per-TLAS/scratch fixed size (B4-guarded)
  // F2: the procedural-BLAS pool (proc_blas_buffer + proc_blas_scratch) was reserved at
  // AVERAGE_AS_SIZE * MAX = 1 GiB EACH (2 GiB total) — a blind over-estimate. Measured peak usage
  // (scene_2, 1001 bodies): 751 KB buffer, 2.6 MB scratch. 64 MiB is a ~25x-over-measured budget
  // that comfortably covers voxel-concave shapes too; the offset guards below fail loudly (not GPU
  // corruption) if a pathological scene ever exceeds it, at which point raise this one number.
  static constexpr u32 BLAS_POOL_BUDGET = 64u * 1024u * 1024u;
  
  // Alignment of the acceleration structure build offset
  static constexpr u64 ACCELERATION_STRUCTURE_BUILD_OFFSET_ALIGMENT = 256;
  // Daxa device
  daxa::Device &device;
  // Initialization flag
  bool initialized = false;
  // Task manager reference
  std::shared_ptr<TaskManager> task_manager;
  // Back-references wired in create() — RAW pointers on purpose (review v3): RendererManager owns
  // shared_ptrs to this manager while this manager pointed back with shared_ptrs, so the reference
  // cycle kept EVERY manager's use-count above zero and no destructor ever ran (destroy() has
  // explicit call sites in main(), but the objects themselves leaked). All managers are constructed
  // and torn down strictly within main()'s scope, so non-owning pointers are safe here.
  RendererManager *renderer_manager = nullptr;
  RigidBodyManager *rigid_body_manager = nullptr;
  GUIManager *gui_manager = nullptr;
  // Compute pipeline for updating acceleration structures
  std::shared_ptr<daxa::ComputePipeline> update_pipeline;
  // Alignment of the scratch buffer
  u64 acceleration_structure_scratch_offset_alignment = 0;

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
  u64 proc_blas_scratch_offset = 0;
  // Scratch buffer for the BLAS
  daxa::BufferId proc_blas_scratch_buffer = {};
  // Offset for the BLAS buffer
  u64 proc_blas_buffer_offset = 0;
  // Buffer for the BLAS
  daxa::BufferId proc_blas_buffer = {};
  // Sub-allocated buffer for the BLAS
  std::vector<daxa::BlasId> proc_blas = {};

  // INCREMENTAL AS (phase 2b): per-body-id BLAS + region tracking so a fracture rebuilds only
  // the handful of changed bodies instead of every BLAS. body_blas_[id] is the live BLAS of
  // body id ({} = none); the region vectors record its proc_blas_buffer / primitive_buffer
  // slices for freeing. blas_region_pool_ / prim_region_pool_ are FreeListPools over those
  // buffers (bytes / Aabb-count units); a fracture frees retired+changed bodies' regions and
  // allocs fresh ones, keeping unchanged bodies' BLAS in place.
  std::vector<daxa::BlasId> body_blas_;                 // [MAX_RIGID_BODY_COUNT]
  std::vector<std::pair<u64, u64>> body_blas_region_;   // (byte offset, aligned byte size)
  std::vector<std::pair<u32, u32>> body_prim_region_;   // (Aabb offset, count)
  FreeListPool blas_region_pool_;                       // over proc_blas_buffer (bytes)
  FreeListPool prim_region_pool_;                       // over primitive_buffer (Aabb count)
  bool incremental_ready_ = false;                      // seeded by the first full build

  // Buffer for the LBVH BLAS
  daxa::BlasId lbvh_blas[DOUBLE_BUFFERING] = {};
  
  // Placeholder BLAS to prevent task graph compiler crashes
  daxa::BlasId placeholder_blas = {};

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
  // B4: the TLAS and its scratch buffer are fixed at AVERAGE_AS_SIZE; the queried build sizes were
  // computed then discarded (FIXME). Check them before building so an over-capacity TLAS fails loudly
  // (like the BLAS offset guards) instead of silently overflowing the GPU buffer -> corruption/device-lost.
  bool tlas_within_budget(char const *where)
  {
    if (tlas_build_sizes.acceleration_structure_size > AVERAGE_AS_SIZE ||
        tlas_build_sizes.build_scratch_size > AVERAGE_AS_SIZE)
    {
      std::cerr << "ERROR: TLAS exceeds the fixed " << AVERAGE_AS_SIZE << "-byte budget at " << where
                << " (AS=" << tlas_build_sizes.acceleration_structure_size
                << ", scratch=" << tlas_build_sizes.build_scratch_size << ")" << std::endl;
      return false;
    }
    return true;
  }
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
  void record_update_TLAS_tasks(TaskGraph &instances_TG, TaskGraph &build_TG, std::shared_ptr<daxa::ComputePipeline> update_AS_pipeline);
  void record_update_AS_buffers_tasks(TaskGraph &AS_buffers_TG);
  void update_buffers();

}; // struct AccelerationStructureManager

BB_NAMESPACE_END
