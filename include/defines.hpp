#pragma once

#include <vector>

#include "shared.inl"
#include <daxa/daxa.hpp>
using namespace daxa::types;
#include <daxa/utils/pipeline_manager.hpp>
#include <daxa/utils/task_graph.hpp>

#define BB_NAMESPACE_BEGIN namespace beatbox {
#define BB_NAMESPACE_END }

#if defined(__cplusplus)
#define FORCE_INLINE inline
#else
#define FORCE_INLINE
#endif


BB_NAMESPACE_BEGIN

static constexpr u32 DOUBLE_BUFFERING = 2;

static constexpr f32 TIME_STEP = 0.01667f;
static constexpr f32 GRAVITY = 9.81f;
static constexpr u32 MAX_PRIMITIVE_COUNT = 1024;
static constexpr u32 MAX_RIGID_BODY_COUNT = 1024;
static constexpr u32 MAX_COLLISION_COUNT = MAX_RIGID_BODY_COUNT * (MAX_RIGID_BODY_COUNT - 1) / 2;
static constexpr u32 MAX_LBVH_NODE_COUNT = MAX_RIGID_BODY_COUNT * 2 - 1;
static constexpr u32 MAX_VERTEX_COUNT = MAX_RIGID_BODY_COUNT * 8;
static constexpr u32 MAX_AXIS_COUNT = MAX_RIGID_BODY_COUNT * 6;
static constexpr u32 MAX_LIGHT_COUNT = 1024;
static constexpr u32 MAX_MATERIAL_COUNT = 1024;
static constexpr u32 DEFAULT_ITERATION_COUNT = 5;

enum StageIndex : u32
{
  RAYGEN,
  MISS,
  MISS2,
  CLOSE_HIT,
  INTERSECTION,
    ANY_HIT_LBVH,
    INTERSECTION_LBVH,
  STAGES_COUNT,
};

enum GroupIndex : u32
{
  PRIMARY_RAY,
  HIT_MISS,
  SHADOW_MISS,
  PROCEDURAL_HIT,
  LBVH_HIT,
  GROUPS_COUNT,
};

FORCE_INLINE std::string to_string(StageIndex index)
{
  switch (index)
  {
  case RAYGEN:
    return "ray_generation";
  case MISS:
    return "miss";
  case MISS2:
    return "miss_shadows";
  case CLOSE_HIT:
    return "closest_hit";
  case INTERSECTION:
    return "intersection";
  case ANY_HIT_LBVH:
    return "any_hit_LBVH";
    case INTERSECTION_LBVH:
    return "intersection_LBVH";
  default:
    return "unknown";
  }
}

const auto RT_shader_file_string = "ray_tracing.slang";
const auto RT_main_pipeline_name = "Main Ray Tracing Pipeline";

const auto RB_sim_shader_file_string = "RB_sim.slang";

// rigid body dispatcher
const auto entry_rigid_body_dispatcher = "entry_rigid_body_dispatcher";
const auto rigid_body_dispatcher_pipeline_name = "Rigid Body Dispatcher";

// generate morton codes
const auto entry_generate_morton_codes = "entry_generate_morton_codes";
const auto generate_morton_codes_pipeline_name = "Generate Morton Codes";

// radix sort histogram
const auto entry_radix_sort_histogram = "entry_radix_sort_histogram";
const auto radix_sort_histogram_pipeline_name = "Radix Sort Histogram";

// single radix sort
const auto entry_rigid_body_single_radix_sort = "entry_single_radix_sort";
const auto rigid_body_single_radix_sort_pipeline_name = "Single Radix Sort";

// generate hierarchy for linear bounding volume hierarchy
const auto entry_generate_hierarchy_linear_bvh = "entry_generate_hierarchy_linear_bvh";
const auto generate_hierarchy_linear_bvh_pipeline_name = "Generate Hierarchy Linear BVH";

// build bounding boxes for linear bounding volume hierarchy
const auto entry_build_bounding_boxes_linear_bvh = "entry_build_bounding_boxes_linear_bvh";
const auto build_bounding_boxes_linear_bvh_pipeline_name = "Build Bounding Boxes Linear BVH";

// reorder rigid bodies
const auto entry_rigid_body_reordering = "entry_rigid_body_reordering";
const auto rigid_body_reordering_pipeline_name = "Rigid Body Reordering";

// reset body links
const auto entry_reset_body_links = "entry_reset_body_links";
const auto reset_body_links_pipeline_name = "Reset Body Links";

// broad phase
const auto entry_broad_phase_sim = "entry_broad_phase";
const auto broad_phase_sim_pipeline_name = "broad phase Simulation";

// narrow phase dispatcher
const auto entry_narrow_phase_dispatcher = "entry_narrow_phase_dispatcher";
const auto narrow_phase_dispatcher_pipeline_name = "Narrow Phase Dispatcher";

// narrow phase
const auto entry_narrow_phase_sim = "entry_narrow_phase";
const auto narrow_phase_sim_pipeline_name = "narrow phase Simulation";

// collision solver dispatcher
const auto entry_collision_solver_dispatcher = "entry_collision_solver_dispatcher";
const auto collision_solver_dispatcher_pipeline_name = "Collision Solver Dispatcher";

// island dispatcher
const auto entry_island_dispatcher = "entry_island_dispatcher";
const auto island_dispatcher_pipeline_name = "Collision Solver Dispatcher";

// island counter
const auto entry_island_counter = "entry_island_counter";
const auto island_counter_pipeline_name = "Island Counter";

// island builder
const auto entry_island_builder = "entry_island_builder";
const auto island_builder_pipeline_name = "Island Builder";

// island prefix sum
const auto entry_island_prefix_sum = "entry_island_prefix_sum";
const auto island_prefix_sum_pipeline_name = "Island Prefix Sum";

// body link to island
const auto entry_body_link_to_island = "entry_body_link_to_island";
const auto body_link_to_island_pipeline_name = "Body Link to Island";

// sort body links in island
const auto entry_sort_body_links_in_island = "entry_sort_body_links_in_island";
const auto sort_body_links_in_island_pipeline_name = "Sort Body Links in Island";

// manifold island builder
const auto entry_manifold_island_builder = "entry_manifold_island_builder";
const auto manifold_island_builder_pipeline_name = "Manifold Island Builder";

// manifold island gather
const auto entry_contact_island_gather = "entry_contact_island_gather";
const auto contact_island_gather_pipeline_name = "Contact Island Gather";

// contact island dispatcher
const auto entry_contact_island_dispatcher = "entry_contact_island_dispatcher";
const auto contact_island_dispatcher_pipeline_name = "Contact Island Dispatcher";

// manifold island prefix sum
const auto entry_manifold_island_prefix_sum = "entry_manifold_island_prefix_sum";
const auto manifold_island_prefix_sum_pipeline_name = "Manifold Island Prefix Sum";

// manifold link to island
const auto entry_manifold_link_to_island = "entry_manifold_link_to_island";
const auto manifold_link_to_island_pipeline_name = "Manifold Link to Island";

// sort manifold links in island
const auto entry_sort_manifold_links_in_island = "entry_sort_manifold_links_in_island";
const auto sort_manifold_links_in_island_pipeline_name = "Sort Body Links in Island";

// sim
const auto entry_rigid_body_sim = "entry_rigid_body_sim";
const auto RB_sim_pipeline_name = "Rigid Body Simulation";

// collision pre solver
const auto entry_collision_pre_solver = "entry_collision_pre_solver";
const auto collision_pre_solver_pipeline_name = "Collision Pre-Solver";

// collision solver
const auto entry_collision_solver = "entry_collision_solver";
const auto collision_solver_pipeline_name = "Collision Solver";

// integrate positions
const auto entry_integrate_positions = "entry_integrate_positions_rigid_bodies";
const auto integrate_positions_pipeline_name = "Integrate Positions Rigid Bodies";

// collision solver
const auto entry_collision_relaxation_solver = "entry_collision_solver_relaxation";
const auto collision_solver_relaxation_pipeline_name = "Collision Solver Relaxation";

// create contact points
const auto entry_create_contact_points = "entry_create_contact_points";
const auto create_contact_points_pipeline_name = "Create Contact Points";

// update
const auto entry_update_rigid_bodies = "entry_update_rigid_bodies";
const auto update_rigid_bodies_pipeline_name = "Update Rigid Bodies";




// update instances
const auto AS_shader_file_string = "AS_mngr.slang";
const auto entry_update_acceleration_structures = "entry_update_acceleration_structures";
const auto AS_update_pipeline_name = "Update Acceleration Structures";




// GUI
const auto GUI_shader_file_string = "GUI.slang";
// GUI points
const auto GUI_vertex = "entry_vertex";
const auto GUI_fragment = "entry_fragment";
const auto GUI_pipeline_name = "GUI Pipeline";
// GUI lines
const auto GUI_line_vertex = "entry_line_vertex";
const auto GUI_line_fragment = "entry_line_fragment";
const auto GUI_line_pipeline_name = "GUI Line Pipeline";
// GUI axes
const auto GUI_axes_vertex = "entry_axes_vertex";
const auto GUI_axes_fragment = "entry_axes_fragment";
const auto GUI_axes_pipeline_name = "GUI Axes Pipeline";

struct MainRayTracingPipeline
{
  daxa::ShaderCompileInfo rt_gen_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RT_shader_file_string},
      .compile_options = {
          .entry_point = to_string(RAYGEN),
      },
  };

  daxa::ShaderCompileInfo rt_miss_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RT_shader_file_string},
      .compile_options = {
          .entry_point = to_string(MISS),
      },
  };

  daxa::ShaderCompileInfo rt_miss_shadow_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RT_shader_file_string},
      .compile_options = {
          .entry_point = to_string(MISS2),
      },
  };

  daxa::ShaderCompileInfo rt_closest_hit_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RT_shader_file_string},
      .compile_options = {
          .entry_point = to_string(CLOSE_HIT),
      },
  };

  daxa::ShaderCompileInfo rt_intersection_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RT_shader_file_string},
      .compile_options = {
          .entry_point = to_string(INTERSECTION),
      },
  };

  daxa::ShaderCompileInfo rt_any_hit_lbvh_shader_file_string = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RT_shader_file_string},
      .compile_options = {
          .entry_point = to_string(ANY_HIT_LBVH),
      },
  };

  daxa::ShaderCompileInfo rt_intersection_lbvh_shader_file_string = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RT_shader_file_string},
      .compile_options = {
          .entry_point = to_string(INTERSECTION_LBVH),
      },
  };

  std::vector<daxa::RayTracingShaderCompileInfo> stages;

  std::vector<daxa::RayTracingShaderGroupInfo> groups;

  daxa::RayTracingPipelineCompileInfo info;

  explicit MainRayTracingPipeline() {

    stages.resize(STAGES_COUNT);
    groups.resize(GROUPS_COUNT);

    stages[RAYGEN] = daxa::RayTracingShaderCompileInfo{
        .shader_info = rt_gen_shader,
        .type = daxa::RayTracingShaderType::RAYGEN,
    };

    stages[MISS] = daxa::RayTracingShaderCompileInfo{
        .shader_info = rt_miss_shader,
        .type = daxa::RayTracingShaderType::MISS,
    };

    stages[MISS2] = daxa::RayTracingShaderCompileInfo{
        .shader_info = rt_miss_shadow_shader,
        .type = daxa::RayTracingShaderType::MISS,
    };

    stages[CLOSE_HIT] = daxa::RayTracingShaderCompileInfo{
        .shader_info = rt_closest_hit_shader,
        .type = daxa::RayTracingShaderType::CLOSEST_HIT,
    };

    stages[INTERSECTION] = daxa::RayTracingShaderCompileInfo{
        .shader_info = rt_intersection_shader,
        .type = daxa::RayTracingShaderType::INTERSECTION,
    };

    stages[ANY_HIT_LBVH] = daxa::RayTracingShaderCompileInfo{
        .shader_info = rt_any_hit_lbvh_shader_file_string,
        .type = daxa::RayTracingShaderType::ANY_HIT,
    };

    stages[INTERSECTION_LBVH] = daxa::RayTracingShaderCompileInfo{
        .shader_info = rt_intersection_lbvh_shader_file_string,
        .type = daxa::RayTracingShaderType::INTERSECTION,
    };

    groups[PRIMARY_RAY] = daxa::RayTracingShaderGroupInfo{
        .type = daxa::ExtendedShaderGroupType::RAYGEN,
        .general_shader_index = StageIndex::RAYGEN,
    };

    groups[HIT_MISS] = daxa::RayTracingShaderGroupInfo{
        .type = daxa::ExtendedShaderGroupType::MISS,
        .general_shader_index = StageIndex::MISS,
    };

    groups[SHADOW_MISS] = daxa::RayTracingShaderGroupInfo{
        .type = daxa::ExtendedShaderGroupType::MISS,
        .general_shader_index = StageIndex::MISS2,
    };

    groups[PROCEDURAL_HIT] = daxa::RayTracingShaderGroupInfo{
        .type = daxa::ExtendedShaderGroupType::PROCEDURAL_HIT_GROUP,
        .closest_hit_shader_index = StageIndex::CLOSE_HIT,
        .intersection_shader_index = StageIndex::INTERSECTION,
    };

    groups[LBVH_HIT] = daxa::RayTracingShaderGroupInfo{
        .type = daxa::ExtendedShaderGroupType::PROCEDURAL_HIT_GROUP,
        // .closest_hit_shader_index = StageIndex::CLOSE_HIT_LBVH,
        .any_hit_shader_index = StageIndex::ANY_HIT_LBVH,
        .intersection_shader_index = StageIndex::INTERSECTION_LBVH,
    };

    info = daxa::RayTracingPipelineCompileInfo{
        .stages = stages,
        .groups = groups,
        .max_ray_recursion_depth = 2,
        .push_constant_size = sizeof(RTPushConstants),
        .name = RT_main_pipeline_name,
    };
  }
};

struct RigidBodyDispatcherInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_rigid_body_dispatcher,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(RigidBodyDispatcherPushConstants),
      .name = rigid_body_dispatcher_pipeline_name,
  };
};

struct GenerateMortonCodesInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_generate_morton_codes,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(RigidBodyGenerateMortonCodePushConstants),
      .name = generate_morton_codes_pipeline_name,
  };
};

struct RigidBodyRadixSortHistogramInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_radix_sort_histogram,
          .defines = {
              {"BB_RADIX_SORT_HISTOGRAM", "1"},
          },
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(RigidBodyRadixSortHistogramPushConstants),
      .name = radix_sort_histogram_pipeline_name,
  };
};

struct RigidBodySingleRadixSortInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_rigid_body_single_radix_sort,
          .defines = {
              {"BB_RADIX_SORT", "1"},
          },
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(RigidBodySingleRadixSortPushConstants),
      .name = rigid_body_single_radix_sort_pipeline_name,
  };
};

struct RigidBodyGenerateHierarchyLinearBVHInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_generate_hierarchy_linear_bvh,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(RigidBodyGenerateHierarchyLinearBVHPushConstants),
      .name = generate_hierarchy_linear_bvh_pipeline_name,
  };
};

struct RigidBodyBuildBoundingBoxesLinearBVHInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_build_bounding_boxes_linear_bvh,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(RigidBodyBuildBoundingBoxesLBVHPushConstants),
      .name = build_bounding_boxes_linear_bvh_pipeline_name,
  };
};

struct RigidBodyReorderingInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_rigid_body_reordering,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(RigidBodyReorderingPushConstants),
      .name = rigid_body_reordering_pipeline_name,
  };
};

struct ResetBodyLinksInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_reset_body_links,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(ResetBodyLinkPushConstants),
      .name = reset_body_links_pipeline_name,
  };
};

struct BroadPhaseInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_broad_phase_sim
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(BroadPhasePushConstants),
      .name = broad_phase_sim_pipeline_name,
  };
};

struct NarrowPhaseDispatcherInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_narrow_phase_dispatcher
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(NarrowPhaseDispatcherPushConstants),
      .name = narrow_phase_dispatcher_pipeline_name,
  };
};

struct NarrowPhaseInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_narrow_phase_sim
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(NarrowPhasePushConstants),
      .name = narrow_phase_sim_pipeline_name,
  };
};

struct CollisionSolverDispatcherInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_collision_solver_dispatcher
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(CollisionSolverDispatcherPushConstants),
      .name = collision_solver_dispatcher_pipeline_name,
  };
};

// ISLANDS
struct IslandDispatcherInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_island_dispatcher,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(IslandDispatcherPushConstants),
      .name = island_dispatcher_pipeline_name,
  };
};

struct IslandCounterInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_island_counter,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(IslandCounterPushConstants),
      .name = island_counter_pipeline_name,
  };
};

struct IslandBuilderInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_island_builder,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(IslandBuilderPushConstants),
      .name = island_builder_pipeline_name,
  };
};

struct IslandPrefixSumInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_island_prefix_sum,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(IslandPrefixSumPushConstants),
      .name = island_prefix_sum_pipeline_name,
  };
};

struct BodyLink2IslandInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_body_link_to_island,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(IslandBuilderBodyLink2IslandPushConstants),
      .name = body_link_to_island_pipeline_name,
  };
};

struct SortBodyLinksInIslandInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_sort_body_links_in_island,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(IslandBuilderSortBodyLinkInIslandPushConstants),
      .name = sort_body_links_in_island_pipeline_name,
  };
};

struct ManifoldIslandBuilderInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_manifold_island_builder,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(ManifoldIslandBuilderPushConstants),
      .name = manifold_island_builder_pipeline_name,
  };
};

struct ContactIslandGatherInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_contact_island_gather,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(ContactIslandGatherPushConstants),
      .name = contact_island_gather_pipeline_name,
  };
};

struct ContactIslandDispatcherInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_contact_island_dispatcher,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(ContactIslandDispatcherPushConstants),
      .name = contact_island_dispatcher_pipeline_name,
  };
};

struct ManifoldIslandPrefixSumInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_manifold_island_prefix_sum,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(ManifoldIslandPrefixSumPushConstants),
      .name = manifold_island_prefix_sum_pipeline_name,
  };
};

struct ManifoldLink2IslandInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_manifold_link_to_island,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(IslandBuilderManifoldLink2IslandPushConstants),
      .name = manifold_link_to_island_pipeline_name,
  };
};

struct SortManifoldLinksInIslandInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_sort_manifold_links_in_island,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(IslandBuilderSortManifoldLinkInIslandPushConstants),
      .name = sort_manifold_links_in_island_pipeline_name,
  };
};
// ISLANDS

struct CollisionPreSolverInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_collision_pre_solver
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(CollisionPreSolverPushConstants),
      .name = collision_pre_solver_pipeline_name,
  };
};

struct CollisionSolverInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_collision_solver
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(CollisionSolverPushConstants),
      .name = collision_solver_pipeline_name,
  };
};

struct IntegratePositionsInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_integrate_positions
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(RigidBodyIntegratePositionsPushConstants),
      .name = integrate_positions_pipeline_name,
  };
};

struct CollisionSolverRelaxationInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_collision_relaxation_solver
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(CollisionSolverRelaxationPushConstants),
      .name = collision_solver_relaxation_pipeline_name,
  };
};

struct RigidBodySim
{
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_rigid_body_sim
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(RigidBodySimPushConstants),
      .name = RB_sim_pipeline_name,
  };
};


struct UpdateAccelerationStructures
{
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{AS_shader_file_string},
      .compile_options = {
          .entry_point = entry_update_acceleration_structures,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(UpdateInstancesPushConstants),
      .name = AS_update_pipeline_name,
  };
};

struct CreateContactPoints
{
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_create_contact_points,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(CreatePointsPushConstants),
      .name = create_contact_points_pipeline_name,
  };
};

struct UIPipeline
{
    daxa::ShaderCompileInfo vertex_shader = daxa::ShaderCompileInfo{
        .source = daxa::ShaderFile{GUI_shader_file_string},
        .compile_options = {
            .entry_point = GUI_vertex,
        },
    };
    
    daxa::ShaderCompileInfo fragment_shader = daxa::ShaderCompileInfo{
        .source = daxa::ShaderFile{GUI_shader_file_string},
        .compile_options = {
            .entry_point = GUI_fragment,
        },
    };
    
    daxa::RasterPipelineCompileInfo info = {
        .vertex_shader_info = vertex_shader,
        .fragment_shader_info = fragment_shader,
        .color_attachments = {
            daxa::RenderAttachment{
                .format = daxa::Format::R8G8B8A8_UNORM,
            },
        },
        .raster = {.primitive_topology = daxa::PrimitiveTopology::POINT_LIST},
        .push_constant_size = sizeof(GUIPushConstants),
        .name = GUI_pipeline_name,
    };
};

struct UILinePipeline
{
    daxa::ShaderCompileInfo vertex_shader = daxa::ShaderCompileInfo{
        .source = daxa::ShaderFile{GUI_shader_file_string},
        .compile_options = {
            .entry_point = GUI_line_vertex,
        },
    };
    
    daxa::ShaderCompileInfo fragment_shader = daxa::ShaderCompileInfo{
        .source = daxa::ShaderFile{GUI_shader_file_string},
        .compile_options = {
            .entry_point = GUI_line_fragment,
        },
    };
    
    daxa::RasterPipelineCompileInfo info = {
        .vertex_shader_info = vertex_shader,
        .fragment_shader_info = fragment_shader,
        .color_attachments = {
            daxa::RenderAttachment{
                .format = daxa::Format::R8G8B8A8_UNORM,
            },
        },
        .raster = {.primitive_topology = daxa::PrimitiveTopology::LINE_LIST, .line_width = 5.0f},
        .push_constant_size = sizeof(GUILinePushConstants),
        .name = GUI_line_pipeline_name,
    };
};

struct UIAxesPipeline {
    daxa::ShaderCompileInfo vertex_shader = daxa::ShaderCompileInfo{
        .source = daxa::ShaderFile{GUI_shader_file_string},
        .compile_options = {
            .entry_point = GUI_axes_vertex,
        },
    };
    
    daxa::ShaderCompileInfo fragment_shader = daxa::ShaderCompileInfo{
        .source = daxa::ShaderFile{GUI_shader_file_string},
        .compile_options = {
            .entry_point = GUI_axes_fragment,
        },
    };
    
    daxa::RasterPipelineCompileInfo info = {
        .vertex_shader_info = vertex_shader,
        .fragment_shader_info = fragment_shader,
        .color_attachments = {
            daxa::RenderAttachment{
                .format = daxa::Format::R8G8B8A8_UNORM,
            },
        },
        .raster = {.primitive_topology = daxa::PrimitiveTopology::LINE_LIST, .line_width = 5.0f},
        .push_constant_size = sizeof(GUIAxesPushConstants),
        .name = "GUI Axes Pipeline",
    };
};

struct UpdateRigidBodies {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_update_rigid_bodies
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(RigidBodyUpdatePushConstants),
      .name = update_rigid_bodies_pipeline_name,
  };
};

BB_NAMESPACE_END