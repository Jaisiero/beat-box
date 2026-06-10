#pragma once

#include <array>
#include <cstring>
#include <functional>
#include <memory>
#include <optional>
#include <type_traits>
#include <vector>

#include "shared.inl"
#include <daxa/daxa.hpp>
using namespace daxa::types;
#include <daxa/utils/pipeline_manager.hpp>
#include <daxa/utils/task_graph.hpp>

#define BB_DAXA_TASK_ALIAS(NAME) namespace NAME { using Task = daxa::TInlineTask<Info>; }
BB_DAXA_TASK_ALIAS(RayTracingTaskHead)
BB_DAXA_TASK_ALIAS(GUITaskHead)
BB_DAXA_TASK_ALIAS(GUILineTaskHead)
BB_DAXA_TASK_ALIAS(GUIAxesTaskHead)
BB_DAXA_TASK_ALIAS(RigidBodyDispatcherTaskHead)
BB_DAXA_TASK_ALIAS(RigidBodyGenerateMortonCodeTaskHead)
BB_DAXA_TASK_ALIAS(RigidBodyRadixSortHistogramTaskHead)
BB_DAXA_TASK_ALIAS(RigidBodySingleRadixSortTaskHead)
BB_DAXA_TASK_ALIAS(RigidBodyGenerateHierarchyLinearBVHTaskHead)
BB_DAXA_TASK_ALIAS(RigidBodyBuildBoundingBoxesLinearBVHTaskHead)
BB_DAXA_TASK_ALIAS(RigidBodyConvertBoundingBoxesLinearBVHTaskHead)
BB_DAXA_TASK_ALIAS(RigidBodyReorderingTaskHead)
BB_DAXA_TASK_ALIAS(ResetBodyLinkTaskHead)
BB_DAXA_TASK_ALIAS(BroadPhaseTaskHead)
BB_DAXA_TASK_ALIAS(NarrowPhaseDispatcherTaskHead)
BB_DAXA_TASK_ALIAS(NarrowPhaseTaskHead)
BB_DAXA_TASK_ALIAS(CollisionSolverDispatcherTaskHead)
BB_DAXA_TASK_ALIAS(RigidBodySimTaskHead)
BB_DAXA_TASK_ALIAS(IslandCounterTaskHead)
BB_DAXA_TASK_ALIAS(IslandDispatcherTaskHead)
BB_DAXA_TASK_ALIAS(IslandBuilderTaskHead)
BB_DAXA_TASK_ALIAS(IslandPrefixSumTaskHead)
BB_DAXA_TASK_ALIAS(IslandBuilderBodyLink2IslandTaskHead)
BB_DAXA_TASK_ALIAS(IslandBuilderSortBodyLinkInIslandTaskHead)
BB_DAXA_TASK_ALIAS(ManifoldIslandBuilderTaskHead)
BB_DAXA_TASK_ALIAS(ContactIslandGatherTaskHead)
BB_DAXA_TASK_ALIAS(ContactIslandDispatcherTaskHead)
BB_DAXA_TASK_ALIAS(ManifoldIslandPrefixSumTaskHead)
BB_DAXA_TASK_ALIAS(IslandBuilderManifoldLink2IslandTaskHead)
BB_DAXA_TASK_ALIAS(IslandBuilderSortManifoldLinkInIslandTaskHead)
BB_DAXA_TASK_ALIAS(CollisionPreSolverTaskHead)
BB_DAXA_TASK_ALIAS(CollisionSolverTaskHead)
BB_DAXA_TASK_ALIAS(IntegratePositionsTaskHead)
BB_DAXA_TASK_ALIAS(CollisionSolverRelaxationTaskHead)
BB_DAXA_TASK_ALIAS(RigidBodyUpdateTaskHead)
BB_DAXA_TASK_ALIAS(UpdateInstancesTaskHead)
BB_DAXA_TASK_ALIAS(CreatePointsTaskHead)
BB_DAXA_TASK_ALIAS(GraphColorTaskHead)
BB_DAXA_TASK_ALIAS(GraphColorSolveTaskHead)
#undef BB_DAXA_TASK_ALIAS

namespace daxa
{
struct ShaderCompileOptions
{
  std::optional<std::string> entry_point = {};
  std::optional<ShaderLanguage> language = {};
  std::vector<ShaderDefine> defines = {};
  std::optional<bool> enable_debug_info = {};
  std::optional<ShaderCreateFlags> create_flags = {};
  std::optional<u32> required_subgroup_size = {};
};

struct ShaderCompileInfo
{
  ShaderSource source = Monostate{};
  ShaderCompileOptions compile_options = {};

  [[nodiscard]] auto to_info2() const -> ShaderCompileInfo2
  {
    return ShaderCompileInfo2{
        .source = source,
        .entry_point = compile_options.entry_point,
        .language = compile_options.language,
        .defines = compile_options.defines,
        .enable_debug_info = compile_options.enable_debug_info,
        .create_flags = compile_options.create_flags,
        .required_subgroup_size = compile_options.required_subgroup_size,
    };
  }

  [[nodiscard]] operator ShaderCompileInfo2() const
  {
    return to_info2();
  }
};

inline auto to_info2(std::vector<ShaderCompileInfo> const &infos) -> std::vector<ShaderCompileInfo2>
{
  auto ret = std::vector<ShaderCompileInfo2>{};
  ret.reserve(infos.size());
  for (auto const &info : infos)
  {
    ret.push_back(info.to_info2());
  }
  return ret;
}

inline auto to_info2(Optional<ShaderCompileInfo> const &info) -> Optional<ShaderCompileInfo2>
{
  if (info.has_value())
  {
    return Optional<ShaderCompileInfo2>{info.value().to_info2()};
  }
  return {};
}

struct RayTracingPipelineCompileInfo
{
  std::vector<ShaderCompileInfo> ray_gen_infos = {};
  std::vector<ShaderCompileInfo> intersection_infos = {};
  std::vector<ShaderCompileInfo> any_hit_infos = {};
  std::vector<ShaderCompileInfo> callable_infos = {};
  std::vector<ShaderCompileInfo> closest_hit_infos = {};
  std::vector<ShaderCompileInfo> miss_hit_infos = {};
  std::vector<RayTracingShaderGroupInfo> shader_groups_infos = {};
  u32 max_ray_recursion_depth = {};
  u32 push_constant_size = DAXA_MAX_PUSH_CONSTANT_BYTE_SIZE;
  std::string name = {};

  [[nodiscard]] operator RayTracingPipelineCompileInfo2() const
  {
    return RayTracingPipelineCompileInfo2{
        .ray_gen_infos = daxa::to_info2(ray_gen_infos),
        .intersection_infos = daxa::to_info2(intersection_infos),
        .any_hit_infos = daxa::to_info2(any_hit_infos),
        .callable_infos = daxa::to_info2(callable_infos),
        .closest_hit_infos = daxa::to_info2(closest_hit_infos),
        .miss_hit_infos = daxa::to_info2(miss_hit_infos),
        .shader_groups_infos = shader_groups_infos,
        .max_ray_recursion_depth = max_ray_recursion_depth,
        .push_constant_size = push_constant_size,
        .name = name,
    };
  }
};

struct ComputePipelineCompileInfo
{
  ShaderCompileInfo shader_info = {};
  u32 push_constant_size = DAXA_MAX_PUSH_CONSTANT_BYTE_SIZE;
  std::string name = {};

  [[nodiscard]] operator ComputePipelineCompileInfo2() const
  {
    auto info = shader_info.to_info2();
    return ComputePipelineCompileInfo2{
        .source = info.source,
        .entry_point = info.entry_point,
        .language = info.language,
        .defines = info.defines,
        .enable_debug_info = info.enable_debug_info,
        .create_flags = info.create_flags,
        .required_subgroup_size = info.required_subgroup_size,
        .push_constant_size = push_constant_size,
        .name = name,
    };
  }
};

struct RasterPipelineCompileInfo
{
  Optional<ShaderCompileInfo> mesh_shader_info = {};
  Optional<ShaderCompileInfo> vertex_shader_info = {};
  Optional<ShaderCompileInfo> tesselation_control_shader_info = {};
  Optional<ShaderCompileInfo> tesselation_evaluation_shader_info = {};
  Optional<ShaderCompileInfo> fragment_shader_info = {};
  Optional<ShaderCompileInfo> task_shader_info = {};
  std::vector<RenderAttachment> color_attachments = {};
  Optional<DepthTestInfo> depth_test = {};
  RasterizerInfo raster = {};
  TesselationInfo tesselation = {};
  u32 push_constant_size = DAXA_MAX_PUSH_CONSTANT_BYTE_SIZE;
  std::string name = {};

  [[nodiscard]] operator RasterPipelineCompileInfo2() const
  {
    return RasterPipelineCompileInfo2{
        .mesh_shader_info = daxa::to_info2(mesh_shader_info),
        .vertex_shader_info = daxa::to_info2(vertex_shader_info),
        .tesselation_control_shader_info = daxa::to_info2(tesselation_control_shader_info),
        .tesselation_evaluation_shader_info = daxa::to_info2(tesselation_evaluation_shader_info),
        .fragment_shader_info = daxa::to_info2(fragment_shader_info),
        .task_shader_info = daxa::to_info2(task_shader_info),
        .color_attachments = color_attachments,
        .depth_test = depth_test,
        .raster = raster,
        .tesselation = tesselation,
        .push_constant_size = push_constant_size,
        .name = name,
    };
  }
};

using TaskBuffer = ExternalTaskBuffer;
using TaskImage = ExternalTaskImage;
using TaskBlas = ExternalTaskBlas;
using TaskTlas = ExternalTaskTlas;

// Task attachment binding — NAME-BASED (order-independent). `attachment_view(AT.<name>, buf)`
// captures the attachment's slot index from `AT.<name>.value` (a Task*AttachmentIndex), and
// `make_attachment_views` places each view at that slot before the memcpy into the head's
// `Views` struct. So a task's binding-array order does NOT need to match the head's `DAXA_TH_*`
// declaration order — each view is routed by name.
// (Until 2026-06 this was POSITIONAL and the AT tag was ignored, which silently mis-wired the
//  solver's collision_map/rigid_bodies and let cubes fall through the floor. Now fixed by
//  honoring the tag's slot index.)
struct TaggedTaskView
{
  u32 slot = {};
  TaskViewVariant view = {};
};

template <typename Views, std::size_t N>
inline auto make_attachment_views(std::array<TaggedTaskView, N> const &tagged) -> Views
{
  std::array<TaskViewVariant, N> values = {};
  for (auto const &t : tagged)
    values[t.slot] = t.view; // route each view to its head slot (by name), not by position

  struct ViewStorage
  {
    u32 dummy = {};
    std::array<TaskViewVariant, N> values = {};
  };

  auto storage = ViewStorage{.values = values};
  auto ret = Views{};
  static_assert(sizeof(Views) == sizeof(ViewStorage));
  std::memcpy(&ret, &storage, sizeof(Views));
  return ret;
}

// First arg is the head's `AT.<name>` (a Task*AttachmentIndex carrying `.value` = slot).
inline auto attachment_view(auto idx, ExternalTaskBuffer const &buffer) -> TaggedTaskView
{
  return {idx.value, buffer.view()};
}

inline auto attachment_view(auto idx, ExternalTaskImage const &image) -> TaggedTaskView
{
  return {idx.value, image.view()};
}

inline auto attachment_view(auto idx, ExternalTaskBlas const &blas) -> TaggedTaskView
{
  return {idx.value, blas.view()};
}

inline auto attachment_view(auto idx, ExternalTaskTlas const &tlas) -> TaggedTaskView
{
  return {idx.value, tlas.view()};
}

inline auto inl_attachment(TaskAccess access, ExternalTaskBuffer const &buffer) -> TaskAttachmentInfo
{
  return TaskBufferAttachmentInfo{
      .name = buffer.info().name.data(),
      .task_access = access,
      .id = buffer.id(),
      .view = buffer.view(),
      .translated_view = buffer.view(),
  };
}

inline auto inl_attachment(TaskAccess access, ExternalTaskImage const &image) -> TaskAttachmentInfo
{
  return TaskImageAttachmentInfo{
      .name = image.info().name.data(),
      .task_access = access,
      .id = image.id(),
      .view = image.view(),
      .translated_view = image.view(),
  };
}

inline auto inl_attachment(TaskAccess access, ExternalTaskBlas const &blas) -> TaskAttachmentInfo
{
  return TaskBlasAttachmentInfo{
      .name = blas.info().name.data(),
      .task_access = access,
      .id = blas.id(),
      .view = blas.view(),
      .translated_view = blas.view(),
  };
}

inline auto inl_attachment(TaskAccess access, ExternalTaskTlas const &tlas) -> TaskAttachmentInfo
{
  return TaskTlasAttachmentInfo{
      .name = tlas.info().name.data(),
      .task_access = access,
      .id = tlas.id(),
      .view = tlas.view(),
      .translated_view = tlas.view(),
  };
}

struct InlineTaskInfo
{
  std::vector<TaskAttachmentInfo> attachments = {};
  std::function<void(TaskInterface const &)> task = {};
  std::string_view name = {};

  [[nodiscard]] auto to_inline_task() const -> InlineTask;
};

inline auto retain_task_callback(std::function<void(TaskInterface const &)> callback) -> std::function<void(TaskInterface const &)> *
{
  static auto callbacks = std::vector<std::unique_ptr<std::function<void(TaskInterface const &)>>>{};
  callbacks.push_back(std::make_unique<std::function<void(TaskInterface const &)>>(std::move(callback)));
  return callbacks.back().get();
}

inline auto InlineTaskInfo::to_inline_task() const -> InlineTask
{
  auto ret = InlineTask{name};
  for (auto const &attachment : attachments)
  {
    ret.uses(attachment);
  }
  auto *callback = retain_task_callback(task);
  ret.executes([callback](TaskInterface ti)
  {
    (*callback)(ti);
  });
  return ret;
}
} // namespace daxa

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
const auto coloring_shader_file_string = "coloring.slang";

// graph coloring (parallel contact-solver coloring)
const auto entry_graph_color_dispatcher = "entry_graph_color_dispatcher";
const auto graph_color_dispatcher_pipeline_name = "Graph Color Dispatcher";
const auto entry_graph_color_reset = "entry_graph_color_reset";
const auto graph_color_reset_pipeline_name = "Graph Color Reset";
const auto entry_graph_color_owner_reset = "entry_graph_color_owner_reset";
const auto graph_color_owner_reset_pipeline_name = "Graph Color Owner Reset";
const auto entry_graph_color_assign_p1 = "entry_graph_color_assign_p1";
const auto graph_color_assign_p1_pipeline_name = "Graph Color Assign P1";
const auto entry_graph_color_assign_p2 = "entry_graph_color_assign_p2";
const auto graph_color_assign_p2_pipeline_name = "Graph Color Assign P2";
const auto entry_graph_color_validate = "entry_graph_color_validate";
const auto graph_color_validate_pipeline_name = "Graph Color Validate";
// per-color solver passes (entries live in RB_sim.slang)
const auto entry_collision_pre_solver_color = "entry_collision_pre_solver_color";
const auto graph_color_pre_solver_pipeline_name = "Graph Color Pre Solver";
const auto entry_collision_solver_color = "entry_collision_solver_color";
const auto graph_color_solver_pipeline_name = "Graph Color Solver";
const auto entry_collision_solver_relax_color = "entry_collision_solver_relax_color";
const auto graph_color_relax_pipeline_name = "Graph Color Solver Relax";
// overflow bucket (serial; solves manifolds the per-color dispatches skip)
const auto entry_collision_pre_solver_overflow = "entry_collision_pre_solver_overflow";
const auto graph_color_pre_solver_overflow_pipeline_name = "Graph Color Pre Solver Overflow";
const auto entry_collision_solver_overflow = "entry_collision_solver_overflow";
const auto graph_color_solver_overflow_pipeline_name = "Graph Color Solver Overflow";
const auto entry_collision_solver_relax_overflow = "entry_collision_solver_relax_overflow";
const auto graph_color_relax_overflow_pipeline_name = "Graph Color Solver Relax Overflow";

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

// convert integer-mapped bounding boxes to float AABBs for linear bounding volume hierarchy
const auto entry_convert_bounding_boxes_linear_bvh = "entry_convert_bounding_boxes_linear_bvh";
const auto convert_bounding_boxes_linear_bvh_pipeline_name = "Convert Bounding Boxes Linear BVH";

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

  std::vector<daxa::ShaderCompileInfo> ray_gen_infos;
  std::vector<daxa::ShaderCompileInfo> intersection_infos;
  std::vector<daxa::ShaderCompileInfo> any_hit_infos;
  std::vector<daxa::ShaderCompileInfo> closest_hit_infos;
  std::vector<daxa::ShaderCompileInfo> miss_hit_infos;
  std::vector<daxa::RayTracingShaderGroupInfo> groups;

  daxa::RayTracingPipelineCompileInfo info;

  explicit MainRayTracingPipeline() {

    ray_gen_infos = {rt_gen_shader};
    intersection_infos = {rt_intersection_shader, rt_intersection_lbvh_shader_file_string};
    any_hit_infos = {rt_any_hit_lbvh_shader_file_string};
    closest_hit_infos = {rt_closest_hit_shader};
    miss_hit_infos = {rt_miss_shader, rt_miss_shadow_shader};

    static constexpr u32 RAYGEN_STAGE = 0;
    static constexpr u32 INTERSECTION_STAGE = 1;
    static constexpr u32 INTERSECTION_LBVH_STAGE = 2;
    static constexpr u32 ANY_HIT_LBVH_STAGE = 3;
    static constexpr u32 CLOSE_HIT_STAGE = 4;
    static constexpr u32 MISS_STAGE = 5;
    static constexpr u32 MISS2_STAGE = 6;

    groups.resize(GROUPS_COUNT);

    groups[PRIMARY_RAY] = daxa::RayTracingShaderGroupInfo{
        .type = daxa::ShaderGroup::GENERAL,
        .general_shader_index = RAYGEN_STAGE,
    };

    groups[HIT_MISS] = daxa::RayTracingShaderGroupInfo{
        .type = daxa::ShaderGroup::GENERAL,
        .general_shader_index = MISS_STAGE,
    };

    groups[SHADOW_MISS] = daxa::RayTracingShaderGroupInfo{
        .type = daxa::ShaderGroup::GENERAL,
        .general_shader_index = MISS2_STAGE,
    };

    groups[PROCEDURAL_HIT] = daxa::RayTracingShaderGroupInfo{
        .type = daxa::ShaderGroup::PROCEDURAL_HIT_GROUP,
        .closest_hit_shader_index = CLOSE_HIT_STAGE,
        .intersection_shader_index = INTERSECTION_STAGE,
    };

    groups[LBVH_HIT] = daxa::RayTracingShaderGroupInfo{
        .type = daxa::ShaderGroup::PROCEDURAL_HIT_GROUP,
        .any_hit_shader_index = ANY_HIT_LBVH_STAGE,
        .intersection_shader_index = INTERSECTION_LBVH_STAGE,
    };

    info = daxa::RayTracingPipelineCompileInfo{
        .ray_gen_infos = ray_gen_infos,
        .intersection_infos = intersection_infos,
        .any_hit_infos = any_hit_infos,
        .closest_hit_infos = closest_hit_infos,
        .miss_hit_infos = miss_hit_infos,
        .shader_groups_infos = groups,
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
          .required_subgroup_size = SUBGROUP_SIZE,
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

struct RigidBodyConvertBoundingBoxesLinearBVHInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = entry_convert_bounding_boxes_linear_bvh,
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(RigidBodyConvertBoundingBoxesLBVHPushConstants),
      .name = convert_bounding_boxes_linear_bvh_pipeline_name,
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

// ---- graph coloring pipelines (all share GraphColorPushConstants / GraphColorTaskHead) ----
struct GraphColorDispatcherInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{coloring_shader_file_string},
      .compile_options = { .entry_point = entry_graph_color_dispatcher, },
  };
  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(RigidBodyDispatcherPushConstants),
      .name = graph_color_dispatcher_pipeline_name,
  };
};
struct GraphColorResetInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{coloring_shader_file_string},
      .compile_options = { .entry_point = entry_graph_color_reset, },
  };
  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(GraphColorPushConstants),
      .name = graph_color_reset_pipeline_name,
  };
};
struct GraphColorOwnerResetInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{coloring_shader_file_string},
      .compile_options = { .entry_point = entry_graph_color_owner_reset, },
  };
  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(GraphColorPushConstants),
      .name = graph_color_owner_reset_pipeline_name,
  };
};
struct GraphColorAssignP1Info {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{coloring_shader_file_string},
      .compile_options = { .entry_point = entry_graph_color_assign_p1, },
  };
  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(GraphColorPushConstants),
      .name = graph_color_assign_p1_pipeline_name,
  };
};
struct GraphColorAssignP2Info {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{coloring_shader_file_string},
      .compile_options = { .entry_point = entry_graph_color_assign_p2, },
  };
  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(GraphColorPushConstants),
      .name = graph_color_assign_p2_pipeline_name,
  };
};
struct GraphColorValidateInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{coloring_shader_file_string},
      .compile_options = { .entry_point = entry_graph_color_validate, },
  };
  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(GraphColorPushConstants),
      .name = graph_color_validate_pipeline_name,
  };
};
// per-color solver pipelines (entries in RB_sim.slang; share GraphColorSolvePushConstants)
struct GraphColorPreSolverInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = { .entry_point = entry_collision_pre_solver_color, },
  };
  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(GraphColorSolvePushConstants),
      .name = graph_color_pre_solver_pipeline_name,
  };
};
struct GraphColorSolverInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = { .entry_point = entry_collision_solver_color, },
  };
  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(GraphColorSolvePushConstants),
      .name = graph_color_solver_pipeline_name,
  };
};
struct GraphColorRelaxInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = { .entry_point = entry_collision_solver_relax_color, },
  };
  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(GraphColorSolvePushConstants),
      .name = graph_color_relax_pipeline_name,
  };
};
// overflow bucket: single-thread serial solve of manifolds the per-color dispatches skip
struct GraphColorPreSolverOverflowInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = { .entry_point = entry_collision_pre_solver_overflow, },
  };
  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(GraphColorSolvePushConstants),
      .name = graph_color_pre_solver_overflow_pipeline_name,
  };
};
struct GraphColorSolverOverflowInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = { .entry_point = entry_collision_solver_overflow, },
  };
  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(GraphColorSolvePushConstants),
      .name = graph_color_solver_overflow_pipeline_name,
  };
};
struct GraphColorRelaxOverflowInfo {
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = { .entry_point = entry_collision_solver_relax_overflow, },
  };
  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(GraphColorSolvePushConstants),
      .name = graph_color_relax_overflow_pipeline_name,
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
