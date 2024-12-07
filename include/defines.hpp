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

static constexpr u32 MAX_VERTEX_COUNT = 1024;

static constexpr u32 DOUBLE_BUFFERING = 2;

static constexpr f32 TIME_STEP = 0.01667f;
static constexpr f32 GRAVITY = 9.81f;
static constexpr u32 MAX_PRIMITIVE_COUNT = 1024;
static constexpr u32 MAX_RIGID_BODY_COUNT = 1024;
static constexpr u32 MAX_LIGHT_COUNT = 1024;
static constexpr u32 MAX_MATERIAL_COUNT = 1024;
static constexpr u32 DEFAULT_ITERATION_COUNT = 4;

enum StageIndex : u32
{
  RAYGEN,
  MISS,
  MISS2,
  CLOSE_HIT,
  INTERSECTION,
  STAGES_COUNT,
};

enum GroupIndex : u32
{
  PRIMARY_RAY,
  HIT_MISS,
  SHADOW_MISS,
  PROCEDURAL_HIT,
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
  default:
    return "unknown";
  }
}

const auto RT_shader_file_string = "ray_tracing.slang";
const auto RT_main_pipeline_name = "Main Ray Tracing Pipeline";

const auto RB_sim_shader_file_string = "RB_sim.slang";
// reset body links
const auto entry_reset_body_links = "entry_reset_body_links";
const auto reset_body_links_pipeline_name = "Reset Body Links";

// broad phase
const auto entry_broad_phase_sim = "entry_broad_phase";
const auto broad_phase_sim_pipeline_name = "broad phase Simulation";

// collision solver dispatcher
const auto entry_collision_solver_dispatcher = "entry_collision_solver_dispatcher";
const auto collision_solver_dispatcher_pipeline_name = "Collision Solver Dispatcher";

// island counter
const auto entry_island_counter = "entry_island_counter";
const auto island_counter_pipeline_name = "Island Counter";

// island builder
const auto entry_island_builder = "entry_island_builder";
const auto island_builder_pipeline_name = "Island Builder";

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

    info = daxa::RayTracingPipelineCompileInfo{
        .stages = stages,
        .groups = groups,
        .max_ray_recursion_depth = 2,
        .push_constant_size = sizeof(RTPushConstants),
        .name = RT_main_pipeline_name,
    };
  }
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