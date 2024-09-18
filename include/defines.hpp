#pragma once

#include <vector>

#include "shared.inl"
#include <daxa/daxa.hpp>
using namespace daxa::types;
#include <daxa/utils/pipeline_manager.hpp>
#include <daxa/utils/task_graph.hpp>

#define BB_NAMESPACE_BEGIN namespace beatbox {
#define BB_NAMESPACE_END }


BB_NAMESPACE_BEGIN

const f32 TIME_STEP = 0.001f;
const f32 GRAVITY = 9.81f;

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

std::string to_string(StageIndex index)
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
const auto RB_sim_shader_file_string = "RB_sim.slang";
const auto AS_shader_file_string = "AS_mngr.slang";

const auto RT_main_pipeline_name = "Main Ray Tracing Pipeline";
const auto RB_sim_pipeline_name = "Rigid Body Simulation";
const auto AS_update_pipeline_name = "Update Acceleration Structures";

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


struct RigidBodySim
{
  daxa::ShaderCompileInfo compute_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{RB_sim_shader_file_string},
      .compile_options = {
          .entry_point = "entry_rigid_body_sim",
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
          .entry_point = "entry_update_acceleration_structures",
      },
  };

  daxa::ComputePipelineCompileInfo info = {
      .shader_info = compute_shader,
      .push_constant_size = sizeof(UpdateInstancesPushConstants),
      .name = AS_update_pipeline_name,
  };
};

BB_NAMESPACE_END