#pragma once

#include <vector>

#include "shared.inl"
#include <daxa/daxa.hpp>
using namespace daxa::types;
#include <daxa/utils/pipeline_manager.hpp>

#define BB_NAMESPACE_BEGIN namespace beatbox {
#define BB_NAMESPACE_END }


BB_NAMESPACE_BEGIN

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

auto shader_file_string = "ray_tracing.slang";

struct MainRayTracingPipeline
{
  daxa::ShaderCompileInfo rt_gen_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{shader_file_string},
      .compile_options = {
          .entry_point = to_string(RAYGEN),
      },
  };

  daxa::ShaderCompileInfo rt_miss_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{shader_file_string},
      .compile_options = {
          .entry_point = to_string(MISS),
      },
  };

  daxa::ShaderCompileInfo rt_miss_shadow_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{shader_file_string},
      .compile_options = {
          .entry_point = to_string(MISS2),
      },
  };

  daxa::ShaderCompileInfo rt_closest_hit_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{shader_file_string},
      .compile_options = {
          .entry_point = to_string(CLOSE_HIT),
      },
  };

  daxa::ShaderCompileInfo rt_intersection_shader = daxa::ShaderCompileInfo{
      .source = daxa::ShaderFile{shader_file_string},
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
        .push_constant_size = sizeof(PushConstants),
        .name = "main ray tracing pipeline",
    };
  }
};

BB_NAMESPACE_END