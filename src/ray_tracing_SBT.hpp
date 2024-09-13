#pragma once
#include "defines.hpp"

BB_NAMESPACE_BEGIN

struct RayTracingSBT
{

  // Daxa Ray Tracing Pipeline
  std::shared_ptr<daxa::RayTracingPipeline> ray_tracing_pipeline;

  // Daxa device
  daxa::Device device;

  // Shader Binding Table
  daxa::RayTracingPipeline::SbtPair sbt_pair;

  explicit RayTracingSBT(std::shared_ptr<daxa::RayTracingPipeline> ray_tracing_pipeline, daxa::Device &device) : ray_tracing_pipeline(ray_tracing_pipeline), device(device)
  {
    sbt_pair = ray_tracing_pipeline->create_default_sbt();
  };

  auto build_sbt() -> daxa::RayTracingShaderBindingTable
  {
    return {
        .raygen_region = sbt_pair.entries.group_regions.at(0).region,
        .miss_region = sbt_pair.entries.group_regions.at(1).region,
        .hit_region = sbt_pair.entries.group_regions.at(2).region,
        .callable_region = {},
    };
  };

  auto free_sbt(daxa::RayTracingPipeline::SbtPair &sbt) -> void
  {
    device.destroy_buffer(sbt.buffer);
    device.destroy_buffer(sbt.entries.buffer);
  };

  auto free_sbt() -> void
  {
    free_sbt(sbt_pair);
  };

  auto rebuild_sbt() -> daxa::RayTracingShaderBindingTable
  {
    if (!sbt_pair.buffer.is_empty())
    {
      free_sbt();
    }
    sbt_pair = ray_tracing_pipeline->create_default_sbt();
    return build_sbt();
  }

  ~RayTracingSBT()
  {
    free_sbt();
  };
};

BB_NAMESPACE_END