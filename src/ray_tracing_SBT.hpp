#pragma once
#include "defines.hpp"

BB_NAMESPACE_BEGIN

struct RayTracingSBT
{

  // Daxa Ray Tracing Pipeline
  std::shared_ptr<daxa::RayTracingPipeline> ray_tracing_pipeline;

  // Daxa device
  daxa::Device& device;

  // Shader Binding Table
  daxa::RayTracingPipeline::SbtPair sbt_pair;

  explicit RayTracingSBT(std::shared_ptr<daxa::RayTracingPipeline> ray_tracing_pipeline, daxa::Device &device) : ray_tracing_pipeline(ray_tracing_pipeline), device(device)
  {
    sbt_pair = ray_tracing_pipeline->create_default_sbt();
  };

  ~RayTracingSBT()
  {
    free_sbt();
  };

  auto free_sbt(daxa::RayTracingPipeline::SbtPair &sbt) -> void
  {
    if(device.is_valid())
    {
      if(!sbt.buffer.is_empty())
      {
        device.destroy_buffer(sbt.buffer);
      }
      if(!sbt.entries.buffer.is_empty())
      {
        device.destroy_buffer(sbt.entries.buffer);
      }
    }
  };

  [[nodiscard]] auto rebuild_sbt() -> daxa::RayTracingShaderBindingTable
  {
    if (!sbt_pair.buffer.is_empty())
    {
      free_sbt();
    }
    sbt_pair = ray_tracing_pipeline->create_default_sbt();
    return build_sbt();
  }

  [[nodiscard]] auto build_sbt() -> daxa::RayTracingShaderBindingTable
  {
    return {
        .raygen_region = sbt_pair.entries.group_regions.at(0).region,
        .miss_region = sbt_pair.entries.group_regions.at(1).region,
        .hit_region = sbt_pair.entries.group_regions.at(2).region,
        .callable_region = {},
    };
  };
private:

  auto free_sbt() -> void
  {
    free_sbt(sbt_pair);
  };
};

BB_NAMESPACE_END