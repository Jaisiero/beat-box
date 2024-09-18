#pragma once
#include "defines.hpp"

BB_NAMESPACE_BEGIN

struct RayTracingPipeline
{

  // Daxa Ray Tracing Pipeline
  std::shared_ptr<daxa::RayTracingPipeline> pipeline;

  // Daxa device
  daxa::Device& device;

  // Shader Binding Table
  daxa::RayTracingPipeline::SbtPair sbt_pair;

  explicit RayTracingPipeline(std::shared_ptr<daxa::RayTracingPipeline> pipeline, daxa::Device &device) : pipeline(pipeline), device(device)
  {
    sbt_pair = pipeline->create_default_sbt();
  };

  ~RayTracingPipeline()
  {
    free_SBT();
  };

  auto free_SBT(daxa::RayTracingPipeline::SbtPair &sbt) -> void
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

  [[nodiscard]] auto rebuild_SBT() -> daxa::RayTracingShaderBindingTable
  {
    if (!sbt_pair.buffer.is_empty())
    {
      free_SBT();
    }
    sbt_pair = pipeline->create_default_sbt();
    return build_SBT();
  }

  [[nodiscard]] auto build_SBT() -> daxa::RayTracingShaderBindingTable
  {
    return {
        .raygen_region = sbt_pair.entries.group_regions.at(0).region,
        .miss_region = sbt_pair.entries.group_regions.at(1).region,
        .hit_region = sbt_pair.entries.group_regions.at(2).region,
        .callable_region = {},
    };
  };
private:

  auto free_SBT() -> void
  {
    free_SBT(sbt_pair);
  };
};

BB_NAMESPACE_END