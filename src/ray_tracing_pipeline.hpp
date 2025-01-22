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
  daxa::BufferId sbt_buffer;
  // Region count
  u32 region_count = 0;
  // Shader Binding Table
  std::vector<daxa::GroupRegionInfo> regions = {};
  // flag for initialization
  bool initialized = false;
  std::vector<u32> groups = {};

  explicit RayTracingPipeline(std::shared_ptr<daxa::RayTracingPipeline> pipeline, daxa::Device &device) : pipeline(pipeline), device(device)
  {
    // FIXME: pass the groups as an argument
    groups = {GroupIndex::PRIMARY_RAY, GroupIndex::HIT_MISS, GroupIndex::SHADOW_MISS, GroupIndex::PROCEDURAL_HIT};
    usize sbt_size = 0;
    pipeline->create_sbt({groups}, &region_count, nullptr, &sbt_size, nullptr);
    regions.clear();
    regions.resize(region_count);
    pipeline->create_sbt({groups}, &region_count, regions.data(), nullptr, &sbt_buffer);
    initialized = true;
  };

  RayTracingPipeline(RayTracingPipeline const &other) = delete;

  ~RayTracingPipeline()
  {
    free_SBT();
  };

  auto free_SBT(daxa::BufferId buffer) -> void
  {
    if(!initialized) return;

    if(device.is_valid())
    {
      if(!buffer.is_empty())
      {
        device.destroy_buffer(buffer);
      }
    }
    initialized = false;
  };

  [[nodiscard]] auto rebuild_SBT() -> daxa::RayTracingShaderBindingTable
  {
    pipeline->create_sbt({groups}, &region_count, regions.data(), nullptr, &sbt_buffer);
    initialized = true;
    return build_SBT();
  }

  [[nodiscard]] auto build_SBT() -> daxa::RayTracingShaderBindingTable
  {
    if (!initialized)
    {
      return rebuild_SBT();
    }

    return {
        .raygen_region = regions.at(0).region,
        .miss_region = regions.at(1).region,
        .hit_region = regions.at(2).region,
        .callable_region = {},
    };
  };
private:

  auto free_SBT() -> void
  {
    free_SBT(sbt_buffer);
  };
};

BB_NAMESPACE_END