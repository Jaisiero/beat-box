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
  // Shader Binding Table
  daxa::RayTracingShaderBindingTable sbt = {};
  // flag for initialization
  bool initialized = false;

  explicit RayTracingPipeline(std::shared_ptr<daxa::RayTracingPipeline> pipeline, daxa::Device &device) : pipeline(pipeline), device(device)
  {
    (void)rebuild_SBT();
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
    sbt = {};
    initialized = false;
  };

  [[nodiscard]] auto rebuild_SBT() -> daxa::RayTracingShaderBindingTable
  {
    free_SBT();
    auto const sbt_pair = pipeline->create_default_sbt();
    sbt_buffer = sbt_pair.buffer;
    sbt = sbt_pair.table;
    initialized = true;
    return build_SBT();
  }

  [[nodiscard]] auto build_SBT() -> daxa::RayTracingShaderBindingTable
  {
    if (!initialized)
    {
      return rebuild_SBT();
    }

    return sbt;
  };
private:

  auto free_SBT() -> void
  {
    free_SBT(sbt_buffer);
  };
};

BB_NAMESPACE_END
