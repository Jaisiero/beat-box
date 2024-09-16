#pragma once
#include "defines.hpp"

BB_NAMESPACE_BEGIN

std::vector<std::filesystem::path> paths{
    DAXA_SHADER_INCLUDE_DIR,
    "include",
    "src/shaders",
};

struct PipelineManager{

  daxa::PipelineManager pipeline_manager;

  explicit PipelineManager(char const * pipeline_mngr_name, daxa::Device& device)
  {
    pipeline_manager = daxa::PipelineManager({
        .device = device,
        .shader_compile_options = {
            .root_paths = paths,
            .language = daxa::ShaderLanguage::SLANG,
#if defined(_DEBUG)
            .enable_debug_info = true,
#endif
        },
        .name = pipeline_mngr_name,
    });
  }

  [[nodiscard]] auto create_ray_tracing(daxa::RayTracingPipelineCompileInfo info) -> std::shared_ptr<daxa::RayTracingPipeline>
  {
    return pipeline_manager.add_ray_tracing_pipeline(info).value();
  }

  [[nodiscard]] auto create_compute(daxa::ComputePipelineCompileInfo info) -> std::shared_ptr<daxa::ComputePipeline>
  {
    return pipeline_manager.add_compute_pipeline(info).value();
  }

  auto reload() -> daxa::PipelineReloadResult
  {
    return pipeline_manager.reload_all();
  }

};

BB_NAMESPACE_END