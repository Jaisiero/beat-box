#include "window.hpp"
#include "gpu_context.hpp"
#include "pipeline_manager.hpp"
#include "defines.hpp"

#include "shared.inl"

using namespace beatbox;

int main(int argc, char const *argv[])
{
    // Create a window
    auto window = AppWindow("Beat Box", 860, 640);

    auto gpu = GPUcontext(window);

    auto pipeline_manager = PipelineManager(gpu.device);

    {
        auto ray_tracing_pipeline_info = MainRayTracingPipeline{};
        auto ray_tracing_pipeline = pipeline_manager.create_ray_tracing(ray_tracing_pipeline_info.info);
    }

    while (!window.should_close())
    {
        window.update();
    }

    return 0;
}