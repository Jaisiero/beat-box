#pragma once
#include "defines.hpp"
#include <map>
#include <variant>
#include "gpu_context.hpp"

BB_NAMESPACE_BEGIN

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

template <typename TaskType, typename Callback>
struct TaskTemplate : public TaskType
{
  using AttachmentViews = typename TaskType::AttachmentViews;

  // Stores the attachment views
  AttachmentViews views = {};

  // The user-defined callback function
  Callback user_callback;

  // Constructor to set up views and user-defined callback
  TaskTemplate(
      AttachmentViews const &views_,
      Callback user_callback_)
      : views(views_), user_callback(user_callback_) {}

  // Defaulted copy and move constructors and assignment operators
  TaskTemplate(const TaskTemplate&) = default;
  TaskTemplate& operator=(const TaskTemplate&) = default;
  TaskTemplate(TaskTemplate&&) noexcept = default;
  TaskTemplate& operator=(TaskTemplate&&) noexcept = default;

  // The callback function called by the task graph
  void callback(daxa::TaskInterface ti)
  {
    // Call the user-defined callback function, passing in 'this' as 'self'
    user_callback(ti, *this);
  }
};

template<typename T>
inline void allocate_fill_copy(daxa::TaskInterface ti, T value, daxa::TaskBufferAttachmentInfo dst, u32 dst_offset = 0)
{
    auto alloc = ti.allocator->allocate_fill(value).value();
    ti.recorder.copy_buffer_to_buffer({
        .src_buffer = ti.allocator->buffer(),
        .dst_buffer = dst.ids[0],
        .src_offset = alloc.buffer_offset,
        .dst_offset = dst_offset,
        .size = sizeof(T),
    });
}

FORCE_INLINE std::vector<std::filesystem::path> paths{
    DAXA_SHADER_INCLUDE_DIR,
    "include",
    "src/shaders",
};

struct TaskGraph
{
  daxa::TaskGraph task_graph;
  std::map<daxa::SmallString, daxa::TaskBuffer> task_buffers;
  std::map<daxa::SmallString, daxa::TaskImage> task_images;
  std::map<daxa::SmallString, daxa::TaskTlas> task_tlas;
  std::map<daxa::SmallString, daxa::TaskBlas> task_blas;

  TaskGraph() = default;

  TaskGraph(daxa::TaskGraph task_graph_)
      : task_graph(task_graph_) {}


  void add_task(auto task)
  {
    task_graph.add_task(task);
  }

  void submit()
  {
    task_graph.submit({});
  }

  void present()
  {
    task_graph.present({});
  }

  void complete()
  {
    task_graph.complete({});
  }

  void execute()
  {
    task_graph.execute({});
  }

  void add_buffer(daxa::SmallString name, daxa::TaskBuffer buffer)
  {
    task_buffers[name] = buffer;
    task_graph.use_persistent_buffer(buffer);
  }

  void add_image(daxa::SmallString name, daxa::TaskImage image)
  {
    task_images[name] = image;
    task_graph.use_persistent_image(image);
  }

  void add_tlas(daxa::SmallString name, daxa::TaskTlas tlas)
  {
    task_tlas[name] = tlas;
    task_graph.use_persistent_tlas(tlas);
  }

  void add_blas(daxa::SmallString name, daxa::TaskBlas blas)
  {
    task_blas[name] = blas;
    task_graph.use_persistent_blas(blas);
  }

};

struct TaskManager
{
  // Gpu context
  std::shared_ptr<GPUcontext> gpu;
  // The pipeline manager
  daxa::PipelineManager pipeline_manager;
  // Task graphs
  // std::map<std::string, TaskGraph> task_graphs;
  std::vector<TaskGraph> task_graphs;

  explicit TaskManager(char const *pipeline_mngr_name, std::shared_ptr<GPUcontext> gpu) : gpu(gpu)
  {
    pipeline_manager = daxa::PipelineManager({
        .device = gpu->device,
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

  TaskGraph& create_task_graph(char const *name, std::span<daxa::TaskBuffer> buffers, std::span<daxa::TaskImage> images, std::span<daxa::TaskBlas> blases, std::span<daxa::TaskTlas> tlases, bool is_swapchain = false)
  {
    auto TG = TaskGraph(daxa::TaskGraph({
        .device = gpu->device,
        .swapchain = is_swapchain? gpu->swapchain : std::optional<daxa::Swapchain>(),
        .name = name,
    }));
    for (auto buffer : buffers)
    {
      TG.add_buffer(buffer.info().name, buffer);
    }
    for (auto image : images)
    {
      TG.add_image(image.info().name, image);
    }
    for (auto tlas : tlases)
    {
      TG.add_tlas(tlas.info().name, tlas);
    }
    for (auto blas : blases)
    {
      TG.add_blas(blas.info().name, blas);
    }
    return task_graphs.emplace_back(TG);
  }

  template<typename TTask>
  TaskGraph& create_task_graph(char const *name,  std::span<TTask> tasks, 
  std::span<daxa::TaskBuffer> buffers, std::span<daxa::TaskImage> images, std::span<daxa::TaskBlas> blases, std::span<daxa::TaskTlas> tlases, bool is_swapchain = false)
  {
    auto TG = TaskGraph(daxa::TaskGraph({
        .device = gpu->device,
        .swapchain = is_swapchain? gpu->swapchain : std::optional<daxa::Swapchain>(),
        .name = name,
    }));
    for (auto buffer : buffers)
    {
      TG.add_buffer(buffer.info().name, buffer);
    }
    for (auto image : images)
    {
      TG.add_image(image.info().name, image);
    }
    for (auto tlas : tlases)
    {
      TG.add_tlas(tlas.info().name, tlas);
    }
    for (auto blas : blases)
    {
      TG.add_blas(blas.info().name, blas);
    }
    // Add tasks using std::visit
    for (auto& task : tasks)
    {
      TG.add_task(task);
    }
    
    
    return task_graphs.emplace_back(TG);
  }
};

BB_NAMESPACE_END