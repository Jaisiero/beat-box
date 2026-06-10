#pragma once
#include "defines.hpp"
#include <map>
#include <string>
#include <variant>
#include "gpu_context.hpp"

BB_NAMESPACE_BEGIN

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

template <typename TaskType, typename Callback>
struct TaskTemplate;

template <typename HeadInfo, typename Callback>
struct TaskTemplate<daxa::TInlineTask<HeadInfo>, Callback> : public daxa::TInlineTask<HeadInfo>
{
  using Base = daxa::TInlineTask<HeadInfo>;
  using AttachmentViews = typename HeadInfo::AttachmentViews;

  Callback user_callback;

  template <std::size_t N>
  TaskTemplate(std::array<daxa::TaggedTaskView, N> const &views_, Callback user_callback_)
      : Base(HeadInfo::NAME, HeadInfo::TYPE), user_callback(user_callback_)
  {
    this->head_views(daxa::make_attachment_views<AttachmentViews>(views_));
    auto *callback = daxa::retain_task_callback([callback = user_callback](daxa::TaskInterface const &ti) mutable
    {
      auto ti_copy = ti;
      auto self = daxa::NoTaskHeadStruct{};
      callback(ti_copy, self);
    });
    this->executes([callback](daxa::TaskInterface ti)
    {
      (*callback)(ti);
    });
  }

  [[nodiscard]] auto as_inline_task() const -> Base const &
  {
    return *this;
  }
};

template<typename T>
inline void allocate_fill_copy(daxa::TaskInterface ti, T value, daxa::TaskBufferAttachmentInfo dst, u32 dst_offset = 0)
{
    auto alloc = ti.allocator->allocate_fill(value).value();
    ti.recorder.copy_buffer_to_buffer({
        .src_buffer = ti.allocator->buffer(),
        .dst_buffer = dst.id,
        .src_offset = alloc.buffer_offset,
        .dst_offset = dst_offset,
        .size = sizeof(T),
    });
}

template<typename T>
inline void allocate_fill_copy(daxa::TaskInterface ti, std::vector<T> value, daxa::TaskBufferAttachmentInfo dst, u32 dst_offset = 0)
{
  auto size = static_cast<daxa::u32>(value.size() * sizeof(T));
  auto alloc = ti.allocator->allocate(size).value();
  memcpy(alloc.host_address, value.data(),size);
  ti.recorder.copy_buffer_to_buffer({
      .src_buffer = ti.allocator->buffer(),
      .dst_buffer = dst.id,
      .src_offset = alloc.buffer_offset,
      .dst_offset = dst_offset,
      .size = size,
  });
}

FORCE_INLINE std::vector<std::filesystem::path> paths{
    DAXA_SHADER_INCLUDE_DIR,
    "include",
    "src/shaders",
    "src/shaders/path_tracing",
    "src/shaders/simulation",
};

#if defined(_DEBUG)
FORCE_INLINE std::filesystem::path debug_path{
    "shaders_debug",
};
#endif

struct TaskGraph
{
  daxa::TaskGraph task_graph;
  std::map<std::string, daxa::TaskBuffer> task_buffers;
  std::map<std::string, daxa::TaskImage> task_images;
  std::map<std::string, daxa::TaskTlas> task_tlas;
  std::map<std::string, daxa::TaskBlas> task_blas;
  // every task added to this graph is forced onto this queue (see add_task)
  daxa::Queue queue = daxa::QUEUE_MAIN;

  TaskGraph() = default;

  TaskGraph(daxa::TaskGraph task_graph_, daxa::Queue queue_ = daxa::QUEUE_MAIN)
      : task_graph(task_graph_), queue(queue_) {}


  void add_task(auto task)
  {
    // Force every task onto this graph's queue: tasks default to QUEUE_MAIN (not QUEUE_NONE),
    // so TaskGraphInfo::default_queue alone would never apply to them.
    if constexpr (requires { task.uses_queue(queue); })
    {
      task.uses_queue(queue);
    }
    if constexpr (requires { task.as_inline_task(); })
    {
      task_graph.add_task(task.as_inline_task());
    }
    else if constexpr (std::is_same_v<std::decay_t<decltype(task)>, daxa::InlineTaskInfo>)
    {
      auto inline_task = task.to_inline_task();
      inline_task.uses_queue(queue);
      task_graph.add_task(std::move(inline_task));
    }
    else
    {
      task_graph.add_task(task);
    }
  }

  void submit()
  {
    task_graph.submit({});
  }

  void submit(daxa::TaskSubmitInfo const & info)
  {
    task_graph.submit(info);
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

  void add_buffer(std::string_view name, daxa::TaskBuffer buffer)
  {
    task_buffers[std::string{name}] = buffer;
    task_graph.register_buffer(buffer);
  }

  void add_image(std::string_view name, daxa::TaskImage image)
  {
    task_images[std::string{name}] = image;
    task_graph.register_image(image);
  }

  void add_tlas(std::string_view name, daxa::TaskTlas tlas)
  {
    task_tlas[std::string{name}] = tlas;
    task_graph.register_tlas(tlas);
  }

  void add_blas(std::string_view name, daxa::TaskBlas blas)
  {
    task_blas[std::string{name}] = blas;
    task_graph.register_blas(blas);
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
        .root_paths = paths,
#if defined(_DEBUG)
        .write_out_spirv = debug_path,
#endif
        .default_language = daxa::ShaderLanguage::SLANG,
#if defined(_DEBUG)
        .default_enable_debug_info = true,
#endif
        .name = pipeline_mngr_name,
    });
  }

  [[nodiscard]] auto create_ray_tracing(daxa::RayTracingPipelineCompileInfo info) -> std::shared_ptr<daxa::RayTracingPipeline>
  {
    return pipeline_manager.add_ray_tracing_pipeline2(static_cast<daxa::RayTracingPipelineCompileInfo2>(info)).value();
  }

  [[nodiscard]] auto create_compute(daxa::ComputePipelineCompileInfo info) -> std::shared_ptr<daxa::ComputePipeline>
  {
    auto result = pipeline_manager.add_compute_pipeline2(static_cast<daxa::ComputePipelineCompileInfo2>(info));
    if (result.is_err())
    {
      // surface the compile error before .value() throws a bare bad-optional-access
      std::cerr << "[PIPELINE ERROR] " << std::string_view{info.name.data(), info.name.size()} << ":\n"
                << result.message() << std::endl;
    }
    return result.value();
  }

  [[nodiscard]] auto create_raster(daxa::RasterPipelineCompileInfo info) -> std::shared_ptr<daxa::RasterPipeline>
  {
    return pipeline_manager.add_raster_pipeline2(static_cast<daxa::RasterPipelineCompileInfo2>(info)).value();
  }

  auto reload() -> daxa::PipelineReloadResult
  {
    return pipeline_manager.reload_all();
  }

  TaskGraph& create_task_graph(char const *name, std::span<daxa::TaskBuffer> buffers, std::span<daxa::TaskImage> images, std::span<daxa::TaskBlas> blases, std::span<daxa::TaskTlas> tlases, bool is_swapchain = false, daxa::Queue queue = daxa::QUEUE_MAIN)
  {
    auto TG = TaskGraph(daxa::TaskGraph({
        .device = gpu->device,
        .swapchain = is_swapchain? gpu->swapchain : std::optional<daxa::Swapchain>(),
        .default_queue = queue,
        .name = name,
    }), queue);
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
  std::span<daxa::TaskBuffer> buffers, std::span<daxa::TaskImage> images, std::span<daxa::TaskBlas> blases, std::span<daxa::TaskTlas> tlases, bool is_swapchain = false, daxa::Queue queue = daxa::QUEUE_MAIN)
  {
    auto TG = TaskGraph(daxa::TaskGraph({
        .device = gpu->device,
        .swapchain = is_swapchain? gpu->swapchain : std::optional<daxa::Swapchain>(),
        .default_queue = queue,
        .name = name,
    }), queue);
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
