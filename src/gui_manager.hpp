#pragma once

#include "defines.hpp"
#include "task_manager.hpp"
#include "window_manager.hpp"
#include "rigid_body_manager.hpp"
#include "status_manager.hpp"

BB_NAMESPACE_BEGIN

class RendererManager;

struct GUIDrawTask : GUITaskHead::Task
{
  AttachmentViews views = {};
  std::shared_ptr<daxa::RasterPipeline> gui_pipeline = {};
  std::shared_ptr<RigidBodyManager> rigid_body_manager = {};
  std::shared_ptr<StatusManager> status_manager = {};
  void callback(daxa::TaskInterface ti)
  {
    if(status_manager->is_gui_enabled()) {
      daxa::ImageInfo color_img_info = ti.info(AT.render_target).value();
      auto const size_x = color_img_info.size.x;
      auto const size_y = color_img_info.size.y;
      auto& sim_config = rigid_body_manager->get_sim_config_reference();
      auto render_recorder = std::move(ti.recorder).begin_renderpass({
        .color_attachments = std::array{
          daxa::RenderAttachmentInfo{
            .image_view = ti.get(GUITaskHead::AT.render_target).ids[0],
            .load_op = daxa::AttachmentLoadOp::LOAD,
          }
        },
        .render_area = {.x = 0, .y = 0, .width = size_x, .height = size_y},
      });
      render_recorder.set_pipeline(*gui_pipeline);
      render_recorder.push_constant(GUIPushConstants{
        .task_head = ti.attachment_shader_blob,
      });
      render_recorder.draw({.vertex_count = sim_config.g_c_info.collision_point_count});
      ti.recorder = std::move(render_recorder).end_renderpass();
    }
  }
};

struct GUIManager
{
  // GPU context reference
  std::shared_ptr<GPUcontext> gpu;
  // Windows manager reference
  WindowManager &window;
  // Initialization flag
  bool initialized = false;
  // Task manager reference
  std::shared_ptr<TaskManager> task_manager;
  // Renderer manager reference
  std::shared_ptr<RendererManager> renderer_manager;
  // Rigid body manager reference
  std::shared_ptr<RigidBodyManager> rigid_body_manager;
  // Raster pipeline for the GUI
  std::shared_ptr<daxa::RasterPipeline> gui_pipeline;
  // Instantiate the task using the template class
  GUIDrawTask gui_task_info;
  // vertex buffer
  daxa::BufferId vertex_buffer[DOUBLE_BUFFERING];
  // vertex task buffer
  daxa::TaskBuffer task_vertex_buffer{{.initial_buffers = {}, .name = "GUI task vertex buffer"}};
  // previous vertex task buffer
  daxa::TaskBuffer task_previous_vertex_buffer{{.initial_buffers = {}, .name = "GUI task previous vertex buffer"}};

  explicit GUIManager(std::shared_ptr<GPUcontext> gpu, WindowManager& window, std::shared_ptr<TaskManager> task_manager, std::shared_ptr<RigidBodyManager> rigid_body_manager) : gpu(gpu), window(window), task_manager(task_manager), rigid_body_manager(rigid_body_manager) {
  }
  
  ~GUIManager() { destroy(); }
  bool create(std::shared_ptr<RendererManager> renderer, std::shared_ptr<StatusManager> status);
  void destroy() {
    if (!initialized) {
      return;
    }

    for (auto f = 0; f < DOUBLE_BUFFERING; ++f)
      gpu->device.destroy_buffer(vertex_buffer[f]);

    initialized = false;
  }

  daxa::BufferId get_vertex_buffer();
  daxa::BufferId get_previous_vertex_buffer();

  bool update() {
    if(!initialized) {
      return initialized;
    }
    update_buffers();
    return initialized;
  }

private:
  void update_buffers();
};

BB_NAMESPACE_END