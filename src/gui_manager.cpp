#include "gui_manager.hpp"
#include "renderer_manager.hpp"

BB_NAMESPACE_BEGIN

bool GUIManager::create(std::shared_ptr<RendererManager> renderer, std::shared_ptr<StatusManager> status) {
  if (initialized) {
    return false;
  }

  renderer_manager = renderer;

  auto ui_pipeline = UIPipeline{};
  ui_pipeline.info.color_attachments.at(0).format = gpu->swapchain.get_format();
  gui_pipeline = task_manager->create_raster(ui_pipeline.info);

  for (auto f = 0; f < DOUBLE_BUFFERING; ++f)
    vertex_buffer[f] = gpu->device.create_buffer({
      .size = sizeof(GUIVertex) * MAX_VERTEX_COUNT,
      .name = "GUI vertex buffer " + std::to_string(f),
    });

  task_vertex_buffer.set_buffers({.buffers = std::array{vertex_buffer[0]}});
  task_previous_vertex_buffer.set_buffers({.buffers = std::array{vertex_buffer[1]}});

  gui_task_info = GUIDrawTask{
    .views = std::array{
      daxa::attachment_view(GUITaskHead::AT.render_target, renderer_manager->task_swapchain_image),
      daxa::attachment_view(GUITaskHead::AT.camera, renderer_manager->task_camera_buffer),
      daxa::attachment_view(GUITaskHead::AT.vertex_buffer, task_vertex_buffer),
    },
    .gui_pipeline = gui_pipeline,
    .rigid_body_manager = rigid_body_manager,
    .status_manager = status,
  };

  auto ui_line_pipeline = UILinePipeline{};
  ui_line_pipeline.info.color_attachments.at(0).format = gpu->swapchain.get_format();
  gui_line_pipeline = task_manager->create_raster(ui_line_pipeline.info);

  for (auto f = 0; f < DOUBLE_BUFFERING; ++f)
    vertex_line_buffer[f] = gpu->device.create_buffer({
      .size = sizeof(GUIVertexLine) * MAX_VERTEX_COUNT,
      .name = "GUI line vertex buffer " + std::to_string(f),
    });

  task_line_vertex_buffer.set_buffers({.buffers = std::array{vertex_buffer[0]}});
  task_previous_line_vertex_buffer.set_buffers({.buffers = std::array{vertex_buffer[1]}});

  gui_line_task_info = GUILineDrawTask{
    .views = std::array{
      daxa::attachment_view(GUILineTaskHead::AT.render_target, renderer_manager->task_swapchain_image),
      daxa::attachment_view(GUILineTaskHead::AT.camera, renderer_manager->task_camera_buffer),
      daxa::attachment_view(GUILineTaskHead::AT.vertex_buffer, task_line_vertex_buffer),
    },
    .gui_pipeline = gui_line_pipeline,
    .rigid_body_manager = rigid_body_manager,
    .status_manager = status,
  };


  auto ui_axes_pipeline = UIAxesPipeline{};
  ui_axes_pipeline.info.color_attachments.at(0).format = gpu->swapchain.get_format();
  gui_axes_pipeline = task_manager->create_raster(ui_axes_pipeline.info);

  for(auto f = 0; f < DOUBLE_BUFFERING; ++f)
    axes_vertex_buffer[f] = gpu->device.create_buffer({
      .size = sizeof(GUIVertexLine) * MAX_AXIS_COUNT,
      .name = "GUI axes vertex buffer " + std::to_string(f),
    });

  task_axes_vertex_buffer.set_buffers({.buffers = std::array{axes_vertex_buffer[0]}});

  gui_axes_task_info = GUIAxesDrawTask{
    .views = std::array{
      daxa::attachment_view(GUIAxesTaskHead::AT.render_target, renderer_manager->task_swapchain_image),
      daxa::attachment_view(GUIAxesTaskHead::AT.camera, renderer_manager->task_camera_buffer),
      daxa::attachment_view(GUIAxesTaskHead::AT.vertex_buffer, task_axes_vertex_buffer),
    },
    .gui_pipeline = gui_axes_pipeline,
    .rigid_body_manager = rigid_body_manager,
    .status_manager = status,
  };

  return initialized = true;
}

daxa::BufferId GUIManager::get_vertex_buffer()
{
  if(!initialized) {
    return {};
  }
  return vertex_buffer[renderer_manager->get_frame_index()];
}

daxa::BufferId GUIManager::get_previous_vertex_buffer()
{
  if(!initialized) {
    return {};
  }
  return vertex_buffer[renderer_manager->get_previous_frame_index()];
}

daxa::BufferId GUIManager::get_line_vertex_buffer()
{
  if(!initialized) {
    return {};
  }
  return vertex_line_buffer[renderer_manager->get_frame_index()];
}

daxa::BufferId GUIManager::get_previous_line_vertex_buffer()
{
  if(!initialized) {
    return {};
  }
  return vertex_line_buffer[renderer_manager->get_previous_frame_index()];
}

daxa::BufferId GUIManager::get_axes_vertex_buffer()
{
  if(!initialized) {
    return {};
  }
  return axes_vertex_buffer[renderer_manager->get_frame_index()];
}

daxa::BufferId GUIManager::get_previous_axes_vertex_buffer()
{
  if(!initialized) {
    return {};
  }
  return axes_vertex_buffer[renderer_manager->get_previous_frame_index()];
}

void GUIManager::update_buffers() {
  task_vertex_buffer.set_buffers({.buffers = std::array{vertex_buffer[renderer_manager->get_frame_index()]}});
  task_previous_vertex_buffer.set_buffers({.buffers = std::array{vertex_buffer[renderer_manager->get_previous_frame_index()]}});
  task_line_vertex_buffer.set_buffers({.buffers = std::array{vertex_line_buffer[renderer_manager->get_frame_index()]}});
  task_previous_line_vertex_buffer.set_buffers({.buffers = std::array{vertex_line_buffer[renderer_manager->get_previous_frame_index()]}});
  task_axes_vertex_buffer.set_buffers({.buffers = std::array{axes_vertex_buffer[renderer_manager->get_frame_index()]}});
  task_previous_axes_vertex_buffer.set_buffers({.buffers = std::array{axes_vertex_buffer[renderer_manager->get_previous_frame_index()]}});
}

BB_NAMESPACE_END