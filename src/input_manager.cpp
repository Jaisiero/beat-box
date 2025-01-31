#include "input_manager.hpp"
#include "status_manager.hpp"

BB_NAMESPACE_BEGIN

void InputManager::on_mouse_move(f32 x, f32 y)
{
  camera_set_mouse_delta(camera_mngr->camera, glm::vec2{x, y});
  if(camera_mngr->camera.mouse_left_press)
  {
    status_mngr->reset_accumulation_count();
  }
}

void InputManager::on_mouse_button(i32 button, i32 action, f32 x, f32 y)
{
  if (button == GLFW_MOUSE_BUTTON_1)
  {
    camera_set_last_mouse_pos(camera_mngr->camera, glm::vec2(x, y));
    // Click right button store the current mouse position
    if (action == GLFW_PRESS)
    {
      camera_set_mouse_left_press(camera_mngr->camera, true);
    }
    else if (action == GLFW_RELEASE)
    {
      camera_set_mouse_left_press(camera_mngr->camera, false);
    }
  }
  else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
  {
    if (action == GLFW_PRESS)
    {
      camera_set_mouse_middle_pressed(camera_mngr->camera, true);
    }
    else if (action == GLFW_RELEASE)
    {
      camera_set_mouse_middle_pressed(camera_mngr->camera, false);
    }
  }
}

void InputManager::on_key(i32 key, i32 action)
{
  switch (key)
  {
  case GLFW_KEY_W:
  case GLFW_KEY_UP:
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
      move_camera_forward(camera_mngr->camera);
      status_mngr->reset_accumulation_count();
    }
    break;
  case GLFW_KEY_S:
  case GLFW_KEY_DOWN:
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
      move_camera_backward(camera_mngr->camera);
      status_mngr->reset_accumulation_count();
    }
    break;
  case GLFW_KEY_A:
  case GLFW_KEY_LEFT:
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
      move_camera_left(camera_mngr->camera);
      status_mngr->reset_accumulation_count();
    }
    break;
  case GLFW_KEY_D:
  case GLFW_KEY_RIGHT:
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
      move_camera_right(camera_mngr->camera);
      status_mngr->reset_accumulation_count();
    }
    break;
  case GLFW_KEY_X:
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
      move_camera_up(camera_mngr->camera);
      status_mngr->reset_accumulation_count();
    }
    break;
  case GLFW_KEY_Z:
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
      move_camera_down(camera_mngr->camera);
      status_mngr->reset_accumulation_count();
    }
    break;
  case GLFW_KEY_SPACE:
    if (action == GLFW_PRESS)
    {
      status_mngr->switch_simulating();
    }
    break;
  case GLFW_KEY_TAB:
    if (action == GLFW_PRESS)
    {
      status_mngr->switch_gui_enabled();
    }
    break;
  case GLFW_KEY_GRAVE_ACCENT:
    if (action == GLFW_PRESS)
    {
      status_mngr->switch_axis_enabled();
    }
    break;
  case GLFW_KEY_L:
    if (action == GLFW_PRESS)
    {
      status_mngr->switch_bvh_enabled();
    }
    break;
  case GLFW_KEY_0:
    if (action == GLFW_PRESS)
    {
      status_mngr->switch_accumulating();
    }
    break;
  case GLFW_KEY_1:
    if (action == GLFW_PRESS)
    {
      status_mngr->set_solver(SimSolverType::PGS);
    }
    break;
  case GLFW_KEY_2:
    if (action == GLFW_PRESS)
    {
      status_mngr->set_solver(SimSolverType::PGS_SOFT);
    }
    break;
  case GLFW_KEY_7:
    if (action == GLFW_PRESS)
    {
      status_mngr->switch_show_collisions();
    }
    break;
  case GLFW_KEY_8:
    if (action == GLFW_PRESS)
    {
      status_mngr->switch_show_normals();
    }
    break;
  case GLFW_KEY_9:
    if (action == GLFW_PRESS)
    {
      status_mngr->switch_show_islands();
    }
    break;
  case GLFW_KEY_P:
    if (action == GLFW_PRESS)
    {
      status_mngr->switch_warm_starting();
    }
    break;
  case GLFW_KEY_LEFT_CONTROL:
    if (action == GLFW_PRESS)
    {
      status_mngr->switch_advection();
    }
    break;
  case GLFW_KEY_LEFT_SHIFT:
    if (action == GLFW_PRESS)
    {
      camera_shift_pressed(camera_mngr->camera);
    }
    else if (action == GLFW_RELEASE)
    {
      camera_shift_released(camera_mngr->camera);
    }
    break;
  default:
    break;
  }
}

void InputManager::on_scroll(f32 x, f32 y)
{
}

BB_NAMESPACE_END