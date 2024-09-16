#pragma once

#include "defines.hpp"
#include "camera_manager.hpp"

BB_NAMESPACE_BEGIN  

struct InputManager{

  // Pointer to the camera manager
  std::shared_ptr<CameraManager> camera_mngr;
  // initialize the input manager
  bool initialized = false;

  explicit InputManager() {}

  ~InputManager() {};

  bool create(std::shared_ptr<CameraManager> camera_manager) {
    if (initialized) return false;
    initialized = true;
    this->camera_mngr = camera_manager;
    return initialized;
  }

  void destroy() {
    if (!initialized) return;
    initialized = false;
  }


  void on_mouse_move(f32 x, f32 y) {
    camera_set_mouse_delta(camera_mngr->camera, glm::vec2{x, y});
  }

  void on_mouse_button(i32 button, i32 action, f32 x, f32 y) {
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

  void on_key(i32 key, i32 action) {
    switch (key)
    {
    case GLFW_KEY_W:
    case GLFW_KEY_UP:
      if (action == GLFW_PRESS || action == GLFW_REPEAT)
      {
        move_camera_forward(camera_mngr->camera);
      }
      break;
    case GLFW_KEY_S:
    case GLFW_KEY_DOWN:
      if (action == GLFW_PRESS || action == GLFW_REPEAT)
      {
        move_camera_backward(camera_mngr->camera);
      }
      break;
    case GLFW_KEY_A:
    case GLFW_KEY_LEFT:
      if (action == GLFW_PRESS || action == GLFW_REPEAT)
      {
        move_camera_left(camera_mngr->camera);
      }
      break;
    case GLFW_KEY_D:
    case GLFW_KEY_RIGHT:
      if (action == GLFW_PRESS || action == GLFW_REPEAT)
      {
        move_camera_right(camera_mngr->camera);
      }
      break;
    case GLFW_KEY_X:
      if (action == GLFW_PRESS || action == GLFW_REPEAT)
      {
        move_camera_up(camera_mngr->camera);
      }
      break;
    case GLFW_KEY_SPACE:
      if (action == GLFW_PRESS || action == GLFW_REPEAT)
      {
        move_camera_down(camera_mngr->camera);
      }
      break;
    case GLFW_KEY_ESCAPE:
      if (action == GLFW_PRESS)
      {
        // windows.window_close();
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

  void on_scroll(f32 x, f32 y) {
  }


};

BB_NAMESPACE_END