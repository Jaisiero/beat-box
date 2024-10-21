#pragma once

#include "defines.hpp"
#include "camera_manager.hpp"

BB_NAMESPACE_BEGIN  

class StatusManager;

struct InputManager{

  // Pointer to the camera manager
  std::shared_ptr<CameraManager> camera_mngr;
  // // Pointer to the status manager
  std::shared_ptr<StatusManager> status_mngr;
  // initialize the input manager
  bool initialized = false;

  explicit InputManager() {}

  ~InputManager() {};

  bool create(std::shared_ptr<CameraManager> camera_manager
  , std::shared_ptr<StatusManager> status_mngr
  ) {
    if (initialized) return false;
    initialized = true;
    this->camera_mngr = camera_manager;
    this->status_mngr = status_mngr;
    return initialized;
  }

  void destroy() {
    if (!initialized) return;
    initialized = false;
  }


  void on_mouse_move(f32 x, f32 y);
  void on_mouse_button(i32 button, i32 action, f32 x, f32 y);
  void on_key(i32 key, i32 action);
  void on_scroll(f32 x, f32 y);

};

BB_NAMESPACE_END