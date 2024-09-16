#pragma once

#include "defines.hpp"
#include "window.hpp"

BB_NAMESPACE_BEGIN

struct GPUcontext{

  // Daxa instance
  daxa::Instance instance;

  // Daxa device
  daxa::Device device;

  // Swapchain
  daxa::Swapchain swapchain;

  explicit GPUcontext(char const * dev_name, char const * swapchain_name, AppWindow &window) {
    instance = daxa::create_instance({});

    // Create a device
    device = [&]()
    {
        daxa::DeviceInfo2 info = {.name = dev_name};
        // Requested features
        daxa::ImplicitFeatureFlags required_features = daxa::ImplicitFeatureFlagBits::BASIC_RAY_TRACING;

        info = instance.choose_device(required_features, info);
        // Create a device with the requested features
        return instance.create_device_2(info);
    }();

    swapchain = device.create_swapchain({
      // this handle is given by the windowing API
      .native_window = window.get_native_handle(),
      // The platform would also be retrieved from the windowing API,
      // or by hard-coding it depending on the OS.
      .native_window_platform = window.get_native_platform(),
      // Here we can supply a user-defined surface format selection
      // function, to rate formats. If you don't care what format the
      // swapchain images are in, then you can just omit this argument
      // because it defaults to `daxa::default_format_score(...)`
      .surface_format_selector = [](daxa::Format format)
      {
          switch (format)
          {
          case daxa::Format::R8G8B8A8_UINT: return 100;
          default: return daxa::default_format_score(format);
          }
      },
      .present_mode = daxa::PresentMode::FIFO,
      .image_usage = daxa::ImageUsageFlagBits::SHADER_STORAGE,
      .name = swapchain_name,
    });
  }

  ~GPUcontext() {
    synchronize();
    garbage_collector();
  }

  auto synchronize() -> void {
    device.wait_idle();
  }

  auto garbage_collector() -> void {
    device.collect_garbage();
  }

  auto swapchain_resize() -> void {
    swapchain.resize();
  }

  auto swapchain_acquire_next_image() -> daxa::ImageId {
    return swapchain.acquire_next_image();
  }

  auto swapchain_get_extent() -> daxa::Extent2D {
    return swapchain.get_surface_extent();
  }

};

BB_NAMESPACE_END