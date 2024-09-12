#pragma once

#include "window.hpp"
#include <daxa/daxa.hpp>
// For things like `u32`. Not necessary of course.
using namespace daxa::types;


struct GPUcontext{

  // Daxa instance
  daxa::Instance instance;

  // Daxa device
  daxa::Device device;

  // Swapchain
  daxa::Swapchain swapchain;

  explicit GPUcontext(AppWindow &window) {
    instance = daxa::create_instance({});

    // Create a device
    device = [&]()
    {
        daxa::DeviceInfo2 info = {.name = "default device"};
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
      .image_usage = daxa::ImageUsageFlagBits::TRANSFER_DST,
      .name = "Swapchain",
    });
  }
};