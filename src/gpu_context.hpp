#pragma once

#include "defines.hpp"
#include "window_manager.hpp"

BB_NAMESPACE_BEGIN

struct GPUcontext{

  // Daxa instance
  daxa::Instance instance;

  // Daxa device
  daxa::Device device;

  // Swapchain
  daxa::Swapchain swapchain;

  explicit GPUcontext(char const * dev_name, char const * swapchain_name, WindowManager &window) {
    instance = daxa::create_instance({});

    // Create a device
    device = [&]()
    {
        daxa::DeviceInfo2 info = {.name = dev_name};
        // Requested features
        daxa::ImplicitFeatureFlags required_features =
            daxa::ImplicitFeatureFlagBits::BASIC_RAY_TRACING |
            daxa::ImplicitFeatureFlagBits::RAY_TRACING_PIPELINE |
            daxa::ImplicitFeatureFlagBits::SWAPCHAIN;

        info = instance.choose_device(required_features, info);
        // Create a device with the requested features
        return instance.create_device_2(info);
    }();

    auto native_window_info = window.get_native_window_info();
    auto preferred_surface_formats = std::array{
        daxa::SurfaceFormat{.format = daxa::Format::R8G8B8A8_UNORM},
        daxa::SurfaceFormat{.format = daxa::Format::B8G8R8A8_UNORM},
    };
    auto surface_format = device.choose_swapchain_surface_format({
        .native_window_info = native_window_info,
        .preferred_formats = preferred_surface_formats,
    });

    swapchain = device.create_swapchain({
      .native_window_info = native_window_info,
      .surface_format = surface_format,
      .present_mode = daxa::PresentMode::FIFO,
      .image_usage = daxa::ImageUsageFlagBits::SHADER_STORAGE | daxa::ImageUsageFlagBits::TRANSFER_SRC,
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
