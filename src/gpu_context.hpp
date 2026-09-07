#pragma once

#include "defines.hpp"
#include "window_manager.hpp"
#include <iostream>
#include <array>
#include <span>
#include <utility>

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

    // async-queue availability (the async-compute sim path requires >= 1 compute queue)
    std::cout << "[QUEUES] compute=" << device.queue_count(daxa::QueueType::COMPUTE)
              << " transfer=" << device.queue_count(daxa::QueueType::TRANSFER) << std::endl;

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
      .image_usage = daxa::ImageUsageFlagBits::SHADER_STORAGE | daxa::ImageUsageFlagBits::TRANSFER_SRC | daxa::ImageUsageFlagBits::TRANSFER_DST,
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

  // Preserve render -> simulation execution and memory ordering on the GPU.
  // Daxa maps queue timeline waits to ALL_COMMANDS. A wait-only submission
  // orders subsequent COMPUTE_0 work without blocking CPU command recording.
  auto order_simulation_after_rendering() -> void {
    device.submit_commands({
        .queue = daxa::QUEUE_COMPUTE_0,
        .wait_queue_submit_indices = std::array{std::pair{daxa::QUEUE_MAIN,
            device.latest_queue_submit_index(daxa::QUEUE_MAIN)}},
    });
  }

  // Only the compute submission containing the step must complete before its
  // host-visible results are consumed. Shutdown/host resource edits still use
  // synchronize(); this is not a replacement for those lifetime boundaries.
  auto wait_for_simulation() -> void {
    device.wait_on_submit({.queue = daxa::QUEUE_COMPUTE_0,
        .queue_submit_index = device.latest_queue_submit_index(daxa::QUEUE_COMPUTE_0)});
  }

};

BB_NAMESPACE_END
