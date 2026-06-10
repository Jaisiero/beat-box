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

    // async-sim <-> render synchronization (see members below)
    sim_done_tsem = device.create_timeline_semaphore({.initial_value = 0, .name = "sim done timeline"});
    sim_signal_pairs[0] = {sim_done_tsem, 0ull};
    sim_wait_pairs[0] = {sim_done_tsem, 0ull};
    sim_signal_span = std::span{sim_signal_pairs};
    sim_wait_span = std::span{sim_wait_pairs};

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

  // ---- async-sim <-> render synchronization ----
  // The sim + TLAS-build task graphs run on QUEUE_COMPUTE_0; the render graph runs on MAIN.
  // The LAST compute submit of a publication (the TLAS build) signals this timeline semaphore
  // with a monotonically increasing value, and the render submit waits the latest published
  // value. daxa::TaskSubmitInfo stores POINTERS to the spans and dereferences them on every
  // execute, so mutating the pair values right before execute() is the per-frame mechanism.
  daxa::TimelineSemaphore sim_done_tsem;
  daxa::u64 sim_timeline_value = 0;
  std::array<std::pair<daxa::TimelineSemaphore, daxa::u64>, 1> sim_signal_pairs = {};
  std::span<std::pair<daxa::TimelineSemaphore, daxa::u64>> sim_signal_span = {};
  std::array<std::pair<daxa::TimelineSemaphore, daxa::u64>, 1> sim_wait_pairs = {};
  std::span<std::pair<daxa::TimelineSemaphore, daxa::u64>> sim_wait_span = {};

  // call right before submitting/executing the LAST compute graph of a sim publication
  // (timeline signal values must be strictly increasing)
  auto advance_sim_timeline() -> void {
    ++sim_timeline_value;
    sim_signal_pairs[0].second = sim_timeline_value;
  }

  // call right before executing the render graph: wait for the latest published sim state
  auto sync_render_to_sim_timeline() -> void {
    sim_wait_pairs[0].second = sim_timeline_value;
  }

};

BB_NAMESPACE_END
