#pragma once
#include "defines.hpp"
#include "performance_metrics.hpp"

BB_NAMESPACE_BEGIN
// A completed submit, not query availability alone, authorizes reading/reusing
// a slot: before a queued GPU reset executes, old availability can still be set.
struct GpuPerformanceTimer {
  static constexpr u32 SLOT_COUNT = 16;
  struct Slot { u64 submit = 0, epoch = 0; bool pending = false; };
  std::array<Slot, SLOT_COUNT> slots = {};
  daxa::TimelineQueryPool pool = {};
  PerformanceMetric metric;
  u64 epoch = 0;
  u32 active = SLOT_COUNT;
  void create(daxa::Device &device, char const *name) {
    pool = device.create_timeline_query_pool({.query_count = SLOT_COUNT * 2, .name = name});
  }
  void collect(daxa::Device &device) {
    auto const oldest = device.oldest_pending_submit_index();
    for (u32 i = 0; i < SLOT_COUNT; ++i) if (slots[i].pending && slots[i].submit < oldest) {
      auto q = pool.get_query_results(i * 2, 2);
      if (!q[1] || !q[3]) continue;
      if (slots[i].epoch == epoch)
        metric.add(double(q[2] - q[0]) * device.properties().limits.timestamp_period / 1e6);
      slots[i].pending = false;
    }
  }
  void prepare(daxa::Device &device) {
    collect(device); active = SLOT_COUNT;
    for (u32 i = 0; i < SLOT_COUNT; ++i) if (!slots[i].pending) { active = i; break; }
  }
  void begin(daxa::CommandRecorder &recorder) {
    if (active == SLOT_COUNT) return;
    recorder.reset_timestamps({.query_pool = pool, .start_index = active * 2, .count = 2});
    recorder.write_timestamp({.query_pool = pool, .pipeline_stage = daxa::PipelineStageFlagBits::ALL_COMMANDS, .query_index = active * 2});
  }
  void end(daxa::CommandRecorder &recorder) {
    if (active != SLOT_COUNT)
      recorder.write_timestamp({.query_pool = pool, .pipeline_stage = daxa::PipelineStageFlagBits::ALL_COMMANDS, .query_index = active * 2 + 1});
  }
  void submitted(daxa::Device &device, daxa::Queue queue) {
    if (active != SLOT_COUNT) slots[active] = {device.latest_queue_submit_index(queue), epoch, true};
  }
  void reset() { ++epoch; metric = {}; }
  void destroy() { pool = {}; }
};
BB_NAMESPACE_END
