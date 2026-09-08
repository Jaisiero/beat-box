#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace beatbox {
// Costs and cadence are independent: 8 ms of GPU work can still run at 60 Hz.
struct PerformanceMetric {
  double current_ms = 0, worst_ms = 0, sum_ms = 0;
  uint64_t samples = 0;
  void add(double ms) {
    if (!std::isfinite(ms) || ms < 0) return;
    worst_ms = std::max(worst_ms, ms); sum_ms += ms; ++samples;
  }
  void reset_average() { current_ms = sum_ms = 0; samples = 0; }
  void observe_peak(double ms) { if (std::isfinite(ms) && ms >= 0) worst_ms = std::max(worst_ms, ms); }
  void refresh() { if (samples) current_ms = sum_ms / double(samples); sum_ms = 0; samples = 0; }
};
struct PerformanceRates {
  double since = 0, sim_hz = 0, render_fps = 0;
  uint64_t steps = 0, frames = 0;
  bool refresh(double now) {
    if (now - since < 0.5) return false;
    sim_hz = double(steps) / (now - since);
    render_fps = double(frames) / (now - since);
    steps = frames = 0; since = now; return true;
  }
};
}
