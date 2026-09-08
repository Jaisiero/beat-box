#pragma once
#include <algorithm>

namespace beatbox {
// Give an in-flight step a gap in MAIN submissions when recent GPU work or
// the current step age reaches half its budget. Stop deferring at one step's
// frame age so a slow solver cannot indefinitely freeze presentation.
inline double render_retry_delay(double step_age_s, double frame_age_s, double step_budget_s,
                                 double recent_step_s = 0.0) {
  // A recent heavy workload needs the gap from submission, before another
  // path-traced frame occupies the GPU. Fast workloads retain the age trigger.
  bool const heavy_workload = recent_step_s >= step_budget_s * 0.5;
  if ((!heavy_workload && step_age_s < step_budget_s * 0.5) || frame_age_s >= step_budget_s) return 0.0;
  return std::min(0.001, step_budget_s - frame_age_s);
}
}
