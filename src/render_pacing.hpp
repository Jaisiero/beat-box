#pragma once
#include <algorithm>

namespace beatbox {
// Give an in-flight step a gap in MAIN submissions after half its real-time
// budget. Keep polling inputs/completion, and stop deferring at one step's
// frame age so a slow solver cannot indefinitely freeze presentation.
inline double render_retry_delay(double step_age_s, double frame_age_s, double step_budget_s) {
  if (step_age_s < step_budget_s * 0.5 || frame_age_s >= step_budget_s) return 0.0;
  return std::min(0.001, step_budget_s - frame_age_s);
}
}
