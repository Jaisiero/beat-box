#include "performance_metrics.hpp"
#include <cstdlib>
#include <limits>
#include <cmath>
static void check(bool ok) { if (!ok) std::abort(); }
int main() {
  beatbox::PerformanceMetric cost;
  cost.add(8); cost.add(12); cost.add(-1); cost.add(std::numeric_limits<double>::quiet_NaN());
  cost.refresh(); check(cost.current_ms == 10 && cost.worst_ms == 12);
  cost.add(6); cost.refresh(); check(cost.current_ms == 6 && cost.worst_ms == 12);
  cost.refresh(); check(cost.current_ms == 6); // paused: retain last measured cost
  beatbox::PerformanceRates rate;
  rate.steps = 30; rate.frames = 15;
  check(!rate.refresh(0.25)); check(rate.refresh(0.5));
  check(rate.sim_hz == 60 && rate.render_fps == 30); // independent cadences
  rate.frames = 30; check(rate.refresh(1)); check(rate.sim_hz == 0 && rate.render_fps == 60);
  cost = {}; check(cost.worst_ms == 0 && cost.current_ms == 0);
}
