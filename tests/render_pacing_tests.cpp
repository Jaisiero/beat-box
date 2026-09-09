#include "render_pacing.hpp"
#include <cstdlib>
#include <cmath>
static void check(bool ok) { if (!ok) std::abort(); }
int main() {
  constexpr double dt = 1.0 / 60.0;
  check(beatbox::render_retry_delay(0.004, 0.007, dt) == 0.0); // fast step: keep high FPS
  check(beatbox::render_retry_delay(0.010, 0.007, dt) == 0.001); // leave a GPU gap, poll in 1 ms
  check(std::abs(beatbox::render_retry_delay(0.020, dt - 0.0002, dt) - 0.0002) < 1e-12);
  check(beatbox::render_retry_delay(0.020, dt, dt) == 0.0); // presentation cannot starve
  check(beatbox::render_retry_delay(0.001, 0.007, dt, 0.010) == 0.001); // heavy history: defer early
  check(beatbox::render_retry_delay(0.001, dt, dt, 0.010) == 0.0); // heavy history cannot starve presentation
  check(beatbox::render_retry_delay(0.001, 0.007, dt, 0.003) == 0.0); // fast history retains high FPS
  check(beatbox::render_retry_delay(1.0, 0.050, dt) == 0.0); // stalled solver still allows rendering
}
