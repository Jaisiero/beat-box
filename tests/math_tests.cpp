// Host-side unit tests for include/math.hpp (review v3): the Quaternion type is shared
// bit-for-bit between C++ and the Slang shaders (collision SAT, solver integration, the path
// tracer's rotate_vector sandwich), so regressions here corrupt physics AND rendering at once.
// Zero test dependencies: a tiny CHECK macro; the exe returns the failure count (CTest pass = 0).
#include "math.hpp"

#include <cmath>
#include <cstdio>

static int g_failures = 0;
#define CHECK(cond)                                                     \
  do {                                                                  \
    if (!(cond)) {                                                      \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);       \
      ++g_failures;                                                     \
    }                                                                   \
  } while (0)

static bool near_f(daxa_f32 a, daxa_f32 b, daxa_f32 eps = 1e-5f) { return std::fabs(a - b) <= eps; }
static bool near_v(daxa_f32vec3 a, daxa_f32vec3 b, daxa_f32 eps = 1e-5f)
{
  return near_f(a.x, b.x, eps) && near_f(a.y, b.y, eps) && near_f(a.z, b.z, eps);
}
static daxa_f32 dot_v(daxa_f32vec3 a, daxa_f32vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

int main()
{
  // Friction warm start must preserve the force on each physical body under
  // a tangent-basis rotation and under swapping A/B (normal sign reversal).
  {
    auto same = reproject_tangent_impulse({1,0,0}, {0,0,1}, {2,3}, {1,0,0}, {0,0,1}, 1);
    CHECK(near_f(same.x,2) && near_f(same.y,3));
    auto rotated = reproject_tangent_impulse({1,0,0}, {0,0,1}, {2,3}, {0,0,1}, {-1,0,0}, 1);
    CHECK(near_f(rotated.x,3) && near_f(rotated.y,-2));
    auto swapped = reproject_tangent_impulse({1,0,0}, {0,0,1}, {2,3}, {-1,0,0}, {0,0,1}, -1);
    CHECK(near_f(swapped.x,2) && near_f(swapped.y,-3));
    auto back = reproject_tangent_impulse({-1,0,0}, {0,0,1}, swapped, {1,0,0}, {0,0,1}, -1);
    CHECK(near_f(back.x,2) && near_f(back.y,3));
  }
  const daxa_f32 s45 = std::sqrt(0.5f); // sin(45 deg) == cos(45 deg)

  // --- identity ---
  {
    Quaternion id(0.0f, 0.0f, 0.0f, 1.0f);
    CHECK(near_f(id.magnitude(), 1.0f));
    CHECK(near_v(id.get_x_axis(), {1, 0, 0}));
    CHECK(near_v(id.get_y_axis(), {0, 1, 0}));
    CHECK(near_v(id.get_z_axis(), {0, 0, 1}));

    Quaternion q(0.1f, -0.2f, 0.3f, 0.9f);
    Quaternion qi = q * id;
    Quaternion iq = id * q;
    CHECK(near_v(qi.v, q.v) && near_f(qi.w, q.w));
    CHECK(near_v(iq.v, q.v) && near_f(iq.w, q.w));
  }

  // --- normalize / magnitude ---
  {
    Quaternion q(2.0f, -1.0f, 0.5f, 3.0f);
    Quaternion n = q.normalize();
    CHECK(near_f(n.magnitude(), 1.0f));
    // direction preserved
    CHECK(near_f(n.v.x * q.w, n.w * q.v.x) && near_f(n.v.y * q.w, n.w * q.v.y));
  }

  // --- conjugate of a unit quaternion is its inverse: q * conj(q) == identity ---
  {
    Quaternion q = Quaternion(0.3f, -0.4f, 0.2f, 0.8f).normalize();
    Quaternion p = q * q.conjugate();
    CHECK(near_v(p.v, {0, 0, 0}));
    CHECK(near_f(p.w, 1.0f));
  }

  // --- known rotations (right-handed): 90 deg about Z maps X->Y; 180 deg about Z maps X->-X ---
  {
    Quaternion z90(0.0f, 0.0f, s45, s45);
    CHECK(near_v(z90.get_x_axis(), {0, 1, 0}));
    CHECK(near_v(z90.get_y_axis(), {-1, 0, 0}));
    CHECK(near_v(z90.get_z_axis(), {0, 0, 1}));

    Quaternion z180(0.0f, 0.0f, 1.0f, 0.0f);
    CHECK(near_v(z180.get_x_axis(), {-1, 0, 0}));
    CHECK(near_v(z180.get_y_axis(), {0, -1, 0}));

    Quaternion x90(s45, 0.0f, 0.0f, s45); // 90 deg about X maps Y->Z
    CHECK(near_v(x90.get_y_axis(), {0, 0, 1}));
    CHECK(near_v(x90.get_z_axis(), {0, -1, 0}));
  }

  // --- get_invert_*_axis are the transpose (inverse rotation) of get_*_axis ---
  {
    Quaternion q = Quaternion(0.2f, 0.5f, -0.1f, 0.9f).normalize();
    daxa_f32vec3 x = q.get_x_axis(), y = q.get_y_axis(), z = q.get_z_axis();
    daxa_f32vec3 ix = q.get_invert_x_axis(), iy = q.get_invert_y_axis(), iz = q.get_invert_z_axis();
    // rows of R == columns of R^T
    CHECK(near_v(ix, {x.x, y.x, z.x}));
    CHECK(near_v(iy, {x.y, y.y, z.y}));
    CHECK(near_v(iz, {x.z, y.z, z.z}));
  }

  // --- axes of a unit quaternion form an orthonormal, right-handed basis ---
  {
    Quaternion q = Quaternion(-0.6f, 0.2f, 0.4f, 0.7f).normalize();
    daxa_f32vec3 x = q.get_x_axis(), y = q.get_y_axis(), z = q.get_z_axis();
    CHECK(near_f(dot_v(x, x), 1.0f) && near_f(dot_v(y, y), 1.0f) && near_f(dot_v(z, z), 1.0f));
    CHECK(near_f(dot_v(x, y), 0.0f) && near_f(dot_v(y, z), 0.0f) && near_f(dot_v(x, z), 0.0f));
    // right-handed: x cross y == z  (det(R) == +1)
    daxa_f32vec3 xy = {x.y * y.z - x.z * y.y, x.z * y.x - x.x * y.z, x.x * y.y - x.y * y.x};
    CHECK(near_v(xy, z));
  }

  // --- to_matrix rows must equal get_*_axis (two implementations of the same rotation) ---
  {
    Quaternion q = Quaternion(0.35f, -0.15f, 0.25f, 0.85f).normalize();
    daxa_f32mat3x3 m = q.to_matrix();
    // to_matrix returns rows (x,y,z); get_*_axis returns the rotated basis vectors — the
    // rows of R are the inverse axes (R^T columns), i.e. get_invert_*_axis.
    CHECK(near_v(m.x, q.get_invert_x_axis()));
    CHECK(near_v(m.y, q.get_invert_y_axis()));
    CHECK(near_v(m.z, q.get_invert_z_axis()));
  }

  // --- multiplication is associative ---
  {
    Quaternion a = Quaternion(0.1f, 0.2f, 0.3f, 0.9f).normalize();
    Quaternion b = Quaternion(-0.4f, 0.1f, 0.2f, 0.8f).normalize();
    Quaternion c = Quaternion(0.3f, -0.3f, 0.1f, 0.9f).normalize();
    Quaternion ab_c = (a * b) * c;
    Quaternion a_bc = a * (b * c);
    CHECK(near_v(ab_c.v, a_bc.v) && near_f(ab_c.w, a_bc.w));
  }

  // Parallel overlapping segments must return coincident points. The previous
  // shader used +dot(PA-PB,DA)/a and returned points 0.5 units apart here.
  {
    auto uv = closest_segment_parameters({0,0,0}, {1,0,0}, {0.5f,0,0}, {1.5f,0,0});
    CHECK(near_f(uv.x, 0.5f + uv.y));
    auto reverse = closest_segment_parameters({1,0,0}, {0,0,0}, {1.5f,0,0}, {0.5f,0,0});
    CHECK(near_f(1.0f - reverse.x, 1.5f - reverse.y));
    auto cross = closest_segment_parameters({-1,0,0}, {1,0,0}, {0,-1,0}, {0,1,0});
    CHECK(near_f(cross.x, 0.5f) && near_f(cross.y, 0.5f));
    auto separated = closest_segment_parameters({0,0,0}, {1,0,0}, {2,1,0}, {3,1,0});
    CHECK(near_f(separated.x, 1.0f) && near_f(separated.y, 0.0f));
    auto point = closest_segment_parameters({0.5f,0,0}, {0.5f,0,0}, {0,0,0}, {1,0,0});
    CHECK(near_f(point.x, 0.0f) && near_f(point.y, 0.5f));
    auto points = closest_segment_parameters({0,0,0}, {0,0,0}, {1,0,0}, {1,0,0});
    CHECK(near_f(points.x, 0.0f) && near_f(points.y, 0.0f));
  }

  if (g_failures == 0) { std::printf("math_tests: ALL PASSED\n"); }
  else                 { std::printf("math_tests: %d FAILURE(S)\n", g_failures); }
  return g_failures;
}
