#include "path_tracing_math.hpp"
#include <cmath>
#include <cstdio>
int main() {
  int failures = 0;
  auto check = [&](bool ok) { if (!ok) ++failures; };
  check(pt_unit_float(0u) == 0.0f);
  check(pt_unit_float(0xffffffffu) < 1.0f);
  check(std::sqrt(1.0f - pt_unit_float(0xffffffffu)) > 0.0f);
  // F9 light is a thin rectangular panel, not a cube. Also test a shifted center.
  daxa_f32vec3 lo(-10.0f, 2.0f, -3.0f), hi(12.0f, 2.4f, 9.0f);
  float total = 0.0f;
  for (unsigned face=0; face<6; ++face) {
    total += pt_face_area(lo, hi, face);
    auto n = pt_face_normal(face);
    check(n.x*n.x+n.y*n.y+n.z*n.z == 1.0f);
    for (float u : {0.0f, 0.5f, pt_unit_float(0xffffffffu)}) {
      for (float v : {0.0f, 0.5f, pt_unit_float(0xffffffffu)}) {
        auto p = pt_face_point(lo, hi, face, u, v);
        check(p.x>=lo.x && p.x<=hi.x && p.y>=lo.y && p.y<=hi.y && p.z>=lo.z && p.z<=hi.z);
        check(face==0 ? p.x==hi.x : face==1 ? p.x==lo.x : face==2 ? p.y==hi.y :
              face==3 ? p.y==lo.y : face==4 ? p.z==hi.z : p.z==lo.z);
      }
    }
  }
  check(std::fabs(total - 555.2f) < 0.001f);
  std::printf("path tracing math failures: %d\n", failures);
  return failures ? 1 : 0;
}
