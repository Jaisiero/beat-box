#include "path_tracing_math.hpp"
#include "shared.inl"
#include <cstring>
#include <cmath>
#include <cstdio>
int main() {
  int failures = 0;
  auto check = [&](bool ok) { if (!ok) ++failures; };
  // Vulkan consumes 3 ROWS of four floats. Compare against an independent
  // quaternion sandwich, including non-identity rotations and translation.
  for (Quaternion q : {Quaternion(0,0,0,1), Quaternion(0,0,0.70710678f,0.70710678f),
                       Quaternion(0.2f,-0.3f,0.4f,0.7f).normalize()})
  {
    RigidBody body = {.rotation = q};
    body.rotation = q;
    body.position = {3,-5,7};
    auto transform = body.get_instance_transform();
    float rows[12];
    std::memcpy(rows, &transform, sizeof(rows));
    for (daxa_f32vec3 point : {daxa_f32vec3(0,0,0), daxa_f32vec3(1,0,0),
                              daxa_f32vec3(0,1,0), daxa_f32vec3(0,0,1), daxa_f32vec3(2,-3,4)})
    {
      auto expected = (q * point * q.conjugate()).v + body.position;
      float actual[3];
      for (unsigned r = 0; r < 3; ++r)
        actual[r] = rows[4*r]*point.x + rows[4*r+1]*point.y + rows[4*r+2]*point.z + rows[4*r+3];
      check(std::fabs(actual[0]-expected.x) < 1e-5f);
      check(std::fabs(actual[1]-expected.y) < 1e-5f);
      check(std::fabs(actual[2]-expected.z) < 1e-5f);
    }
  }
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
