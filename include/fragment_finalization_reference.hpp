#pragma once
#include "fragment_finalization.inl"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <algorithm>

// Previous CPU calculation, retained only as the validation oracle.
inline void fragment_finalize_reference(RigidBody &b, VoxelShape &sh,
    VoxelShapeDerived const &d, FragmentFinalizePushConstants const &f)
{
  if (d.count == 0u) return;
  b.mass = f.voxel_mass * float(d.count);
  b.inv_mass = 1.0f / b.mass;
  glm::mat3 I;
  I[0] = {d.unit_inertia.x.x, d.unit_inertia.x.y, d.unit_inertia.x.z};
  I[1] = {d.unit_inertia.y.x, d.unit_inertia.y.y, d.unit_inertia.y.z};
  I[2] = {d.unit_inertia.z.x, d.unit_inertia.z.y, d.unit_inertia.z.z};
  b.inv_inertia = daxa_mat3_from_glm_mat3(glm::inverse(I * f.voxel_mass));
  glm::vec3 com(d.com.x, d.com.y, d.com.z);
  sh.grid_origin = {-com.x,-com.y,-com.z};
  b.minimum = sh.grid_origin;
  b.maximum = {sh.grid_origin.x + sh.dims.x * sh.voxel_size,
               sh.grid_origin.y + sh.dims.y * sh.voxel_size,
               sh.grid_origin.z + sh.dims.z * sh.voxel_size};
  glm::vec3 shift = (com + glm::vec3(f.crop_off.x,f.crop_off.y,f.crop_off.z)) -
                   glm::vec3(f.com_old.x,f.com_old.y,f.com_old.z);
  Quaternion q = f.parent_rot;
  auto ws = (q * Quaternion(daxa_f32vec3(shift.x,shift.y,shift.z),0.0f) * q.conjugate()).v;
  b.position = f.parent_pos + ws;
  auto dv = glm::cross(glm::vec3(f.parent_omega.x,f.parent_omega.y,f.parent_omega.z), glm::vec3(ws.x,ws.y,ws.z));
  b.velocity = {f.parent_vel.x+dv.x, f.parent_vel.y+dv.y, f.parent_vel.z+dv.z};
  b.rotation = q.normalize();
  b.omega = f.parent_omega;
  b.prev_velocity = b.velocity;
  b.prev_omega = b.omega;
  b.flags = RigidBodyFlag(daxa_u32(b.flags) & ~daxa_u32(RigidBodyFlag::SLEEPING));
  b.sleep_timer = 0;
}

// GPU arithmetic need not reproduce CPU division/contraction bit for bit. Reject
// non-finite results or differences above a tight absolute/relative bound; report
// the actual maximum error so validation cannot silently hide drift.
inline void fragment_verify_finalization(RigidBody const &expected, VoxelShape const &expected_shape,
    RigidBody const &actual, VoxelShape const &actual_shape, daxa_u32 body)
{
  float max_error = 0.0f;
  auto compare = [&](char const *name, auto const &a, auto const &b) {
    static_assert(sizeof(a) % sizeof(float) == 0);
    float aa[sizeof(a)/sizeof(float)], bb[sizeof(b)/sizeof(float)];
    std::memcpy(aa, &a, sizeof(a)); std::memcpy(bb, &b, sizeof(b));
    for (size_t k = 0; k < sizeof(a)/sizeof(float); ++k)
    {
      float const error = std::abs(aa[k] - bb[k]) / std::max({1.0f, std::abs(aa[k]), std::abs(bb[k])});
      if (!std::isfinite(aa[k]) || !std::isfinite(bb[k]) || error > 1.0e-6f)
      {
        std::cerr << "[FRAGMENT-VERIFY] FAILED body=" << body << " field=" << name
                  << " component=" << k << " expected=" << aa[k] << " actual=" << bb[k] << std::endl;
        std::abort();
      }
      max_error = std::max(max_error, error);
    }
  };
  compare("mass", expected.mass, actual.mass);
  compare("inv_mass", expected.inv_mass, actual.inv_mass);
  compare("inertia", expected.inv_inertia, actual.inv_inertia);
  compare("position", expected.position, actual.position);
  compare("rotation", expected.rotation, actual.rotation);
  compare("velocity", expected.velocity, actual.velocity);
  compare("omega", expected.omega, actual.omega);
  compare("prev_velocity", expected.prev_velocity, actual.prev_velocity);
  compare("prev_omega", expected.prev_omega, actual.prev_omega);
  compare("minimum", expected.minimum, actual.minimum);
  compare("maximum", expected.maximum, actual.maximum);
  compare("grid_origin", expected_shape.grid_origin, actual_shape.grid_origin);
  if (expected.flags != actual.flags || expected.sleep_timer != actual.sleep_timer) std::abort();
  std::cout << "[FRAGMENT-VERIFY] body=" << body << " max_scaled_error=" << max_error << std::endl;
}
