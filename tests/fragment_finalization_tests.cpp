#include "fragment_finalization_reference.hpp"
#include <iostream>

int main()
{
  int failures = 0;
  auto near = [&](float actual, float expected) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-5f)
    {
      std::cerr << "Expected " << expected << ", got " << actual << '\n';
      ++failures;
    }
  };
  // Eight half-unit voxels form a unit cube. At two mass units per voxel,
  // mass=16 and each principal moment is m*(1^2+1^2)/12 = 8/3.
  VoxelShapeDerived d{};
  d.count = 8;
  d.com = {0.5f, 0.5f, 0.5f};
  d.unit_inertia = daxa_mat3_from_glm_mat3(glm::mat3(4.0f / 3.0f));
  VoxelShape shape{};
  shape.dims = {2, 2, 2};
  shape.voxel_size = 0.5f;
  RigidBody body{.rotation = Quaternion(0, 0, 0, 1)};
  body.id = 42;
  body.flags = RigidBodyFlag(daxa_u32(RigidBodyFlag::DYNAMIC) | daxa_u32(RigidBodyFlag::SLEEPING));
  body.sleep_timer = 123;
  FragmentFinalizePushConstants f{.parent_rot = Quaternion(0, 0, 0, 1)};
  f.voxel_mass = 2.0f;
  f.com_old = {1.0f, 0.5f, 0.5f};
  f.parent_pos = {10.0f, 20.0f, 30.0f};
  float const h = std::sqrt(0.5f);
  f.parent_rot = Quaternion(0.0f, 0.0f, h, h); // +90 degrees about Z.
  f.parent_vel = {1.0f, 2.0f, 3.0f};
  f.parent_omega = {0.0f, 0.0f, 4.0f};
  fragment_finalize_reference(body, shape, d, f);
  near(body.mass, 16.0f);
  near(body.inv_mass, 1.0f / 16.0f);
  near(body.inv_inertia.x.x, 3.0f / 8.0f);
  near(body.inv_inertia.y.y, 3.0f / 8.0f);
  near(body.inv_inertia.z.z, 3.0f / 8.0f);
  near(body.inv_inertia.x.y, 0.0f);
  near(body.position.x, 10.0f);
  near(body.position.y, 19.5f);
  near(body.position.z, 30.0f);
  // COM moved half a unit in -Y: omega x delta_position adds +2 in X.
  near(body.velocity.x, 3.0f);
  near(body.velocity.y, 2.0f);
  near(body.velocity.z, 3.0f);
  near(body.prev_velocity.x, body.velocity.x);
  near(body.prev_omega.z, 4.0f);
  near(shape.grid_origin.x, -0.5f);
  near(body.minimum.y, -0.5f);
  near(body.maximum.z, 0.5f);
  if (body.id != 42 || body.sleep_timer != 0 || (body.flags & RigidBodyFlag::SLEEPING) != RigidBodyFlag::NONE) ++failures;
  // Empty derived records must leave the body and shape untouched.
  d.count = 0;
  auto before = body;
  auto shape_before = shape;
  fragment_finalize_reference(body, shape, d, f);
  if (std::memcmp(&before, &body, sizeof(body)) || std::memcmp(&shape_before, &shape, sizeof(shape))) ++failures;
  std::cout << failures << " fragment finalization reference failures\n";
  return failures ? 1 : 0;
}
