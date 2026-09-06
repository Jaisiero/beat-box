#include "fragment_finalization.inl"

[[vk::push_constant]] FragmentFinalizePushConstants F;

float fragment_reciprocal(float value)
{
  // Refine the hardware reciprocal before applying inverse mass/inertia. This
  // improves accuracy but does not promise bitwise equivalence with CPU division.
  float estimate = 1.0f / value;
  return fma(estimate, fma(-value, estimate, 1.0f), estimate);
}

float3x3 fragment_inverse(float3x3 m)
{
  // Cofactors in the same order as the previous glm::inverse path.
  precise float det = m[0][0] * (m[1][1]*m[2][2] - m[2][1]*m[1][2])
                    - m[1][0] * (m[0][1]*m[2][2] - m[2][1]*m[0][2])
                    + m[2][0] * (m[0][1]*m[1][2] - m[1][1]*m[0][2]);
  precise float inv_det = fragment_reciprocal(det);
  precise float3x3 result;
  result[0][0] =  (m[1][1]*m[2][2] - m[2][1]*m[1][2]) * inv_det;
  result[1][0] = -(m[1][0]*m[2][2] - m[2][0]*m[1][2]) * inv_det;
  result[2][0] =  (m[1][0]*m[2][1] - m[2][0]*m[1][1]) * inv_det;
  result[0][1] = -(m[0][1]*m[2][2] - m[2][1]*m[0][2]) * inv_det;
  result[1][1] =  (m[0][0]*m[2][2] - m[2][0]*m[0][2]) * inv_det;
  result[2][1] = -(m[0][0]*m[2][1] - m[2][0]*m[0][1]) * inv_det;
  result[0][2] =  (m[0][1]*m[1][2] - m[1][1]*m[0][2]) * inv_det;
  result[1][2] = -(m[0][0]*m[1][2] - m[1][0]*m[0][2]) * inv_det;
  result[2][2] =  (m[0][0]*m[1][1] - m[1][0]*m[0][1]) * inv_det;
  return result;
}

[shader("compute")]
[numthreads(1,1,1)]
void entry_fragment_finalize(uint3 tid : SV_DispatchThreadID)
{
  RigidBody* r = &((RigidBody*)F.bodies_addr)[F.body];
  VoxelShape* shape = &((VoxelShape*)F.shapes_addr)[r->shape_index - 1u];
  VoxelShapeDerived* d = &((VoxelShapeDerived*)F.derived_addr)[r->shape_index - 1u];
  if (d->count == 0u) return;

  r->mass = F.voxel_mass * float(d->count);
  r->inv_mass = fragment_reciprocal(r->mass);
  r->inv_inertia = fragment_inverse(d->unit_inertia * F.voxel_mass);
  shape->grid_origin = -d->com;
  r->minimum = shape->grid_origin;
  r->maximum = shape->grid_origin + float3(shape->dims) * shape->voxel_size;

  precise float3 shift = (d->com + F.crop_off) - F.com_old;
  Quaternion q = F.parent_rot;
  precise float3 ws = (q * Quaternion(shift, 0.0f) * q.conjugate()).v;
  r->position = F.parent_pos + ws;
  precise float3 dv = float3(F.parent_omega.y*ws.z - F.parent_omega.z*ws.y,
                             F.parent_omega.z*ws.x - F.parent_omega.x*ws.z,
                             F.parent_omega.x*ws.y - F.parent_omega.y*ws.x);
  precise float3 velocity = F.parent_vel + dv;
  r->velocity = velocity;
  r->rotation = q.normalize();
  r->omega = F.parent_omega;
  r->prev_velocity = r->velocity;
  r->prev_omega = r->omega;
  r->flags = RigidBodyFlag(uint(r->flags) & ~uint(RigidBodyFlag::SLEEPING));
  r->sleep_timer = 0u;
  // Initial AS construction consumes these instances before the regular updater.
  daxa_BlasInstanceData* instance = &((daxa_BlasInstanceData*)F.instances_addr)[F.body];
  instance->transform = r->get_instance_transform();
}
