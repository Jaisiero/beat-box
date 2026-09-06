#include "fracture_allocator.inl"

[[vk::push_constant]] FracturePublicationPushConstants F;

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
[numthreads(64,1,1)]
void entry_fragment_finalize(uint3 tid : SV_DispatchThreadID)
{
  if (tid.x>=F.body_count) return;
  FractureAllocationState* state=(FractureAllocationState*)F.allocator_addr;
  FractureBodyEdit edit=state->body_edits[tid.x];
  if (edit.kind==0u) return;
  RigidBody* r=&((RigidBody*)F.bodies_addr)[tid.x];
  state->body_edits[tid.x].kind=0u;
  if (edit.kind==2u)
  {
    r->flags=RigidBodyFlag::NONE;r->shape_index=0u;r->primitive_count=1u;
    r->mass=0.0f;r->inv_mass=0.0f;r->velocity=float3(0);r->omega=float3(0);
    r->position=float3(0,-1000,0);r->minimum=float3(-0.001f);r->maximum=float3(0.001f);
    r->material_index=0u;r->sleep_timer=0u;
    return;
  }
  if (edit.kind==3u) { *r=((RigidBody*)F.spawn_addr)[0];return; }
  FractureParentContext* context=&((FractureParentContext*)F.contexts_addr)[edit.parent_id];
  *r=context->parent;
  r->id=tid.x;r->shape_index=edit.shape_id;r->primitive_count=edit.primitive_count;
  r->island_index=MAX_U32;r->manifold_node_index=MAX_U32;r->active_index=MAX_U32;
  float voxel_mass = context->parent.mass / float(context->parent.primitive_count);
  VoxelShape* shape = &((VoxelShape*)F.shapes_addr)[r->shape_index - 1u];
  VoxelShapeDerived* d = &((VoxelShapeDerived*)F.derived_addr)[r->shape_index - 1u];
  if (d->count == 0u) return;

  r->mass = voxel_mass * float(d->count);
  r->inv_mass = fragment_reciprocal(r->mass);
  r->inv_inertia = fragment_inverse(d->unit_inertia * voxel_mass);
  shape->grid_origin = -d->com;
  r->minimum = shape->grid_origin;
  r->maximum = shape->grid_origin + float3(shape->dims) * shape->voxel_size;

  precise float3 shift = (d->com + edit.crop_off) - (-context->shape.grid_origin);
  Quaternion q = context->parent.rotation;
  precise float3 ws = (q * Quaternion(shift, 0.0f) * q.conjugate()).v;
  r->position = context->parent.position + ws;
  precise float3 dv = float3(context->parent.omega.y*ws.z - context->parent.omega.z*ws.y,
                             context->parent.omega.z*ws.x - context->parent.omega.x*ws.z,
                             context->parent.omega.x*ws.y - context->parent.omega.y*ws.x);
  precise float3 velocity = context->parent.velocity + dv;
  r->velocity = velocity;
  r->rotation = q.normalize();
  r->omega = context->parent.omega;
  r->prev_velocity = r->velocity;
  r->prev_omega = r->omega;
  r->flags = RigidBodyFlag(uint(r->flags) & ~uint(RigidBodyFlag::SLEEPING));
  r->sleep_timer = 0u;
}
