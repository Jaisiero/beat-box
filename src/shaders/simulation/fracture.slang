#pragma once
#include "shared.inl"

// Each prepare/impact invocation owns one manifold slot. No wrapping atomic
// append: simultaneous contacts cannot overwrite another invocation's event.
void emit_fracture_pair(SimConfig* config, RigidBody* a, RigidBody* b,
                       daxa_u32 manifold_index, daxa_f32 impulse,
                       daxa_f32vec3 position, daxa_f32vec3 normal)
{
  if (!(impulse > 0.0f) || !isfinite(impulse)
      || !all(isfinite(position)) || !all(isfinite(normal))) return;
  bool eligible = false;
  [unroll] for (daxa_u32 side = 0u; side < 2u; ++side) {
    RigidBody* body = side == 0u ? a : b;
    eligible = eligible || (body->has_flag(RigidBodyFlag::DYNAMIC)
        && !body->has_flag(RigidBodyFlag::SLEEPING) && body->shape_index != 0u
        && body->fracture_impulse > 0.0f && impulse > body->fracture_impulse);
  }
  if (!eligible || manifold_index >= BB_MAX_COLLISION_COUNT) return;
  FractureEventBuffer* events = (FractureEventBuffer*)(config->fracture_events_addr);
  FractureImpactScratch* scratch = (FractureImpactScratch*)(events->scratch_addr);
  scratch->candidates[manifold_index].body_id = 0u;
  scratch->candidates[manifold_index].impulse = impulse;
  scratch->candidates[manifold_index].position = position;
  scratch->candidates[manifold_index].normal = normal;
}
