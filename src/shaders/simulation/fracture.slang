#pragma once
#include "shared.inl"

// Material thresholds use a stopping impulse (kg*m/s), independent of solver dual units.
void emit_fracture_pair(SimConfig* config, RigidBody* a, RigidBody* b,
                       daxa_f32 impulse, daxa_f32vec3 position, daxa_f32vec3 normal)
{
  if (!(impulse > 0.0f) || !isfinite(impulse)) return;
  FractureEventBuffer* events = (FractureEventBuffer*)(config->fracture_events_addr);
  [unroll]
  for (daxa_u32 side = 0u; side < 2u; ++side) {
    RigidBody* body = side == 0u ? a : b;
    if (!body->has_flag(RigidBodyFlag::DYNAMIC) || body->has_flag(RigidBodyFlag::SLEEPING)
        || body->shape_index == 0u || body->fracture_impulse <= 0.0f
        || impulse <= body->fracture_impulse) continue;
    daxa_u32 serial;
    InterlockedAdd(events->serial, 1u, serial);
    daxa_u32 slot = serial % BB_MAX_FRACTURE_EVENTS;
    events->events[slot].body_id = body->id;
    events->events[slot].impulse = impulse;
    events->events[slot].position = position;
    events->events[slot].normal = normal;
  }
}
