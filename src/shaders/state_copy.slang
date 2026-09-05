#pragma once
#include "shared.inl"

void copy_rigid_body_fields(RigidBody* dst, RigidBody* src)
{
  dst->id = src->id;
  dst->flags = src->flags;
  dst->island_index = src->island_index;
  dst->active_index = src->active_index;
  dst->manifold_node_index = src->manifold_node_index;
  dst->sleep_timer = src->sleep_timer;
  dst->material_index = src->material_index;
#if defined(BB_DEBUG)
  dst->face_collided = src->face_collided;
#endif // BB_DEBUG
  dst->primitive_count = src->primitive_count;
  dst->primitive_offset = src->primitive_offset;
  dst->shape_index = src->shape_index;
  dst->position = src->position;
  dst->rotation.v = src->rotation.v;
  dst->rotation.w = src->rotation.w;
  dst->minimum = src->minimum;
  dst->maximum = src->maximum;
  dst->mass = src->mass;
  dst->inv_mass = src->inv_mass;
  dst->velocity = src->velocity;
  dst->omega = src->omega;
  dst->prev_velocity = src->prev_velocity;
  dst->prev_omega = src->prev_omega; // E2: keep step-entry omega across the row reorder (next to prev_velocity)
  dst->inv_inertia[0][0] = src->inv_inertia[0][0];
  dst->inv_inertia[0][1] = src->inv_inertia[0][1];
  dst->inv_inertia[0][2] = src->inv_inertia[0][2];
  dst->inv_inertia[1][0] = src->inv_inertia[1][0];
  dst->inv_inertia[1][1] = src->inv_inertia[1][1];
  dst->inv_inertia[1][2] = src->inv_inertia[1][2];
  dst->inv_inertia[2][0] = src->inv_inertia[2][0];
  dst->inv_inertia[2][1] = src->inv_inertia[2][1];
  dst->inv_inertia[2][2] = src->inv_inertia[2][2];
  dst->restitution = src->restitution;
  dst->friction = src->friction;
  dst->fracture_impulse = src->fracture_impulse;   // fracture threshold rides the reorder too
  dst->fracture_material = src->fracture_material;  // ... and its material kind (fragment pattern)
}

// review v2 #5: Contact and Manifold are POD, so a whole-struct assignment copies EVERY field
// (verified: the old field-by-field bodies enumerated all of Contact/Manifold incl. all 4
// FeaturePair ids and contacts[]). Collapsing to *dst = src removes ~80 lines whose real hazard
// was DRIFT — a field added to the struct but forgotten here silently drops from the warm-start /
// pause-flush "previous" buffers (the documented explosion class). Whole-struct copy can never
// drift. Bitwise-identical, proven via the determinism harness (towers stay bit-for-bit).
void copy_contact_fields(Contact* dst, Contact* src) { *dst = *src; }

void write_contact_fields_from_value(Contact* dst, Contact src) { *dst = src; }

void copy_manifold_fields(Manifold* dst, Manifold* src) { *dst = *src; }

void write_manifold_fields_from_value(Manifold* dst, Manifold src) { *dst = src; }
