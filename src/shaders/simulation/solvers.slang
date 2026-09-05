// Static bodies are shared across contacts of one color: even a zero-delta store
// is a data race. Only dynamic bodies may receive solver writes.
#include "shared.inl"

// Sleeping bodies do not integrate. They must also have zero solver mass and
// receive no impulses until the sleep/wake pass activates them.
bool solver_body_moves(RigidBody* body) {
  return body->has_flag(RigidBodyFlag::DYNAMIC) && !body->has_flag(RigidBodyFlag::SLEEPING);
}
daxa_f32 solver_inverse_mass(RigidBody* body) {
  return solver_body_moves(body) ? body->inv_mass : 0.0f;
}
daxa_f32mat3x3 solver_inverse_inertia(RigidBody* body) {
  return solver_body_moves(body) ? body->get_transformed_inertia() : daxa_f32mat3x3(0.0f);
}

struct PGS {
  static void pre_step(Manifold* manifold, daxa_f32 inv_dt, RigidBody* body_a,  RigidBody* body_b, daxa_b32 accumulate_impulses, daxa_b32 enable_friction, daxa_f32 k_allowed_penetration = PENETRATION_FACTOR, daxa_f32 k_bias_factor = BIAS_FACTOR) {

    for(daxa_u32 i = 0u; i < manifold->contact_count; i++) {

      daxa_f32vec3 r1 = manifold->contacts[i].position - body_a->position;
      daxa_f32vec3 r2 = manifold->contacts[i].position - body_b->position;

      // Precompute normal mass, tangent mass, and bias
      daxa_f32vec3 rn1 = cross(r1, manifold->normal);
      daxa_f32vec3 rn2 = cross(r2, manifold->normal);
      daxa_f32 k_normal = solver_inverse_mass(body_a) + solver_inverse_mass(body_b);
      k_normal += dot(rn1, mul(solver_inverse_inertia(body_a), rn1));
      k_normal += dot(rn2, mul(solver_inverse_inertia(body_b), rn2));
      manifold->contacts[i].normal_mass = k_normal == 0.0 ? 0.0f : 1.0f / k_normal;

      for(daxa_u32 j = 0u; j < 2; j++) {
        daxa_f32vec3 rt1 = cross(r1, manifold->tangent_vectors[j]);
        daxa_f32vec3 rt2 = cross(r2, manifold->tangent_vectors[j]);
        daxa_f32 k_tangent = solver_inverse_mass(body_a) + solver_inverse_mass(body_b);
        k_tangent += dot(rt1, mul(solver_inverse_inertia(body_a), rt1));
        k_tangent += dot(rt2, mul(solver_inverse_inertia(body_b), rt2));
        manifold->contacts[i].tangent_mass[j] = k_tangent == 0.0 ? 0.0 : 1.0f / k_tangent;
      }

      manifold->contacts[i].bias_factor = -k_bias_factor * inv_dt * min(manifold->contacts[i].penetration + k_allowed_penetration, 0.0f);

      // accumlate impulse
      if(accumulate_impulses) {
        // Apply normal + friction impulse
        daxa_f32vec3 P = manifold->normal * manifold->contacts[i].normal_impulse;
        // printf("presolver - contact normal_impulse: %f, tangent_impulse[0]: %f, tangent_impulse[1]: %f, contacts[%d].warm_start: %d\n", contacts[i].normal_impulse, contacts[i].tangent_impulse[0], contacts[i].tangent_impulse[1], i, contacts[i].warm_start);

        if(enable_friction) {
          for(daxa_u32 j = 0u; j < 2; j++) {
            P += manifold->tangent_vectors[j] * manifold->contacts[i].tangent_impulse[j];
          }
        }

        if (solver_body_moves(body_a)) {
          body_a->velocity -= P * solver_inverse_mass(body_a);
          body_a->omega -= mul(solver_inverse_inertia(body_a), cross(r1, P));
        }

        if (solver_body_moves(body_b)) {
          body_b->velocity += P * solver_inverse_mass(body_b);
          body_b->omega += mul(solver_inverse_inertia(body_b), cross(r2, P));
        }
      }

      // TODO: Revisit this
      // // Apply restitution
      // daxa_f32 relative_velocity = dot(body_b->velocity + cross(body_b->omega, r2) - body_a->velocity - cross(body_a->omega, r1), manifold->normal);

      // if(relative_velocity < -1.0f) {
      //   daxa_f32 restitution = max(body_a->restitution, body_b->restitution);
      //   manifold->contacts[i].bias += -restitution * relative_velocity;
      // }
    }

#if defined(BB_DEBUG)
    // printf("pre_step %lu - %lu - velocity: %f, %f, %f, omega: %f, %f, %f\n", frame_count, body_b->frame_count, body_b->velocity.x, body_b->velocity.y, body_b->velocity.z, body_b->omega.x, body_b->omega.y, body_b->omega.z);
#endif // BB_DEBUG
  }

  static void solve(Manifold* manifold, RigidBody* body_a,  RigidBody* body_b, daxa_b32 accumulate_impulses, daxa_b32 enable_friction) {

    daxa_f32 friction = sqrt(body_a->friction * body_b->friction);
    for(daxa_u32 c = 0u; c < manifold->contact_count; ++c)
    {
      daxa_f32vec3 collision_point = manifold->contacts[c].position;

      // Calculate relative velocity
      daxa_f32vec3 r1_to_point = collision_point - body_a->position;
      daxa_f32vec3 r2_to_point = collision_point - body_b->position;

      daxa_f32vec3 relative_velocity = body_b->velocity + cross(body_b->omega, r2_to_point) - body_a->velocity - cross(body_a->omega, r1_to_point);

      // Friction
      if(enable_friction) {
          for(daxa_u32 j = 0u; j < 2u; ++j)
          {
            daxa_f32 tangent_velocity = dot(relative_velocity, manifold->tangent_vectors[j]);
            daxa_f32 tangent_impulse = manifold->contacts[c].tangent_mass[j] * -tangent_velocity;

            daxa_f32 max_friction_impulse = friction * manifold->contacts[c].normal_impulse;

            // Clamp the accumulated impulse
            if(accumulate_impulses) {
              daxa_f32 old_impulse = manifold->contacts[c].tangent_impulse[j];
              manifold->contacts[c].tangent_impulse[j] = clamp(old_impulse + tangent_impulse, -max_friction_impulse, max_friction_impulse);
              tangent_impulse = manifold->contacts[c].tangent_impulse[j] - old_impulse;
            } else {
              tangent_impulse = clamp(tangent_impulse, -max_friction_impulse, max_friction_impulse);
            }

            // Apply contact impulse
            daxa_f32vec3 tangent_impulse_vector = manifold->tangent_vectors[j] * tangent_impulse;

            if (solver_body_moves(body_a)) {
              body_a->velocity -= tangent_impulse_vector * solver_inverse_mass(body_a);
              body_a->omega -= mul(solver_inverse_inertia(body_a), cross(r1_to_point, tangent_impulse_vector));
            }

            if (solver_body_moves(body_b)) {
              body_b->velocity += tangent_impulse_vector * solver_inverse_mass(body_b);
              body_b->omega += mul(solver_inverse_inertia(body_b), cross(r2_to_point, tangent_impulse_vector));
            }
          }
      }

      relative_velocity = body_b->velocity + cross(body_b->omega, r2_to_point) - body_a->velocity - cross(body_a->omega, r1_to_point);

      // Calculate normal impulse
      daxa_f32 normal_velocity = dot(relative_velocity, manifold->normal);

      daxa_f32 normal_impulse = manifold->contacts[c].normal_mass * (-normal_velocity + manifold->contacts[c].bias_factor);

      // Clamp the accumulated impulse
      if(accumulate_impulses) {
        daxa_f32 old_normal_impulse = manifold->contacts[c].normal_impulse;
        manifold->contacts[c].normal_impulse = max(old_normal_impulse + normal_impulse, 0.0f);
        normal_impulse = manifold->contacts[c].normal_impulse - old_normal_impulse;
        // printf("normal_impulse: %f = contact normal_impulse: %f - old_normal_impulse: %f, normal_mass: %f, normal_velocity: %f, bias: %f\n", normal_impulse, contacts[c].normal_impulse, old_normal_impulse, contacts[c].normal_mass, normal_velocity, contacts[c].bias);
      } else {
        normal_impulse = max(normal_impulse, 0.0f);
      }

      // Apply contact impulse
      daxa_f32vec3 normal_impulse_vector = manifold->normal * normal_impulse;

      if (solver_body_moves(body_a)) {
        body_a->velocity -= normal_impulse_vector * solver_inverse_mass(body_a);
        body_a->omega -= mul(solver_inverse_inertia(body_a), cross(r1_to_point, normal_impulse_vector));
      }

      if (solver_body_moves(body_b)) {
        body_b->velocity += normal_impulse_vector * solver_inverse_mass(body_b);
        body_b->omega += mul(solver_inverse_inertia(body_b), cross(r2_to_point, normal_impulse_vector));
      }

    }

#if defined(BB_DEBUG)
    // printf("solve %lu - %lu - velocity: %f, %f, %f, omega: %f, %f, %f\n", frame_count, body_b->frame_count, body_b->velocity.x, body_b->velocity.y, body_b->velocity.z, body_b->omega.x, body_b->omega.y, body_b->omega.z);
#endif // BB_DEBUG
  }
}



struct PGS_SOFT {
  static void pre_step(Manifold* manifold, daxa_f32 inv_dt, RigidBody* body_a, RigidBody* body_b, daxa_b32 accumulate_impulses, daxa_b32 enable_friction, daxa_f32 h, daxa_f32 hertz) {

    for(daxa_u32 i = 0u; i < manifold->contact_count; i++) {

      daxa_f32vec3 r1 = manifold->contacts[i].position - manifold->anchor_a;
      daxa_f32vec3 r2 = manifold->contacts[i].position - manifold->anchor_b;

      // Precompute normal mass, tangent mass, and bias
      daxa_f32vec3 rn1 = cross(r1, manifold->normal);
      daxa_f32vec3 rn2 = cross(r2, manifold->normal);
      daxa_f32 k_normal = solver_inverse_mass(body_a) + solver_inverse_mass(body_b);
      k_normal += dot(rn1, mul(solver_inverse_inertia(body_a), rn1));
      k_normal += dot(rn2, mul(solver_inverse_inertia(body_b), rn2));
      manifold->contacts[i].normal_mass = k_normal == 0.0 ? 0.0f : 1.0f / k_normal;

      for(daxa_u32 j = 0u; j < 2; j++) {
        daxa_f32vec3 rt1 = cross(r1, manifold->tangent_vectors[j]);
        daxa_f32vec3 rt2 = cross(r2, manifold->tangent_vectors[j]);
        daxa_f32 k_tangent = solver_inverse_mass(body_a) + solver_inverse_mass(body_b);
        k_tangent += dot(rt1, mul(solver_inverse_inertia(body_a), rt1));
        k_tangent += dot(rt2, mul(solver_inverse_inertia(body_b), rt2));
        manifold->contacts[i].tangent_mass[j] = k_tangent == 0.0 ? 0.0 : 1.0f / k_tangent;
      }

      daxa_f32 contact_hertz = (solver_inverse_mass(body_a) == 0.0f || solver_inverse_mass(body_b) == 0.0f) ? 2.0f * hertz : hertz;

      // soft contact
			// should use the substep not the full time step
			const daxa_f32 zeta = 10.0f;
			const daxa_f32 omega = 2.0f * _PI * contact_hertz;
			const daxa_f32 c = h * omega * (2.0f * zeta + h * omega);
			manifold->contacts[i].bias_factor = omega / (2.0f * zeta + h * omega);
			manifold->contacts[i].impulse_coefficient = 1.0f / (1.0f + c);
			manifold->contacts[i].mass_coefficient = c * manifold->contacts[i].impulse_coefficient;

      // accumlate impulse
      if(accumulate_impulses) {
        // Apply normal + friction impulse
        daxa_f32vec3 P = manifold->normal * manifold->contacts[i].normal_impulse;

        if(enable_friction) {
          for(daxa_u32 j = 0u; j < 2; j++) {
            P += manifold->tangent_vectors[j] * manifold->contacts[i].tangent_impulse[j];
          }
        }

        if (solver_body_moves(body_a)) {
          body_a->velocity -= P * solver_inverse_mass(body_a);
          body_a->omega -= mul(solver_inverse_inertia(body_a), cross(r1, P));
        }

        if (solver_body_moves(body_b)) {
          body_b->velocity += P * solver_inverse_mass(body_b);
          body_b->omega += mul(solver_inverse_inertia(body_b), cross(r2, P));
        }
      }
    }
  }

  
  static void solve(Manifold* manifold, RigidBody* body_a, RigidBody* body_b, daxa_b32 accumulate_impulses, daxa_b32 enable_friction, daxa_b32 use_bias, daxa_f32 inv_h) {

    daxa_f32 friction = sqrt(body_a->friction * body_b->friction);
    for(daxa_u32 c = 0u; c < manifold->contact_count; ++c)
    {
      daxa_f32vec3 collision_point = manifold->contacts[c].position;

      // Calculate relative velocity
      daxa_f32vec3 r1_to_point = collision_point - manifold->anchor_a;
      daxa_f32vec3 r2_to_point = collision_point - manifold->anchor_b;

      daxa_f32vec3 relative_velocity = body_b->velocity + cross(body_b->omega, r2_to_point) - body_a->velocity - cross(body_a->omega, r1_to_point);

      // Friction
      if(enable_friction) {
          for(daxa_u32 j = 0u; j < 2u; ++j)
          {
            daxa_f32 tangent_velocity = dot(relative_velocity, manifold->tangent_vectors[j]);
            daxa_f32 tangent_impulse = manifold->contacts[c].tangent_mass[j] * -tangent_velocity;

            daxa_f32 max_friction_impulse = friction * manifold->contacts[c].normal_impulse;

            // Clamp the accumulated impulse
            if(accumulate_impulses) {
              daxa_f32 old_impulse = manifold->contacts[c].tangent_impulse[j];
              manifold->contacts[c].tangent_impulse[j] = clamp(old_impulse + tangent_impulse, -max_friction_impulse, max_friction_impulse);
              tangent_impulse = manifold->contacts[c].tangent_impulse[j] - old_impulse;
            } else {
              tangent_impulse = clamp(tangent_impulse, -max_friction_impulse, max_friction_impulse);
            }

            // Apply contact impulse
            daxa_f32vec3 tangent_impulse_vector = manifold->tangent_vectors[j] * tangent_impulse;

            if (solver_body_moves(body_a)) {
              body_a->velocity -= tangent_impulse_vector * solver_inverse_mass(body_a);
              body_a->omega -= mul(solver_inverse_inertia(body_a), cross(r1_to_point, tangent_impulse_vector));
            }

            if (solver_body_moves(body_b)) {
              body_b->velocity += tangent_impulse_vector * solver_inverse_mass(body_b);
              body_b->omega += mul(solver_inverse_inertia(body_b), cross(r2_to_point, tangent_impulse_vector));
            }
          }
      }

      relative_velocity = body_b->velocity + cross(body_b->omega, r2_to_point) - body_a->velocity - cross(body_a->omega, r1_to_point);

      // Calculate normal impulse
      daxa_f32 normal_velocity = dot(relative_velocity, manifold->normal);
      daxa_f32 bias = 0.0f;
      daxa_f32 mass_scale = 1.0f;
      daxa_f32 impulse_scale = 0.0f;

      if(manifold->contacts[c].penetration > 0.0f) {
        bias = manifold->contacts[c].penetration * inv_h;
      } else if(use_bias) {
        bias = max(manifold->contacts[c].bias_factor * manifold->contacts[c].penetration, -4.0f);
        mass_scale = manifold->contacts[c].mass_coefficient;
        impulse_scale = manifold->contacts[c].impulse_coefficient;
      }

      daxa_f32 impulse = -manifold->contacts[c].normal_mass * mass_scale * (normal_velocity + bias) - impulse_scale * manifold->contacts[c].normal_impulse;

      // Clamp the accumulated impulse
      if(accumulate_impulses) {
        daxa_f32 old_normal_impulse = manifold->contacts[c].normal_impulse;
        manifold->contacts[c].normal_impulse = max(old_normal_impulse + impulse, 0.0f);
        impulse = manifold->contacts[c].normal_impulse - old_normal_impulse;
      } else {
        impulse = max(impulse, 0.0f);
      }

      // Apply contact impulse
      daxa_f32vec3 normal_impulse_vector = manifold->normal * impulse;

      if (solver_body_moves(body_a)) {
        body_a->velocity -= normal_impulse_vector * solver_inverse_mass(body_a);
        body_a->omega -= mul(solver_inverse_inertia(body_a), cross(r1_to_point, normal_impulse_vector));
      }

      if (solver_body_moves(body_b)) {
        body_b->velocity += normal_impulse_vector * solver_inverse_mass(body_b);
        body_b->omega += mul(solver_inverse_inertia(body_b), cross(r2_to_point, normal_impulse_vector));
      }

    }
  }
};

// TGS_SOFT (Box2D v3 / solver2d): the soft-constraint solve of PGS_SOFT, but driven by sub-stepping.
// The defining difference is that the contact SEPARATION is re-derived from the body poses every
// sub-step (the "temporal" / TGS part) instead of read from a static narrow-phase penetration. To
// do that each contact stores its point in each body's LOCAL frame at prepare time (anchor_a/b) plus
// the base separation; the live separation each sub-step is base_sep + dot(n, worldB - worldA). The
// soft coefficients are computed with the SUB-step h. Reuses Contact's existing impulse/coeff fields.
struct TGS_SOFT {
  // local-frame contact anchor on each body (recomputed at prepare from the current pose)
  static daxa_f32vec3 anchor_local(RigidBody* body, daxa_f32vec3 world_point) {
    return body->rotate_vector_inverse(world_point - body->position);
  }

  // Prepare runs ONCE per frame (masses, soft coefficients at the sub-step h, local anchors).
  // It deliberately does NOT warm start -- that is a separate stage inside the sub-step loop,
  // exactly as in Box2D v3 (b2PrepareContactsTask once, b2WarmStartContactsTask per sub-step).
  static void pre_step(Manifold* manifold, RigidBody* body_a, RigidBody* body_b,
                       daxa_f32 h, daxa_f32 hertz) {
    for (daxa_u32 i = 0u; i < manifold->contact_count; i++) {
      // store the contact point in each body's LOCAL frame (so the separation tracks the pose) and
      // its current world arm for the mass terms (Box2D uses the prepare arms across the sub-steps)
      daxa_f32vec3 cp = manifold->contacts[i].position;
      manifold->contacts[i].anchor_a = anchor_local(body_a, cp);
      manifold->contacts[i].anchor_b = anchor_local(body_b, cp);
      daxa_f32vec3 r1 = cp - body_a->position;
      daxa_f32vec3 r2 = cp - body_b->position;

      daxa_f32vec3 rn1 = cross(r1, manifold->normal);
      daxa_f32vec3 rn2 = cross(r2, manifold->normal);
      daxa_f32 k_normal = solver_inverse_mass(body_a) + solver_inverse_mass(body_b);
      k_normal += dot(rn1, mul(solver_inverse_inertia(body_a), rn1));
      k_normal += dot(rn2, mul(solver_inverse_inertia(body_b), rn2));
      manifold->contacts[i].normal_mass = k_normal == 0.0 ? 0.0f : 1.0f / k_normal;

      for (daxa_u32 j = 0u; j < 2; j++) {
        daxa_f32vec3 rt1 = cross(r1, manifold->tangent_vectors[j]);
        daxa_f32vec3 rt2 = cross(r2, manifold->tangent_vectors[j]);
        daxa_f32 k_tangent = solver_inverse_mass(body_a) + solver_inverse_mass(body_b);
        k_tangent += dot(rt1, mul(solver_inverse_inertia(body_a), rt1));
        k_tangent += dot(rt2, mul(solver_inverse_inertia(body_b), rt2));
        manifold->contacts[i].tangent_mass[j] = k_tangent == 0.0 ? 0.0 : 1.0f / k_tangent;
      }

      // soft contact coefficients (solver2d TGS Soft) computed with the SUB-step h
      daxa_f32 contact_hertz = (solver_inverse_mass(body_a) == 0.0f || solver_inverse_mass(body_b) == 0.0f) ? 2.0f * hertz : hertz;
      const daxa_f32 zeta = 10.0f;
      const daxa_f32 omega = 2.0f * _PI * contact_hertz;
      const daxa_f32 a1 = 2.0f * zeta + h * omega;
      const daxa_f32 a2 = h * omega * a1;
      const daxa_f32 a3 = 1.0f / (1.0f + a2);
      manifold->contacts[i].bias_factor = omega / a1;
      manifold->contacts[i].mass_coefficient = a2 * a3;
      manifold->contacts[i].impulse_coefficient = a3;
    }
  }

  // Warm start: re-apply the accumulated impulse as a velocity delta. Box2D v3 runs this at the
  // START OF EVERY SUB-STEP, and that is what carries the contact load across the sub-steps: the
  // following solve then only computes the CORRECTION on top of a load already being held. Doing
  // it once per frame instead leaves sub-steps 2..N re-deriving the contact force from zero through
  // a deliberately compliant constraint with a single iteration, which under-resolves resting
  // stacks and lets them sink. Uses the LIVE arms (rotated local anchors) so it stays consistent
  // with solve()'s separation tracking.
  static void warm_start(Manifold* manifold, RigidBody* body_a, RigidBody* body_b,
                         daxa_b32 accumulate_impulses, daxa_b32 enable_friction) {
    if (!accumulate_impulses) return;
    for (daxa_u32 i = 0u; i < manifold->contact_count; i++) {
      daxa_f32vec3 r1 = body_a->rotate_vector(manifold->contacts[i].anchor_a);
      daxa_f32vec3 r2 = body_b->rotate_vector(manifold->contacts[i].anchor_b);
      daxa_f32vec3 P = manifold->normal * manifold->contacts[i].normal_impulse;
      if (enable_friction) {
        for (daxa_u32 j = 0u; j < 2; j++) {
          P += manifold->tangent_vectors[j] * manifold->contacts[i].tangent_impulse[j];
        }
      }
      if (solver_body_moves(body_a)) {
        body_a->velocity -= P * solver_inverse_mass(body_a);
        body_a->omega -= mul(solver_inverse_inertia(body_a), cross(r1, P));
      }
      if (solver_body_moves(body_b)) {
        body_b->velocity += P * solver_inverse_mass(body_b);
        body_b->omega += mul(solver_inverse_inertia(body_b), cross(r2, P));
      }
    }
  }

  static void solve(Manifold* manifold, RigidBody* body_a, RigidBody* body_b,
                    daxa_b32 accumulate_impulses, daxa_b32 enable_friction, daxa_b32 use_bias, daxa_f32 inv_h) {
    daxa_f32 friction = sqrt(body_a->friction * body_b->friction);
    daxa_f32vec3 n = manifold->normal;
    for (daxa_u32 c = 0u; c < manifold->contact_count; ++c) {
      // LIVE arms + separation from the current pose (the TGS "temporal" tracking)
      daxa_f32vec3 r1 = body_a->rotate_vector(manifold->contacts[c].anchor_a);
      daxa_f32vec3 r2 = body_b->rotate_vector(manifold->contacts[c].anchor_b);
      daxa_f32vec3 world_a = body_a->position + r1;
      daxa_f32vec3 world_b = body_b->position + r2;
      // base separation = the narrow-phase penetration (unchanged by the solver); >0 gap, <0 overlap
      daxa_f32 separation = manifold->contacts[c].penetration + dot(n, world_b - world_a);

      // friction first (Coulomb box on the accumulated tangent impulse, bounded by mu*normal)
      if (enable_friction) {
        for (daxa_u32 j = 0u; j < 2u; ++j) {
          daxa_f32vec3 tdir = manifold->tangent_vectors[j];
          daxa_f32vec3 dv = body_b->velocity + cross(body_b->omega, r2) - body_a->velocity - cross(body_a->omega, r1);
          daxa_f32 vt = dot(dv, tdir);
          daxa_f32 timp = manifold->contacts[c].tangent_mass[j] * -vt;
          daxa_f32 max_f = friction * manifold->contacts[c].normal_impulse;
          if (accumulate_impulses) {
            daxa_f32 old = manifold->contacts[c].tangent_impulse[j];
            manifold->contacts[c].tangent_impulse[j] = clamp(old + timp, -max_f, max_f);
            timp = manifold->contacts[c].tangent_impulse[j] - old;
          } else {
            timp = clamp(timp, -max_f, max_f);
          }
          daxa_f32vec3 P = tdir * timp;
          if (solver_body_moves(body_a)) {
            body_a->velocity -= P * solver_inverse_mass(body_a);
            body_a->omega -= mul(solver_inverse_inertia(body_a), cross(r1, P));
          }
          if (solver_body_moves(body_b)) {
            body_b->velocity += P * solver_inverse_mass(body_b);
            body_b->omega += mul(solver_inverse_inertia(body_b), cross(r2, P));
          }
        }
      }

      // normal: soft constraint with the LIVE separation
      daxa_f32vec3 dvn = body_b->velocity + cross(body_b->omega, r2) - body_a->velocity - cross(body_a->omega, r1);
      daxa_f32 vn = dot(dvn, n);
      daxa_f32 bias = 0.0f;
      daxa_f32 mass_scale = 1.0f;
      daxa_f32 impulse_scale = 0.0f;
      if (separation > 0.0f) {
        bias = separation * inv_h; // speculative: stop bodies before they close the gap
      } else if (use_bias) {
        bias = max(manifold->contacts[c].bias_factor * separation, -4.0f);
        mass_scale = manifold->contacts[c].mass_coefficient;
        impulse_scale = manifold->contacts[c].impulse_coefficient;
      }
      daxa_f32 impulse = -manifold->contacts[c].normal_mass * mass_scale * (vn + bias)
                         - impulse_scale * manifold->contacts[c].normal_impulse;
      if (accumulate_impulses) {
        daxa_f32 old = manifold->contacts[c].normal_impulse;
        manifold->contacts[c].normal_impulse = max(old + impulse, 0.0f);
        impulse = manifold->contacts[c].normal_impulse - old;
      } else {
        impulse = max(impulse, 0.0f);
      }
      daxa_f32vec3 Pn = n * impulse;
      if (solver_body_moves(body_a)) {
        body_a->velocity -= Pn * solver_inverse_mass(body_a);
        body_a->omega -= mul(solver_inverse_inertia(body_a), cross(r1, Pn));
      }
      if (solver_body_moves(body_b)) {
        body_b->velocity += Pn * solver_inverse_mass(body_b);
        body_b->omega += mul(solver_inverse_inertia(body_b), cross(r2, Pn));
      }
    }
  }
};