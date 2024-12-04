#include "shared.inl"

struct PGS {
  static void pre_step(inout Manifold manifold, daxa_f32 inv_dt, inout RigidBody body_a,  inout RigidBody body_b, daxa_b32 accumulate_impulses, daxa_b32 enable_friction, daxa_f32 k_allowed_penetration = PENETRATION_FACTOR, daxa_f32 k_bias_factor = BIAS_FACTOR) {

    for(daxa_u32 i = 0u; i < manifold.contact_count; i++) {

      daxa_f32vec3 r1 = manifold.contacts[i].position - body_a.position;
      daxa_f32vec3 r2 = manifold.contacts[i].position - body_b.position;

      // Precompute normal mass, tangent mass, and bias
      daxa_f32vec3 rn1 = cross(r1, manifold.normal);
      daxa_f32vec3 rn2 = cross(r2, manifold.normal);
      daxa_f32 k_normal = body_a.inv_mass + body_b.inv_mass;
      k_normal += dot(rn1, mul(body_a.get_transformed_inertia(), rn1));
      k_normal += dot(rn2, mul(body_b.get_transformed_inertia(), rn2));
      manifold.contacts[i].normal_mass = k_normal == 0.0 ? 0.0f : 1.0f / k_normal;

      for(daxa_u32 j = 0u; j < 2; j++) {
        daxa_f32vec3 rt1 = cross(r1, manifold.tangent_vectors[j]);
        daxa_f32vec3 rt2 = cross(r2, manifold.tangent_vectors[j]);
        daxa_f32 k_tangent = body_a.inv_mass + body_b.inv_mass;
        k_tangent += dot(rt1, mul(body_a.get_transformed_inertia(), rt1));
        k_tangent += dot(rt2, mul(body_b.get_transformed_inertia(), rt2));
        manifold.contacts[i].tangent_mass[j] = k_tangent == 0.0 ? 0.0 : 1.0f / k_tangent;
      }

      manifold.contacts[i].bias_factor = -k_bias_factor * inv_dt * min(manifold.contacts[i].penetration + k_allowed_penetration, 0.0f);

      // accumlate impulse
      if(accumulate_impulses) {
        // Apply normal + friction impulse
        daxa_f32vec3 P = manifold.normal * manifold.contacts[i].normal_impulse;
        // printf("presolver - contact normal_impulse: %f, tangent_impulse[0]: %f, tangent_impulse[1]: %f, contacts[%d].warm_start: %d\n", contacts[i].normal_impulse, contacts[i].tangent_impulse[0], contacts[i].tangent_impulse[1], i, contacts[i].warm_start);

        if(enable_friction) {
          for(daxa_u32 j = 0u; j < 2; j++) {
            P += manifold.tangent_vectors[j] * manifold.contacts[i].tangent_impulse[j];
          }
        }

        body_a.velocity -= P * body_a.inv_mass;
        body_a.omega -= mul(body_a.get_transformed_inertia(), cross(r1, P));

        body_b.velocity += P * body_b.inv_mass;
        body_b.omega += mul(body_b.get_transformed_inertia(), cross(r2, P));
      }

      // TODO: Revisit this
      // // Apply restitution
      // daxa_f32 relative_velocity = dot(body_b.velocity + cross(body_b.omega, r2) - body_a.velocity - cross(body_a.omega, r1), manifold.normal);

      // if(relative_velocity < -1.0f) {
      //   daxa_f32 restitution = max(body_a.restitution, body_b.restitution);
      //   manifold.contacts[i].bias += -restitution * relative_velocity;
      // }
    }

#if defined(BB_DEBUG)
    // printf("pre_step %lu - %lu - velocity: %f, %f, %f, omega: %f, %f, %f\n", frame_count, body_b.frame_count, body_b.velocity.x, body_b.velocity.y, body_b.velocity.z, body_b.omega.x, body_b.omega.y, body_b.omega.z);
#endif // BB_DEBUG
  }

  static void solve(inout Manifold manifold, inout RigidBody body_a,  inout RigidBody body_b, daxa_b32 accumulate_impulses, daxa_b32 enable_friction) {

    daxa_f32 friction = sqrt(body_a.friction * body_b.friction);
    for(daxa_u32 c = 0u; c < manifold.contact_count; ++c)
    {
      daxa_f32vec3 collision_point = manifold.contacts[c].position;

      // Calculate relative velocity
      daxa_f32vec3 r1_to_point = collision_point - body_a.position;
      daxa_f32vec3 r2_to_point = collision_point - body_b.position;

      daxa_f32vec3 relative_velocity = body_b.velocity + cross(body_b.omega, r2_to_point) - body_a.velocity - cross(body_a.omega, r1_to_point);

      // Friction
      if(enable_friction) {
          for(daxa_u32 j = 0u; j < 2u; ++j)
          {
            daxa_f32 tangent_velocity = dot(relative_velocity, manifold.tangent_vectors[j]);
            daxa_f32 tangent_impulse = manifold.contacts[c].tangent_mass[j] * -tangent_velocity;

            daxa_f32 max_friction_impulse = friction * manifold.contacts[c].normal_impulse;

            // Clamp the accumulated impulse
            if(accumulate_impulses) {
              daxa_f32 old_impulse = manifold.contacts[c].tangent_impulse[j];
              manifold.contacts[c].tangent_impulse[j] = clamp(old_impulse + tangent_impulse, -max_friction_impulse, max_friction_impulse);
              tangent_impulse = manifold.contacts[c].tangent_impulse[j] - old_impulse;
            } else {
              tangent_impulse = clamp(tangent_impulse, -max_friction_impulse, max_friction_impulse);
            }

            // Apply contact impulse
            daxa_f32vec3 tangent_impulse_vector = manifold.tangent_vectors[j] * tangent_impulse;

            body_a.velocity -= tangent_impulse_vector * body_a.inv_mass;
            body_a.omega -= mul(body_a.get_transformed_inertia(), cross(r1_to_point, tangent_impulse_vector));

            body_b.velocity += tangent_impulse_vector * body_b.inv_mass;
            body_b.omega += mul(body_b.get_transformed_inertia(), cross(r2_to_point, tangent_impulse_vector));
          }
      }

      relative_velocity = body_b.velocity + cross(body_b.omega, r2_to_point) - body_a.velocity - cross(body_a.omega, r1_to_point);

      // Calculate normal impulse
      daxa_f32 normal_velocity = dot(relative_velocity, manifold.normal);

      daxa_f32 normal_impulse = manifold.contacts[c].normal_mass * (-normal_velocity + manifold.contacts[c].bias_factor);

      // Clamp the accumulated impulse
      if(accumulate_impulses) {
        daxa_f32 old_normal_impulse = manifold.contacts[c].normal_impulse;
        manifold.contacts[c].normal_impulse = max(old_normal_impulse + normal_impulse, 0.0f);
        normal_impulse = manifold.contacts[c].normal_impulse - old_normal_impulse;
        // printf("normal_impulse: %f = contact normal_impulse: %f - old_normal_impulse: %f, normal_mass: %f, normal_velocity: %f, bias: %f\n", normal_impulse, contacts[c].normal_impulse, old_normal_impulse, contacts[c].normal_mass, normal_velocity, contacts[c].bias);
      } else {
        normal_impulse = max(normal_impulse, 0.0f);
      }

      // Apply contact impulse
      daxa_f32vec3 normal_impulse_vector = manifold.normal * normal_impulse;

      body_a.velocity -= normal_impulse_vector * body_a.inv_mass;
      body_a.omega -= mul(body_a.get_transformed_inertia(), cross(r1_to_point, normal_impulse_vector));

      body_b.velocity += normal_impulse_vector * body_b.inv_mass;
      body_b.omega += mul(body_b.get_transformed_inertia(), cross(r2_to_point, normal_impulse_vector));

    }

#if defined(BB_DEBUG)
    // printf("solve %lu - %lu - velocity: %f, %f, %f, omega: %f, %f, %f\n", frame_count, body_b.frame_count, body_b.velocity.x, body_b.velocity.y, body_b.velocity.z, body_b.omega.x, body_b.omega.y, body_b.omega.z);
#endif // BB_DEBUG
  }
}



struct PGS_SOFT {
  static void pre_step(inout Manifold manifold, daxa_f32 inv_dt, inout RigidBody body_a, inout RigidBody body_b, daxa_b32 accumulate_impulses, daxa_b32 enable_friction, daxa_f32 h, daxa_f32 hertz) {

    for(daxa_u32 i = 0u; i < manifold.contact_count; i++) {

      daxa_f32vec3 r1 = manifold.contacts[i].position - manifold.anchor_a;
      daxa_f32vec3 r2 = manifold.contacts[i].position - manifold.anchor_b;

      // Precompute normal mass, tangent mass, and bias
      daxa_f32vec3 rn1 = cross(r1, manifold.normal);
      daxa_f32vec3 rn2 = cross(r2, manifold.normal);
      daxa_f32 k_normal = body_a.inv_mass + body_b.inv_mass;
      k_normal += dot(rn1, mul(body_a.get_transformed_inertia(), rn1));
      k_normal += dot(rn2, mul(body_b.get_transformed_inertia(), rn2));
      manifold.contacts[i].normal_mass = k_normal == 0.0 ? 0.0f : 1.0f / k_normal;

      for(daxa_u32 j = 0u; j < 2; j++) {
        daxa_f32vec3 rt1 = cross(r1, manifold.tangent_vectors[j]);
        daxa_f32vec3 rt2 = cross(r2, manifold.tangent_vectors[j]);
        daxa_f32 k_tangent = body_a.inv_mass + body_b.inv_mass;
        k_tangent += dot(rt1, mul(body_a.get_transformed_inertia(), rt1));
        k_tangent += dot(rt2, mul(body_b.get_transformed_inertia(), rt2));
        manifold.contacts[i].tangent_mass[j] = k_tangent == 0.0 ? 0.0 : 1.0f / k_tangent;
      }

      daxa_f32 contact_hertz = (body_a.inv_mass == 0.0f || body_b.inv_mass == 0.0f) ? 2.0f * hertz : hertz;

      // soft contact
			// should use the substep not the full time step
			const daxa_f32 zeta = 10.0f;
			const daxa_f32 omega = 2.0f * _PI * contact_hertz;
			const daxa_f32 c = h * omega * (2.0f * zeta + h * omega);
			manifold.contacts[i].bias_factor = omega / (2.0f * zeta + h * omega);
			manifold.contacts[i].impulse_coefficient = 1.0f / (1.0f + c);
			manifold.contacts[i].mass_coefficient = c * manifold.contacts[i].impulse_coefficient;

      // accumlate impulse
      if(accumulate_impulses) {
        // Apply normal + friction impulse
        daxa_f32vec3 P = manifold.normal * manifold.contacts[i].normal_impulse;

        if(enable_friction) {
          for(daxa_u32 j = 0u; j < 2; j++) {
            P += manifold.tangent_vectors[j] * manifold.contacts[i].tangent_impulse[j];
          }
        }

        body_a.velocity -= P * body_a.inv_mass;
        body_a.omega -= mul(body_a.get_transformed_inertia(), cross(r1, P));

        body_b.velocity += P * body_b.inv_mass;
        body_b.omega += mul(body_b.get_transformed_inertia(), cross(r2, P));
      }
    }
  }

  
  static void solve(inout Manifold manifold, inout RigidBody body_a, inout RigidBody body_b, daxa_b32 accumulate_impulses, daxa_b32 enable_friction, daxa_b32 use_bias, daxa_f32 inv_h) {

    daxa_f32 friction = sqrt(body_a.friction * body_b.friction);
    for(daxa_u32 c = 0u; c < manifold.contact_count; ++c)
    {
      daxa_f32vec3 collision_point = manifold.contacts[c].position;

      // Calculate relative velocity
      daxa_f32vec3 r1_to_point = collision_point - manifold.anchor_a;
      daxa_f32vec3 r2_to_point = collision_point - manifold.anchor_b;

      daxa_f32vec3 relative_velocity = body_b.velocity + cross(body_b.omega, r2_to_point) - body_a.velocity - cross(body_a.omega, r1_to_point);

      // Friction
      if(enable_friction) {
          for(daxa_u32 j = 0u; j < 2u; ++j)
          {
            daxa_f32 tangent_velocity = dot(relative_velocity, manifold.tangent_vectors[j]);
            daxa_f32 tangent_impulse = manifold.contacts[c].tangent_mass[j] * -tangent_velocity;

            daxa_f32 max_friction_impulse = friction * manifold.contacts[c].normal_impulse;

            // Clamp the accumulated impulse
            if(accumulate_impulses) {
              daxa_f32 old_impulse = manifold.contacts[c].tangent_impulse[j];
              manifold.contacts[c].tangent_impulse[j] = clamp(old_impulse + tangent_impulse, -max_friction_impulse, max_friction_impulse);
              tangent_impulse = manifold.contacts[c].tangent_impulse[j] - old_impulse;
            } else {
              tangent_impulse = clamp(tangent_impulse, -max_friction_impulse, max_friction_impulse);
            }

            // Apply contact impulse
            daxa_f32vec3 tangent_impulse_vector = manifold.tangent_vectors[j] * tangent_impulse;

            body_a.velocity -= tangent_impulse_vector * body_a.inv_mass;
            body_a.omega -= mul(body_a.get_transformed_inertia(), cross(r1_to_point, tangent_impulse_vector));

            body_b.velocity += tangent_impulse_vector * body_b.inv_mass;
            body_b.omega += mul(body_b.get_transformed_inertia(), cross(r2_to_point, tangent_impulse_vector));
          }
      }

      relative_velocity = body_b.velocity + cross(body_b.omega, r2_to_point) - body_a.velocity - cross(body_a.omega, r1_to_point);

      // Calculate normal impulse
      daxa_f32 normal_velocity = dot(relative_velocity, manifold.normal);
      daxa_f32 bias = 0.0f;
      daxa_f32 mass_scale = 1.0f;
      daxa_f32 impulse_scale = 0.0f;

      if(manifold.contacts[c].penetration > 0.0f) {
        bias = manifold.contacts[c].penetration * inv_h;
      } else if(use_bias) {
        bias = max(manifold.contacts[c].bias_factor * manifold.contacts[c].penetration, -4.0f);
        mass_scale = manifold.contacts[c].mass_coefficient;
        impulse_scale = manifold.contacts[c].impulse_coefficient;
      }

      daxa_f32 impulse = -manifold.contacts[c].normal_mass * mass_scale * (normal_velocity + bias) - impulse_scale * manifold.contacts[c].normal_impulse;

      // Clamp the accumulated impulse
      if(accumulate_impulses) {
        daxa_f32 old_normal_impulse = manifold.contacts[c].normal_impulse;
        manifold.contacts[c].normal_impulse = max(old_normal_impulse + impulse, 0.0f);
        impulse = manifold.contacts[c].normal_impulse - old_normal_impulse;
      } else {
        impulse = max(impulse, 0.0f);
      }

      // Apply contact impulse
      daxa_f32vec3 normal_impulse_vector = manifold.normal * impulse;

      body_a.velocity -= normal_impulse_vector * body_a.inv_mass;
      body_a.omega -= mul(body_a.get_transformed_inertia(), cross(r1_to_point, normal_impulse_vector));

      body_b.velocity += normal_impulse_vector * body_b.inv_mass;
      body_b.omega += mul(body_b.get_transformed_inertia(), cross(r2_to_point, normal_impulse_vector));

    }
  }
};