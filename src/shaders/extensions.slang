#include "shared.inl"

extension RigidBody
{
  [mutating] void advect(Ptr<SimConfig> sim_config)
  {
    integrate_velocities(sim_config->dt);
    apply_temporal_velocity();
    advance(sim_config->dt, LINEAR_DAMPING, ANGULAR_DAMPING);
    if(has_flag(RigidBodyFlag::GRAVITY)){
      apply_impulse(this.mass * daxa_f32vec3(0, sim_config->gravity, 0) *       sim_config->dt, this.position);
    }
    reset_state();
  }

  [mutating] void apply_temporal_velocity()
  {
    this.velocity += this.tmp_velocity;
    this.omega += this.tmp_omega;
  }

  [mutating] void advance(daxa_f32 dt, daxa_f32 linear_damping, daxa_f32 angular_damping)
  {
    this.velocity *= exp(-dt * linear_damping);
    this.omega *= exp(-dt * angular_damping);
  }

  [mutating] void integrate_velocities(daxa_f32 dt) {
    
    this.position += this.velocity * dt;
    this.rotation = apply_angular_velocity(this.rotation, this.omega, dt);
  }

      [mutating] Quaternion apply_angular_velocity(Quaternion rotation, daxa_f32vec3 omega, daxa_f32 dt)
  {
    daxa_f32 angle = length(omega) * dt;
    if (angle > 0.0001f)
    {
      daxa_f32vec3 axis = normalize(omega);
      daxa_f32 half_angle = angle / 2.0f;
      daxa_f32 sin_half_angle = sin(half_angle);
      daxa_f32 cos_half_angle = cos(half_angle);
      Quaternion q = Quaternion(axis * sin_half_angle, cos_half_angle);
      return (q * rotation).normalize();
    }
    else
    {
      // For small angles, use approximation
      daxa_f32vec3 delta_rotation = omega * dt * 0.5f;
      Quaternion q = Quaternion(delta_rotation, 1.0f);
      return (q * rotation).normalize();
    }
  }

  [mutating] void apply_impulse(daxa_f32vec3 impulse, daxa_f32vec3 point)
  {
    daxa_f32vec3 linear_velocity = impulse * this.inv_mass;
    this.velocity += linear_velocity;

    daxa_f32vec3 r = point - this.position;
    daxa_f32vec3 torque = cross(r, impulse);
    daxa_f32vec3 angular_velocity = mul(torque, this.inv_inertia);
    this.omega += angular_velocity;
  }

  [mutating] void reset_state()
  {
    this.tmp_velocity = daxa_f32vec3(0, 0, 0);
    this.tmp_omega = daxa_f32vec3(0, 0, 0);
  }

  daxa_f32vec3 get_velocity_at(daxa_f32vec3 point)
  {
    return this.velocity - cross(point - this.position, this.omega);
  }

  daxa_f32mat3x3 get_transformed_inertia()
  {
    daxa_f32mat3x3 rotation_matrix = get_rotation_matrix();
    return mul(rotation_matrix, mul(inv_inertia, transpose(rotation_matrix)));
  }

  daxa_f32mat3x3 cross_product_matrix(daxa_f32vec3 v)
  {
    return daxa_f32mat3x3(0, -v.z, v.y,
                          v.z, 0, -v.x,
                          -v.y, v.x, 0);
  }

  daxa_f32 get_impulse_contribution(daxa_f32vec3 position, daxa_f32vec3 normal)
  {
    daxa_f32mat3x3 inv_inertia = get_transformed_inertia();
    daxa_f32mat3x3 rn = cross_product_matrix(position);
    daxa_f32mat3x3 K = mul(rn, mul(inv_inertia, transpose(rn)));
    return (this.inv_mass + dot(normal, mul(K, normal)));
  }

  Aabb get_min_max_from_OBB()
  {
    Aabb aabb = Aabb(this.minimum, this.maximum);

    daxa_f32vec3 minimum = daxa_f32vec3(FLT_MAX, FLT_MAX, FLT_MAX);
    daxa_f32vec3 maximum = daxa_f32vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for(daxa_u32 i = 0u; i < AABB_CORNER_COUNT; ++i) {
        daxa_f32vec3 corner = aabb.get_corner(i);
        corner = object_to_world(corner);
        minimum = min(minimum, corner);
        maximum = max(maximum, corner);
    }

    return Aabb(minimum, maximum);
  }

  [mutating] 
  void apply_contact(daxa_f32vec3 collision_point, daxa_f32vec3 normal) {
    daxa_f32 friction = friction;
    daxa_f32 restitution = restitution;
    daxa_f32vec3 v_at_point = get_velocity_at(collision_point);
    daxa_f32vec3 r0 = collision_point - position;
    daxa_f32 v_normal = dot(normal, v_at_point);

    daxa_f32 impulse_denom = get_impulse_contribution(r0, normal);
    daxa_f32 J = -(1 + restitution) * v_normal / impulse_denom;

    if (J > 0) {
      daxa_f32vec3 impulse = J * normal;
      apply_impulse(impulse, collision_point);

      // Friction
      daxa_f32vec3 v_after = get_velocity_at(collision_point);
      daxa_f32vec3 v_tangent = v_after - dot(v_after, normal) * normal;
      
      if (length(v_tangent) > COLLISION_GUARD) {
          daxa_f32vec3 tangent = normalize(v_tangent);
          daxa_f32 j_tangent = -dot(v_after, tangent) / get_impulse_contribution(r0, tangent);
          j_tangent = clamp(j_tangent, -friction * J, friction * J);
          daxa_f32vec3 friction_impulse = j_tangent * tangent;
          apply_impulse(friction_impulse, collision_point);
      }
    }
  }
};


extension Transform {

    daxa_f32mat3x3 get_rotation_matrix() {
      return rotation;
    }

    daxa_f32mat3x3 get_inverse_rotation_matrix() {
      return transpose(rotation);
    }

    daxa_f32vec3 get_position() {
      return position;
    }

    daxa_f32vec3 rotate(daxa_f32vec3 v) {
      return mul(v, rotation);
      // return mul(rotation, v);
    }

    daxa_f32vec3 rotate_transpose(daxa_f32vec3 v) {
      return mul(v, transpose(rotation));
      // return mul(transpose(rotation), v);
    }

    daxa_f32vec3 transform(daxa_f32vec3 v) {
      return mul(v, rotation) + position;
      // return mul(rotation, v) + position;
    }

    daxa_f32vec3 transform_transpose(daxa_f32vec3 v) {
      return mul(v - position, transpose(rotation));
      // return mul(transpose(rotation), v - position);
    }

    daxa_f32vec3 get_axis(daxa_u32 i) {
      return daxa_f32vec3(rotation[0][i], rotation[1][i], rotation[2][i]);
    }

    daxa_f32vec3 get_inverse_axis(daxa_u32 i) {
      return daxa_f32vec3(rotation[i][0], rotation[i][1], rotation[i][2]);
    }

    daxa_f32 get_rotation_element(daxa_u32 i, daxa_u32 j) {
      return rotation[i][j];
    }
};

extension Manifold {
  [mutating] void compute_basis() 
  {
    if (abs(normal.x) > abs(normal.y)) {
      tangent_vectors[0] = daxa_f32vec3(normal.z, 0, -normal.x);
    } else {
      tangent_vectors[0] = daxa_f32vec3(0, -normal.z, normal.y);
    }

    tangent_vectors[0] = normalize(tangent_vectors[0]);
    tangent_vectors[1] = cross(tangent_vectors[0], normal);
  }


  bool is_same_contact(FeaturePair a, FeaturePair b) {
    return a.in_reference == b.in_reference && a.out_reference == b.out_reference && a.in_incident == b.in_incident && a.out_incident == b.out_incident;
  }

  void key_to_fp(daxa_u32 key, out FeaturePair fp) {
    fp.in_reference = key & 0x000000FF;
    fp.out_reference = (key & 0x0000FF00) >> 8;
    fp.in_incident = (key & 0x00FF0000) >> 16;
    fp.out_incident = (key & 0xFF000000) >> 24;
  }

  [mutating]
  void update(Manifold old_manifold) {
    daxa_f32vec3 otv0  = old_manifold.tangent_vectors[0];
    daxa_f32vec3 otv1  = old_manifold.tangent_vectors[1];

    for(daxa_u32 i = 0u; i < contact_count; i++) {
      for(daxa_u32 j = 0u; j < old_manifold.contact_count; j++) {
        if(is_same_contact(old_manifold.contacts[j].fp, contacts[i].fp)) {

            contacts[i].normal_impulse = old_manifold.contacts[j].normal_impulse;

            // Attempt to re-project old friction solutions
            daxa_f32vec3 friction = otv0 * old_manifold.contacts[j].tangent_impulse[0] + otv1 * old_manifold.contacts[j].tangent_impulse[1];

            contacts[i].tangent_impulse[0] = dot(friction, tangent_vectors[0]);
            contacts[i].tangent_impulse[1] = dot(friction, tangent_vectors[1]);

            contacts[i].warm_start = max(old_manifold.contacts[j].warm_start, 1u);

            // printf("fp %d, %d, %d, %d, old %d, %d, %d, %d\n", contacts[i].fp.in_reference, contacts[i].fp.out_reference, contacts[i].fp.in_incident, contacts[i].fp.out_incident, old_manifold.contacts[j].fp.in_reference, old_manifold.contacts[j].fp.out_reference, old_manifold.contacts[j].fp.in_incident, old_manifold.contacts[j].fp.out_incident);

            // if(contacts[i].normal_impulse != 0) {
              printf("tangent_impulse[0] %f, tangent_impulse[1] %f normal_impulse %f friction %f %f %f \n", contacts[i].tangent_impulse[0], contacts[i].tangent_impulse[1], contacts[i].normal_impulse, friction.x, friction.y, friction.z);
              printf("    tangent_vectors[0] %f %f %f tangent_vectors[1] %f %f %f\n", tangent_vectors[0].x, tangent_vectors[0].y, tangent_vectors[0].z, tangent_vectors[1].x, tangent_vectors[1].y, tangent_vectors[1].z);
            // }
          break;
        }
      }
    }
  }

  [mutating] void pre_step(daxa_f32 inv_dt, Ptr<RigidBody> body_a,  Ptr<RigidBody> body_b, daxa_b32 accumulate_impulses, daxa_b32 enable_friction) {
    let k_allowed_penetration = 0.01f;
    // let k_allowed_penetration = 0.001f;
    let k_bias_factor = 0.2f;

    for(daxa_u32 i = 0u; i < contact_count; i++) {

      daxa_f32vec3 r1 = contacts[i].position - body_a->position;
      daxa_f32vec3 r2 = contacts[i].position - body_b->position;

      // Precompute normal mass, tangent mass, and bias
      daxa_f32vec3 rn1 = cross(r1, normal);
      daxa_f32vec3 rn2 = cross(r2, normal);
      daxa_f32 k_normal = body_a->inv_mass + body_b->inv_mass;
      k_normal += dot(rn1, mul(body_a->inv_inertia, rn1));
      k_normal += dot(rn2, mul(body_b->inv_inertia, rn2));
      contacts[i].normal_mass = 1.0f / k_normal;

      for(daxa_u32 j = 0u; j < 2; j++) {
        daxa_f32vec3 r1ct = cross(tangent_vectors[j], r1);
        daxa_f32vec3 r2ct = cross(tangent_vectors[j], r2);
        daxa_f32 k_tangent = body_a->inv_mass + body_b->inv_mass;
        k_tangent += dot(r1ct, mul(body_a->inv_inertia, r1ct));
        k_tangent += dot(r2ct, mul(body_b->inv_inertia, r2ct));
        contacts[i].tangent_mass[j] = 1.0f / k_tangent;

        // printf("      tangent_mass %f\n", contacts[i].tangent_mass[j]);
      }

      contacts[i].bias = -k_bias_factor * inv_dt * min(contacts[i].penetration + k_allowed_penetration, 0.0f);

      // accumlate impulse
      if(accumulate_impulses) {
        // Apply normal + friction impulse
        daxa_f32vec3 P = normal * contacts[i].normal_impulse;

        if(enable_friction) {
          for(daxa_u32 j = 0u; j < 2; j++) {
            P += tangent_vectors[j] * contacts[i].tangent_impulse[j];
          }
        }

        if(body_a->has_flag(RigidBodyFlag::DYNAMIC)) {
          body_a->velocity -= P * body_a->inv_mass;
          body_a->omega -= mul(body_a->inv_inertia, cross(r1, P));
        } 

        if(body_b->has_flag(RigidBodyFlag::DYNAMIC)) {
          body_b->velocity += P * body_b->inv_mass;
          body_b->omega += mul(body_b->inv_inertia, cross(r2, P));
        }
      }

      daxa_f32 relative_velocity = dot(body_b->velocity + cross(body_b->omega, r2) - body_a->velocity - cross(body_a->omega, r1), normal);

      if(relative_velocity < -1.0f) {
        daxa_f32 restitution = max(body_a->restitution, body_b->restitution);
        contacts[i].bias += -restitution * relative_velocity;
      }
    }
  }

  [mutating]
  void solve(Ptr<RigidBody> body_a, Ptr<RigidBody> body_b, daxa_b32 accumulate_impulses, daxa_b32 enable_friction) {

    daxa_f32 friction = sqrt(body_a->friction * body_b->friction);
    for(daxa_u32 c = 0u; c < contact_count; ++c)
    {
      daxa_f32vec3 collision_point = contacts[c].position;

      // Calculate relative velocity
      daxa_f32vec3 r1_to_point = collision_point - body_a->position;
      daxa_f32vec3 r2_to_point = collision_point - body_b->position;

      daxa_f32vec3 relative_velocity = body_b->velocity + cross(body_b->omega, r2_to_point) - body_a->velocity - cross(body_a->omega, r1_to_point);

      // Friction
      if(enable_friction) {
          for(daxa_u32 j = 0u; j < 2u; ++j)
          {
            daxa_f32 tangent_velocity = dot(relative_velocity, tangent_vectors[j]);
            daxa_f32 tangent_impulse = contacts[c].tangent_mass[j] * -tangent_velocity;

            daxa_f32 max_friction_impulse = friction * contacts[c].normal_impulse;

            // Clamp the accumulated impulse
            if(accumulate_impulses) {
              daxa_f32 old_impulse = contacts[c].tangent_impulse[j];
              contacts[c].tangent_impulse[j] = clamp(old_impulse + tangent_impulse, -max_friction_impulse, max_friction_impulse);
              tangent_impulse = contacts[c].tangent_impulse[j] - old_impulse;

              // printf("        tangent_impulse %f, friction %f, normal_impulse %f, tangent_impulse[%d] %f\n", contacts[c].tangent_impulse[j], friction, contacts[c].normal_impulse, j, contacts[c].tangent_mass[j]);
            } else {
              tangent_impulse = clamp(tangent_impulse, -max_friction_impulse, max_friction_impulse);
            }

            // Apply contact impulse
            daxa_f32vec3 tangent_impulse_vector = tangent_vectors[j] * tangent_impulse;

            if(body_a->has_flag(RigidBodyFlag::DYNAMIC)) {
              body_a->velocity -= tangent_impulse_vector * body_a->inv_mass;
              body_a->omega -= mul(body_a->inv_inertia, cross(r1_to_point, tangent_impulse_vector));
            }

            if(body_b->has_flag(RigidBodyFlag::DYNAMIC)) {
              body_b->velocity += tangent_impulse_vector * body_b->inv_mass;
              body_b->omega += mul(body_b->inv_inertia, cross(r2_to_point, tangent_impulse_vector));
            }
          }
      }

      relative_velocity = body_b->velocity + cross(body_b->omega, r2_to_point) - body_a->velocity - cross(body_a->omega, r1_to_point);

      // Calculate normal impulse
      daxa_f32 normal_velocity = dot(relative_velocity, normal);

      daxa_f32 normal_impulse = contacts[c].normal_mass * (-normal_velocity + contacts[c].bias);

      // Clamp the accumulated impulse
      if(accumulate_impulses) {
        daxa_f32 old_normal_impulse = contacts[c].normal_impulse;
        contacts[c].normal_impulse = max(old_normal_impulse + normal_impulse, 0.0f);
        normal_impulse = contacts[c].normal_impulse - old_normal_impulse;
      } else {
        normal_impulse = max(normal_impulse, 0.0f);
      }

      // if(contacts[c].normal_impulse != 0) {
      //   printf("      contacts[c].normal_impulse %f, normal_velocity %f, normal_mass %f\n", contacts[c].normal_impulse, normal_velocity, contacts[c].normal_mass);
      // }

      // Apply contact impulse
      daxa_f32vec3 normal_impulse_vector = normal * normal_impulse;

      if(body_a->has_flag(RigidBodyFlag::DYNAMIC)) {
        body_a->velocity -= normal_impulse_vector * body_a->inv_mass;
        body_a->omega -= mul(body_a->inv_inertia, cross(r1_to_point, normal_impulse_vector));
      }

      if(body_b->has_flag(RigidBodyFlag::DYNAMIC)) {
        body_b->velocity += normal_impulse_vector * body_b->inv_mass;
        body_b->omega += mul(body_b->inv_inertia, cross(r2_to_point, normal_impulse_vector));
      }

      // if(any(isnan(collision_point))) {
        // printf("manifold.error %d, contact_count: %d\n", error, contact_count);
        // printf("collision_point : %f %f %f, body_a.position : %f %f %f, body_b.position : %f %f %f\n", collision_point.x, collision_point.y, collision_point.z, body_a->position.x, body_a->position.y, body_a->position.z, body_b->position.x, body_b->position.y, body_b->position.z);

        // printf("r1_to_point : %f %f %f, r2_to_point : %f %f %f, normal_impulse_vector : %f %f %f\n", r1_to_point.x, r1_to_point.y, r1_to_point.z, r2_to_point.x, r2_to_point.y, r2_to_point.z, normal_impulse_vector.x, normal_impulse_vector.y, normal_impulse_vector.z);

        // printf("relative_velocity : %f %f %f, normal_velocity : %f, normal_impulse : %f\n", relative_velocity.x, relative_velocity.y, relative_velocity.z, normal_velocity, normal_impulse);
      // }
    }
  }
};


func find_old_collision(daxa_u32 body_a_index, daxa_u32 body_b_index, daxa_u32 old_collision_count, Ptr<Manifold> old_collisions, out Manifold old_manifold) -> daxa_b32 {
  for(daxa_u32 i = 0u; i < old_collision_count; ++i) {
    if(old_collisions[i].obb1_index == body_a_index && old_collisions[i].obb2_index == body_b_index) {
      old_manifold = old_collisions[i];
      return true;
    }
  }
  return false;
}