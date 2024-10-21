#include "shared.inl"

extension RigidBody
{
  [mutating] void advect(Ptr<SimConfig> sim_config)
  {
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
    this.position += this.velocity * dt;
    this.omega *= exp(-dt * angular_damping);
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

    for(int i = 0; i < AABB_CORNER_COUNT; ++i) {
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
    daxa_f32vec3 normal = this.normal;

    if (abs(normal.x) > abs(normal.y)) {
      tangent_vectors[0] = daxa_f32vec3(normal.z, 0, -normal.x);
    } else {
      tangent_vectors[0] = daxa_f32vec3(0, -normal.z, normal.y);
    }

    tangent_vectors[0] = normalize(tangent_vectors[0]);
    tangent_vectors[1] = cross(tangent_vectors[0], normal);
  }

  [mutating] void pre_step(daxa_f32 inv_dt, inout Ptr<RigidBody> body_a, inout Ptr<RigidBody> body_b, daxa_b32 accumulate_impulses) {
    let k_allowed_penetration = 0.01f;
    let k_bias_factor = 0.2f;

    for(int i = 0; i < contact_count; i++) {
      daxa_f32vec3 r1 = contacts[i].position - body_a->position;
      daxa_f32vec3 r2 = contacts[i].position - body_b->position;

      // Precompute normal mass, tangent mass, and bias
      daxa_f32vec3 rn1 = cross(r1, normal);
      daxa_f32vec3 rn2 = cross(r2, normal);
      daxa_f32 k_normal = body_a->inv_mass + body_b->inv_mass;
      k_normal += dot(rn1, mul(rn1, body_a->inv_inertia));
      k_normal += dot(rn2, mul(rn2, body_b->inv_inertia));
      contacts[i].normal_mass = 1.0f / k_normal;

      for(int j = 0; j < 2; j++) {
        daxa_f32vec3 r1ct = cross(tangent_vectors[j], r1);
        daxa_f32vec3 r2ct = cross(tangent_vectors[j], r2);
        daxa_f32 k_tangent = body_a->inv_mass + body_b->inv_mass;
        k_tangent += dot(r1ct, mul(r1ct, body_a->inv_inertia));
        k_tangent += dot(r2ct, mul(r2ct, body_b->inv_inertia));
        contacts[i].tangent_mass[j] = 1.0f / k_tangent;
      }

      contacts[i].bias = -k_bias_factor * inv_dt * min(contacts[i].penetration + k_allowed_penetration, 0.0f);

      // accumlate impulse
      if(accumulate_impulses) {
        // Apply normal + friction impulse
        daxa_f32vec3 P = normal * contacts[i].normal_impulse;

        for(int j = 0; j < 2; j++) {
          P += contacts[i].tangent_impulse[j] * contacts[i].tangent_impulse[j];
        }

        if(body_a->has_flag(RigidBodyFlag::DYNAMIC)) {
          body_a->velocity -= P * body_a->inv_mass;
          body_a->omega -= mul(cross(r1, P), body_a->inv_inertia);
        } 

        if(body_b->has_flag(RigidBodyFlag::DYNAMIC)) {
          body_b->velocity += P * body_b->inv_mass;
          body_b->omega += mul(cross(r2, P), body_b->inv_inertia);
        }
      }
    }
  }

  [mutating]
  void solve(Ptr<RigidBody> body_a, Ptr<RigidBody> body_b, daxa_b32 accumulate_impulses) {
    for(daxa_u32 c = 0u; c < contact_count; ++c)
    {
      Contact contact = contacts[c];
      daxa_f32vec3 collision_point = contact.position;

      // Calculate relative velocity
      daxa_f32vec3 r1_to_point = collision_point - body_a->position;
      daxa_f32vec3 r2_to_point = collision_point - body_b->position;

      daxa_f32vec3 v1 = body_a->velocity;
      daxa_f32vec3 v2 = body_b->velocity;

      // daxa_f32vec3 relative_velocity = v2 + cross(body_b->omega, r2_to_point) - v1 - cross(body_a->omega, r1_to_point);

      daxa_f32vec3 relative_velocity = v2 + cross(r2_to_point, body_b->omega) - v1 - cross(r1_to_point, body_a->omega);

      // Calculate normal impulse
      daxa_f32 normal_velocity = dot(relative_velocity, normal);

      daxa_f32 normal_impulse = contact.normal_mass * (-normal_velocity + contact.bias);

      // Clamp the accumulated impulse
      if(accumulate_impulses) {
        daxa_f32 old_normal_impulse = contact.normal_impulse;
        contact.normal_impulse = max(old_normal_impulse + normal_impulse, 0.0f);
        normal_impulse = contact.normal_impulse - old_normal_impulse;
      } else {
        normal_impulse = max(normal_impulse, 0.0f);
      }

      // Apply contact impulse
      daxa_f32vec3 normal_impulse_vector = normal * normal_impulse;

      if(body_a->has_flag(RigidBodyFlag::DYNAMIC)) {
        body_a->velocity -= normal_impulse_vector * body_a->inv_mass;
        body_a->omega -= mul(cross(normal_impulse_vector, r1_to_point), body_a->inv_inertia);
        // body_a->omega -= mul(body_a->inv_inertia, cross(r1_to_point, normal_impulse_vector));
        body_a->omega = min(body_b->omega, 10.0f);
      }

      if(body_b->has_flag(RigidBodyFlag::DYNAMIC)) {
        body_b->velocity += normal_impulse_vector * body_b->inv_mass;
        body_b->omega += mul(cross(normal_impulse_vector, r2_to_point), body_b->inv_inertia);
        // body_b->omega += mul(body_b->inv_inertia, cross(r2_to_point, normal_impulse_vector));
        // limit the angular velocity
        body_b->omega = min(body_b->omega, 10.0f);
      }

    }
  }
};