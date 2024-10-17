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
};