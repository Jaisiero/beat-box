#include "shared.inl"

void copy_rigid_body_fields(RigidBody* dst, RigidBody* src)
{
  dst->id = src->id;
  dst->flags = src->flags;
  dst->island_index = src->island_index;
  dst->active_index = src->active_index;
  dst->manifold_node_index = src->manifold_node_index;
  dst->material_index = src->material_index;
#if defined(BB_DEBUG)
  dst->face_collided = src->face_collided;
#endif // BB_DEBUG
  dst->primitive_count = src->primitive_count;
  dst->primitive_offset = src->primitive_offset;
  dst->position = src->position;
  dst->rotation.v = src->rotation.v;
  dst->rotation.w = src->rotation.w;
  dst->minimum = src->minimum;
  dst->maximum = src->maximum;
  dst->mass = src->mass;
  dst->inv_mass = src->inv_mass;
  dst->velocity = src->velocity;
  dst->omega = src->omega;
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
}

void copy_contact_fields(Contact* dst, Contact* src)
{
  dst->position = src->position;
  dst->penetration = src->penetration;
  dst->normal_impulse = src->normal_impulse;
  dst->tangent_impulse[0] = src->tangent_impulse[0];
  dst->tangent_impulse[1] = src->tangent_impulse[1];
  dst->bias_factor = src->bias_factor;
  dst->impulse_coefficient = src->impulse_coefficient;
  dst->mass_coefficient = src->mass_coefficient;
  dst->normal_mass = src->normal_mass;
  dst->tangent_mass[0] = src->tangent_mass[0];
  dst->tangent_mass[1] = src->tangent_mass[1];
  dst->fp.in_reference = src->fp.in_reference;
  dst->fp.out_reference = src->fp.out_reference;
  dst->fp.in_incident = src->fp.in_incident;
  dst->fp.out_incident = src->fp.out_incident;
  dst->warm_start = src->warm_start;
}

void write_contact_fields_from_value(Contact* dst, Contact src)
{
  dst->position = src.position;
  dst->penetration = src.penetration;
  dst->normal_impulse = src.normal_impulse;
  dst->tangent_impulse[0] = src.tangent_impulse[0];
  dst->tangent_impulse[1] = src.tangent_impulse[1];
  dst->bias_factor = src.bias_factor;
  dst->impulse_coefficient = src.impulse_coefficient;
  dst->mass_coefficient = src.mass_coefficient;
  dst->normal_mass = src.normal_mass;
  dst->tangent_mass[0] = src.tangent_mass[0];
  dst->tangent_mass[1] = src.tangent_mass[1];
  dst->fp.in_reference = src.fp.in_reference;
  dst->fp.out_reference = src.fp.out_reference;
  dst->fp.in_incident = src.fp.in_incident;
  dst->fp.out_incident = src.fp.out_incident;
  dst->warm_start = src.warm_start;
}

void copy_manifold_fields(Manifold* dst, Manifold* src)
{
  dst->obb1_index = src->obb1_index;
  dst->obb2_index = src->obb2_index;
  dst->anchor_a = src->anchor_a;
  dst->anchor_b = src->anchor_b;
  dst->key = src->key;
  dst->error = src->error;
  dst->frame_count = src->frame_count;
  dst->normal = src->normal;
  dst->tangent_vectors[0] = src->tangent_vectors[0];
  dst->tangent_vectors[1] = src->tangent_vectors[1];
  dst->contact_count = src->contact_count;
  for (daxa_u32 i = 0u; i < MAX_CONTACT_POINT_COUNT; ++i)
  {
    copy_contact_fields(&dst->contacts[i], &src->contacts[i]);
  }
}

void write_manifold_fields_from_value(Manifold* dst, Manifold src)
{
  dst->obb1_index = src.obb1_index;
  dst->obb2_index = src.obb2_index;
  dst->anchor_a = src.anchor_a;
  dst->anchor_b = src.anchor_b;
  dst->key = src.key;
  dst->error = src.error;
  dst->frame_count = src.frame_count;
  dst->normal = src.normal;
  dst->tangent_vectors[0] = src.tangent_vectors[0];
  dst->tangent_vectors[1] = src.tangent_vectors[1];
  dst->contact_count = src.contact_count;
  for (daxa_u32 i = 0u; i < MAX_CONTACT_POINT_COUNT; ++i)
  {
    write_contact_fields_from_value(&dst->contacts[i], src.contacts[i]);
  }
}

extension RigidBody
{
  [mutating] void advect(Ptr<SimConfig> sim_config)
  {
    if(has_flag(RigidBodyFlag::GRAVITY)){
      apply_impulse(daxa_f32vec3(0, sim_config->gravity, 0) *       sim_config->dt, this.position);
    }
    advance(sim_config->dt, LINEAR_DAMPING, ANGULAR_DAMPING);
  }

  [mutating] void advance(daxa_f32 dt, daxa_f32 linear_damping, daxa_f32 angular_damping)
  {
    this.velocity *= exp(-dt * linear_damping);
    this.omega *= exp(-dt * angular_damping);
  }

  [mutating] void integrate_positions(daxa_f32 dt) {
    
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

  [mutating] daxa_f32vec3 get_velocity_at(daxa_f32vec3 point)
  {
    return this.velocity - cross(point - this.position, this.omega);
  }

  [mutating] daxa_f32mat3x3 get_transformed_inertia()
  {
    daxa_f32mat3x3 rotation_matrix = get_rotation_matrix();
    return mul(rotation_matrix, mul(inv_inertia, transpose(rotation_matrix)));
  }

  [mutating] Aabb get_AABB()
  {
    return Aabb(minimum, maximum);
  }

  [mutating] Aabb get_AABB_from_OBB()
  {
    Aabb aabb = get_AABB();

    daxa_f32vec3 _minimum = daxa_f32vec3(FLT_MAX, FLT_MAX, FLT_MAX);
    daxa_f32vec3 _maximum = daxa_f32vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for(daxa_u32 i = 0u; i < AABB_CORNER_COUNT; ++i) {
        daxa_f32vec3 corner = aabb.get_corner(i);
        corner = object_to_world(corner);
        _minimum = min(_minimum, corner);
        _maximum = max(_maximum, corner);
    }

    return Aabb(_minimum, _maximum);
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
    }

    daxa_f32vec3 rotate_transpose(daxa_f32vec3 v) {
      return mul(v, transpose(rotation));
    }

    daxa_f32vec3 transform(daxa_f32vec3 v) {
      return mul(v, rotation) + position;
    }

    daxa_f32vec3 transform_transpose(daxa_f32vec3 v) {
      return mul(v - position, transpose(rotation));
    }

    daxa_f32vec3 get_axis(daxa_u32 i) {
      // Return the i-th row of the rotation matrix
      return daxa_f32vec3(rotation[i][0], rotation[i][1], rotation[i][2]);
    }

    daxa_f32vec3 get_inverse_axis(daxa_u32 i) {
        // For rotation matrices, the inverse is the transpose
        // So the inverse axes are the columns of the transpose
        daxa_f32mat3x3 inv_rotation = transpose(rotation);
        return daxa_f32vec3(inv_rotation[i][0], inv_rotation[i][1], inv_rotation[i][2]);
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

static const daxa_f32 POSITION_TOLERANCE = 0.01f;

  bool is_same_contact(Contact* old_contact, FeaturePair contact_fp)
{
    bool features_match = (old_contact->fp.in_incident == contact_fp.in_incident) &&
                          (old_contact->fp.out_incident == contact_fp.out_incident);
    // bool positions_close = length(c1.position - c2.position) < POSITION_TOLERANCE;

    return features_match 
    // && positions_close
    ;
}

  bool is_valid_contact(FeaturePair a) {
    return a.in_reference != -1 && a.out_reference != -1 && a.in_incident != -1 && a.out_incident != -1;
  }

  void key_to_fp(daxa_u32 key, out FeaturePair fp) {
    fp.in_reference = key;
    fp.out_reference = key;
    fp.in_incident = key;
    fp.out_incident = key;
  }

  [mutating]
  void update_from_old(Manifold* old_manifold) {
    // Impulses are only transferable when both manifolds came from the same SAT axis. With a
    // different key the normal/feature interpretation changed — and the contact feature ids are
    // not globally unique across types (face contacts use Edge ids 0..11, edge-edge contacts use
    // axis keys 6..14), so a cross-axis feature match is a false positive that warm-starts a
    // contact with an impulse measured along a different direction.
    if (key != old_manifold->key) {
      return;
    }
    daxa_f32vec3 otv0  = old_manifold->tangent_vectors[0];
    daxa_f32vec3 otv1  = old_manifold->tangent_vectors[1];

    // bool _found[4] = {false, false, false, false};
    for(daxa_u32 i = 0u; i < contact_count; i++) {
      // bool found = false;
      for(daxa_u32 j = 0u; j < old_manifold->contact_count; j++) {
        Contact* old_contact = &old_manifold->contacts[j];
        if(is_same_contact(old_contact, contacts[i].fp)) {
          // found = true;
          // _found[j] = true;
          // printf("update - old fp %d %d %d %d fp %d %d %d %d ws: %d\n", old_manifold.contacts[j].fp.in_reference, old_manifold.contacts[j].fp.out_reference, old_manifold.contacts[j].fp.in_incident, old_manifold.contacts[j].fp.out_incident, contacts[i].fp.in_reference, contacts[i].fp.out_reference, contacts[i].fp.in_incident, contacts[i].fp.out_incident, old_manifold.contacts[j].warm_start);

            contacts[i].normal_impulse = old_contact->normal_impulse;
            contacts[i].tangent_impulse[0] = old_contact->tangent_impulse[0];
            contacts[i].tangent_impulse[1] = old_contact->tangent_impulse[1];

            contacts[i].warm_start = min(255, old_contact->warm_start + 1u);
          break;
        } 
      }

      // if(!found) {
      //   printf("Not updated - fp %d %d %d %d ws: %d\n",  contacts[i].fp.in_reference, contacts[i].fp.out_reference, contacts[i].fp.in_incident, contacts[i].fp.out_incident, contacts[i].warm_start);
      // }
    }
  }
};


func find_old_collision(daxa_u32 body_a_index, daxa_u32 body_b_index, daxa_u32 old_collision_count, Manifold* old_collisions, out daxa_u32 old_index) -> daxa_b32 {
  for(daxa_u32 i = 0u; i < old_collision_count; ++i) {
    if(old_collisions[i].obb1_index == body_a_index && old_collisions[i].obb2_index == body_b_index) {
      old_index = i;
      return true;
    }
  }
  return false;
}
