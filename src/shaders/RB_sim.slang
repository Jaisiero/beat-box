#include "shared.inl"

#if defined(GJK_SIM)
[[vk::push_constant]] GJKPushConstants p;
#elif defined(RIGID_BODY_SIM)
[[vk::push_constant]] RigidBodySimPushConstants p;
#endif

extension RigidBody
{
  [mutating] void advect(Ptr<SimConfig> sim_config)
  {
    apply_temporal_velocity();
    advance(sim_config->dt, LINEAR_DAMPING, ANGULAR_DAMPING);
    apply_impulse(this.mass * daxa_f32vec3(0, sim_config->gravity, 0) * sim_config->dt, this.position);
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

  daxa_f32vec4 quat_mul(daxa_f32vec4 q1, daxa_f32vec4 q2)
  {
    daxa_f32 x = q1.x;
    daxa_f32 y = q1.y;
    daxa_f32 z = q1.z;
    daxa_f32 w = q1.w;

    daxa_f32 x2 = q2.x;
    daxa_f32 y2 = q2.y;
    daxa_f32 z2 = q2.z;
    daxa_f32 w2 = q2.w;

    daxa_f32 result_x = w * x2 + x * w2 + y * z2 - z * y2;
    daxa_f32 result_y = w * y2 - x * z2 + y * w2 + z * x2;
    daxa_f32 result_z = w * z2 + x * y2 - y * x2 + z * w2;
    daxa_f32 result_w = w * w2 - x * x2 - y * y2 - z * z2;

    return daxa_f32vec4(result_x, result_y, result_z, result_w);
  }

      [mutating] daxa_f32vec4 apply_angular_velocity(daxa_f32vec4 rotation, daxa_f32vec3 omega, daxa_f32 dt)
  {
    daxa_f32 angle = length(omega) * dt;
    if (angle > 0.0001f)
    {
      daxa_f32vec3 axis = normalize(omega);
      daxa_f32 half_angle = angle / 2.0f;
      daxa_f32 sin_half_angle = sin(half_angle);
      daxa_f32 cos_half_angle = cos(half_angle);
      daxa_f32vec4 q = daxa_f32vec4(axis * sin_half_angle, cos_half_angle);
      return normalize(quat_mul(q, rotation));
    }
    else
    {
      // For small angles, use approximation
      daxa_f32vec3 delta_rotation = omega * dt * 0.5f;
      daxa_f32vec4 q = daxa_f32vec4(delta_rotation, 1.0f);
      return normalize(quat_mul(q, rotation));
    }
  }

  [mutating] void apply_impulse(daxa_f32vec3 impulse, daxa_f32vec3 point)
  {
    daxa_f32vec3 torque = cross(point - this.position, impulse);
    daxa_f32vec3 linear_velocity = impulse * this.inv_mass;
    daxa_f32vec3 angular_velocity = mul(this.inv_inertia, torque);

    this.velocity += linear_velocity;
    this.omega += angular_velocity;
  }

      [mutating] void reset_state()
  {
    this.tmp_velocity = daxa_f32vec3(0, 0, 0);
    this.tmp_omega = daxa_f32vec3(0, 0, 0);
  }

  daxa_f32vec3 rotate_vector(const daxa_f32vec3 v)
  {
    daxa_f32vec3 u = daxa_f32vec3(this.rotation.x, this.rotation.y, this.rotation.z);
    daxa_f32 s = this.rotation.w;

    return 2.0f * dot(u, v) * u
         + (s * s - dot(u, u)) * v
         + 2.0f * s * cross(u, v);
  }
}

func GJK(const daxa_u32 rigid_body_index, Ptr<RigidBody> r,
         Ptr<SimConfig> sim_config, Ptr<Collision> collisions) -> void
{
  var any_collision = false;
  r.clear_flag(RigidBodyFlag::COLLIDING);
  for (daxa_u32 i = 0; i < sim_config->rigid_body_count; i++)
  {
    if (i == rigid_body_index)
    {
        continue;
    }
    // Simplex simplex;
    Ptr<RigidBody> r2 = Ptr<RigidBody>(&p.task_head.rigid_bodies[i]);

    Aabb aabb1 = Aabb(r.min, r.max);
    aabb1.min = r.rotate_vector(aabb1.min) + r.position;  
    aabb1.max = r.rotate_vector(aabb1.max) + r.position;

    Aabb aabb2 = Aabb(r2.min, r2.max);
    aabb2.min = r2.rotate_vector(aabb2.min) + r2.position;
    aabb2.max = r2.rotate_vector(aabb2.max) + r2.position;

    daxa_f32vec3 penetration;

    let collision = AABB_intersection(aabb1, aabb2, penetration);

    // 
    if (collision)
    {
      any_collision = true;
      daxa_u32 index = 0;
      InterlockedAdd(sim_config->collision_count, 1u, index);

      collisions[index] = Collision(rigid_body_index, i, penetration);
    }
  }

  if(any_collision)
  {
    r->flags |= RigidBodyFlag::COLLIDING;
  }
}

func AABB_intersection(const Aabb aabb1, const Aabb aabb2, out daxa_f32vec3 penetration)->bool
{
  // get the minkowski difference between the two
  var difference = minkowski_difference(aabb1, aabb2);
  if (difference.min.x <= 0 && difference.max.x >= 0 && 
      difference.min.y <= 0 && difference.max.y >= 0 && 
      difference.min.z <= 0 && difference.max.z >= 0)
  {
    penetration = closest_point_on_bounds_to_point(difference, daxa_f32vec3(0, 0, 0));

    return true;
  }
  return false;
}

func minkowski_difference(const Aabb aabb1, const Aabb aabb2) -> Aabb
{
    var difference_min = aabb1.min - aabb2.max;
    var difference_max = aabb1.max - aabb2.min;
    return Aabb(difference_min, difference_max);
}

func closest_point_on_bounds_to_point(const Aabb bounds, const daxa_f32vec3 point) -> daxa_f32vec3
{
    var closest_point = daxa_f32vec3(0, 0, 0);
    closest_point.x = clamp(point.x, bounds.min.x, bounds.max.x);
    closest_point.y = clamp(point.y, bounds.min.y, bounds.max.y);
    closest_point.z = clamp(point.z, bounds.min.z, bounds.max.z);
    return closest_point;
}

#if defined(GJK_SIM)
[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_GJK(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(p.task_head.sim_config);

  if (pixel_i.x >= sim_config->rigid_body_count)
  {
    return;
  }

  Ptr<RigidBody> r =
      Ptr<RigidBody>(&p.task_head.rigid_bodies[pixel_i.x]);

  Ptr<Collision> collisions =
      Ptr<Collision>(p.task_head.collisions);

  GJK(pixel_i.x, r, sim_config, collisions);
}
# elif defined(RIGID_BODY_SIM)

    [shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_rigid_body_sim(uint3 pixel_i : SV_DispatchThreadID)
{
  Ptr<SimConfig> sim_config =
      Ptr<SimConfig>(p.task_head.sim_config);

  if (pixel_i.x >= sim_config->rigid_body_count)
  {
    return;
  }

  Ptr<RigidBody> r =
      Ptr<RigidBody>(&p.task_head.rigid_bodies[pixel_i.x]);

  if (pixel_i.x == 1) {
    r->advect(sim_config);
    // r->position  = daxa_f32vec3(0.5, 2.0, -0.5);
  } else {
    // r->position = daxa_f32vec3(0.0, 0.0, 0.0);
  }
}
#endif // RIGID_BODY_SIM