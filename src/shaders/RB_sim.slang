#include "shared.inl"

[[vk::push_constant]] RigidBodySimPushConstants p;

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

  daxa_f32vec3 rotate_vector(daxa_f32vec3 v)
  {
    daxa_f32vec3 u = daxa_f32vec3(this.rotation.x, this.rotation.y, this.rotation.z);
    daxa_f32 s = this.rotation.w;

    return 2.0f * dot(u, v) * u
         + (s * s - dot(u, u)) * v
         + 2.0f * s * cross(u, v);
  }

  static const daxa_u32 CUBOID_VERTEX_COUNT = 8;

  daxa_f32vec3 find_furthest_point(daxa_f32vec3 dir)
  {
    daxa_f32 max_dist = -FLT_MAX;

    // TODO: optimize this
    // Get vertices from r.min and r.max
    daxa_f32vec3 vertices[CUBOID_VERTEX_COUNT] = {
        daxa_f32vec3(this.min.x, this.min.y, this.min.z),
        daxa_f32vec3(this.min.x, this.min.y, this.max.z),
        daxa_f32vec3(this.min.x, this.max.y, this.min.z),
        daxa_f32vec3(this.min.x, this.max.y, this.max.z),
        daxa_f32vec3(this.max.x, this.min.y, this.min.z),
        daxa_f32vec3(this.max.x, this.min.y, this.max.z),
        daxa_f32vec3(this.max.x, this.max.y, this.min.z),
        daxa_f32vec3(this.max.x, this.max.y, this.max.z),
    };

    for (daxa_u32 i = 0; i < CUBOID_VERTEX_COUNT; i++)
    {
      vertices[i] = rotate_vector(vertices[i]);
      vertices[i] += this.position;
    }

    daxa_f32vec3 furthest_point = daxa_f32vec3(0, 0, 0);
    for (daxa_u32 i = 0; i < CUBOID_VERTEX_COUNT; i++)
    {
      daxa_f32vec3 vertex = vertices[i];
      daxa_f32 dist = dot(vertex, dir);
      if (dist > max_dist)
      {
        max_dist = dist;
        furthest_point = vertex;
      }
    }
    return furthest_point;
  }
};

struct SupportPoint
{
  daxa_f32vec3 point;  // Minkowski difference point
  daxa_f32vec3 pointA; // Original point from A
  daxa_f32vec3 pointB; // Original point from B

  __init()
  {
  }
  __init(const daxa_f32vec3 p, const daxa_f32vec3 a, const daxa_f32vec3 b)
  {
    point = p;
    pointA = a;
    pointB = b;
  }
};

struct Simplex
{
  const static daxa_u32 MAX_SUPPORT_POINTS = 4;
  SupportPoint points[MAX_SUPPORT_POINTS];

  daxa_u32 count;

  __init()
  {
    count = 0;
  }

  __init(const SupportPoint point)
  {
    count = 1;
    points[0] = point;
  }

  __init(const SupportPoint point1, const SupportPoint point2)
  {
    count = 2;
    points[0] = point1;
    points[1] = point2;
  }

  __init(const SupportPoint point1, const SupportPoint point2, const SupportPoint point3)
  {
    count = 3;
    points[0] = point1;
    points[1] = point2;
    points[2] = point3;
  }

  __init(const SupportPoint point1, const SupportPoint point2, const SupportPoint point3, const SupportPoint point4)
  {
    count = 4;
    points[0] = point1;
    points[1] = point2;
    points[2] = point3;
    points[3] = point4;
  }

  [mutating] void add(const SupportPoint point)
  {
    points[count++] = point;
  }

      [mutating] void remove(daxa_u32 index)
  {
    for (daxa_u32 i = index; i < count - 1; ++i)
    {
      points[i] = points[i + 1];
    }
    --count;
  }
};

struct Edge
{
  daxa_f32vec3 normal;
  float distance;
  // Relevant data for the edge
};

struct Polytope
{
  const static daxa_u32 MAX_SUPPORT_POINTS = 4;
  SupportPoint vertices[MAX_SUPPORT_POINTS];
  Edge edges[MAX_SUPPORT_POINTS];
  // Relevant data for the polytope
};

func support(const Ptr<RigidBody> r, const Ptr<RigidBody> r2, const daxa_f32vec3 dir) -> SupportPoint
{
  daxa_f32vec3 support_r1 = r.find_furthest_point(dir);
  daxa_f32vec3 support_r2 = r2.find_furthest_point(-dir);
  daxa_f32vec3 point = support_r1 - support_r2;
  return SupportPoint(point, support_r1, support_r2);
}

func GJK(const daxa_u32 rigid_body_index, Ptr<RigidBody> r,
         const Ptr<SimConfig> sim_config) -> void
{
  var any_collision = false;
  r.clear_flag(RigidBodyFlag::COLLIDING);
  for (daxa_u32 i = 0; i < sim_config->rigid_body_count; i++)
  {
    if (i == rigid_body_index)
    {
        continue;
    }
    Simplex simplex;
    Ptr<RigidBody> r2 = Ptr<RigidBody>(&p.task_head.rigid_bodies[i]);
    // daxa_u32 iter_count;
    // Verify collision between r and r2
    let collision = GJK_pair(r, r2, simplex);

    // EPA
    if (collision)
    {
      any_collision = true;
      // Compute the Minimum Translation Vector (MTV)
      // daxa_f32vec3 mtv = EPA(r, r2, simplex);
      InterlockedAdd(sim_config->collision_count, 1);
    }
  }

  if(any_collision)
  {
    r->flags |= RigidBodyFlag::COLLIDING;
  }
}

func GJK_pair(const Ptr<RigidBody> r, const Ptr<RigidBody> r2, out Simplex simplex) -> bool
{

  daxa_f32vec3 initial_dir = normalize(r2.position - r.position);
  if (length(initial_dir) < 0.0001f)
  {
    initial_dir = daxa_f32vec3(1, 0, 0); // Random initial direction
  }

  simplex.add(support(r, r2, initial_dir));

  daxa_f32vec3 dir = -simplex.points[0].point;

  const daxa_u32 MAX_ITERATIONS = 64;
  daxa_u32 iterations = 0;
  while (iterations++ < MAX_ITERATIONS)
  {
    SupportPoint new_point = support(r, r2, dir);

    if (dot(new_point.point, dir) < 0) // TODO: Check if equal to zero
    {
      return false; // There is no collision
    }

    simplex.add(new_point);

    if (handle_simplex(simplex, dir))
    {
      return true; // Collision detected
    }
  }

  return false;
}

// Function to handle the simplex based on its dimension
func handle_simplex(inout Simplex simplex, inout daxa_f32vec3 dir) -> bool
{
  switch (simplex.count)
  {
  case 2:
    // Segment: Check if the origin is inside the segment
    return handle_segment(simplex, dir);
  case 3:
    // Triangle: Check if the origin is inside the triangle
    return handle_triangle(simplex, dir);
  case 4:
    // Tetrahedron: Check if the origin is inside the tetrahedron
    return handle_tetrahedron(simplex, dir);
  }
  return false;
}

func handle_segment(inout Simplex simplex, inout daxa_f32vec3 dir) -> bool
{
  daxa_f32vec3 A = simplex.points[0].point;
  daxa_f32vec3 B = simplex.points[1].point;
  daxa_f32vec3 AB = B - A;
  daxa_f32vec3 AO = -A;

  if (dot(AB, AO) > 0)
  {
    // Origin is outside the segment
    dir = cross(cross(AB, AO), AB);
  }
  else
  {
    // Origin is inside the segment
    simplex.remove(1);
    dir = AO;
  }
  dir = normalize(dir);

  return false;
}

func handle_triangle(inout Simplex simplex, inout daxa_f32vec3 dir) -> bool
{
  daxa_f32vec3 A = simplex.points[0].point;
  daxa_f32vec3 B = simplex.points[1].point;
  daxa_f32vec3 C = simplex.points[2].point;
  daxa_f32vec3 AB = B - A;
  daxa_f32vec3 AC = C - A;
  daxa_f32vec3 AO = -A;

  daxa_f32vec3 ABC = cross(AB, AC);

  if (dot(cross(ABC, AC), AO) > 0)
  {
    if (dot(AC, AO) > 0)
    {
      simplex.remove(1);
      dir = cross(cross(AC, AO), AC);
    }
    else
    {
      simplex.remove(2);
      return handle_segment(simplex, dir);
    }
  }
  else
  {
    if (dot(cross(AB, ABC), AO) > 0)
    {
      simplex.remove(2);
      return handle_segment(simplex, dir);
    }
    else
    {
      if (dot(ABC, AO) > 0)
      {
        dir = ABC;
      }
      else
      {
        simplex = Simplex(simplex.points[0], simplex.points[2], simplex.points[1]);
        dir = -ABC;
      }
    }
  }
  dir = normalize(dir);

  return false;
}

func handle_tetrahedron(inout Simplex simplex, inout daxa_f32vec3 dir) -> bool
{
  daxa_f32vec3 A = simplex.points[0].point;
  daxa_f32vec3 B = simplex.points[1].point;
  daxa_f32vec3 C = simplex.points[2].point;
  daxa_f32vec3 D = simplex.points[3].point;

  daxa_f32vec3 AB = B - A;
  daxa_f32vec3 AC = C - A;
  daxa_f32vec3 AD = D - A;
  daxa_f32vec3 AO = -A;

  daxa_f32vec3 ABC = cross(AB, AC);
  daxa_f32vec3 ACD = cross(AC, AD);
  daxa_f32vec3 ADB = cross(AD, AB);

  if (dot(ABC, AO) > 0)
  {
    simplex.remove(3);
    return handle_triangle(simplex, dir);
  }
  else if (dot(ACD, AO) > 0)
  {
    simplex.remove(1);
    return handle_triangle(simplex, dir);
  }
  else if (dot(ADB, AO) > 0)
  {
    return handle_triangle(simplex = Simplex(simplex.points[0], simplex.points[3], simplex.points[1]), dir);
  }

  return true;
}

func EPA(const Ptr<RigidBody> r, const Ptr<RigidBody> r2, Simplex simplex) -> daxa_f32vec3
{
  // Construir el poliedro inicial a partir del simplex
  Polytope polytope;
  // Agregar vértices y caras al poliedro

  while (true)
  {
    // Encontrar la cara más cercana al origen
    Edge closest_edge = find_closest_edge(polytope);

    // Obtener un nuevo punto de soporte en la dirección de la normal de la cara
    SupportPoint new_point = support(r, r2, closest_edge.normal);

    float distance = dot(new_point.point, closest_edge.normal);
    float diff = distance - closest_edge.distance;

    if (diff < EPSILON)
    {
      // Convergencia alcanzada
      // Devolver el vector mínimo de traslación (MTV)
      return closest_edge.normal * distance;
    }
    else
    {
      // Expandir el poliedro con el nuevo punto
      // Actualizar caras y bordes
    }
  }
}

Edge find_closest_edge(const Polytope polytope)
{
  Edge closest_edge;
  // Inicializar con un valor grande
  closest_edge.distance = FLT_MAX;

  // Recorrer las caras del poliedro
  // Calcular la distancia al origen y actualizar 'closest_edge' si es más cercano

  return closest_edge;
}

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

  Ptr<RigidBody> rigid_body =
      Ptr<RigidBody>(&p.task_head.rigid_bodies[pixel_i.x]);

  GJK(pixel_i.x, rigid_body, sim_config);
}

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
    // rigid_body->advect(sim_config);
    r->position  = daxa_f32vec3(0.0, 0.4, 0.0);
  } else {
    r->position = daxa_f32vec3(0.5, 0.1, 0.1);
  }
}