#include "shared.inl"

[[vk::push_constant]] RTPushConstants p;

[shader("raygeneration")] void ray_generation()
{
  const daxa_u32vec2 thread_idx = DispatchRaysIndex().xy;
  const daxa_u32vec2 rt_size = DispatchRaysDimensions().xy;

  HitPayload payload = {daxa_f32vec3(1.0, 1.0, 1.0), daxa_f32vec3(0.0, 0.0, 0.0), daxa_f32vec3(0.0, 0.0, 0.0)};

  // Camera setup
  daxa_f32mat4x4 inv_view = Ptr<CameraView>(p.task_head.camera).inv_view;
  daxa_f32mat4x4 inv_proj = Ptr<CameraView>(p.task_head.camera).inv_proj;
  daxa_f32 tMin = 0.0001;
  daxa_f32 tMax = 10000.0;

  RayDesc ray = create_ray(inv_view, inv_proj, thread_idx, rt_size, tMin, tMax);

  TraceRay(RaytracingAccelerationStructure::get(p.task_head.tlas), RAY_FLAG_NONE, ~0, 0, 0, 0, ray, payload);

  RWTexture2D<daxa_f32vec4>::get(p.task_head.swapchain)[thread_idx.xy] = daxa_f32vec4(payload.hit_value, 1.0);
}

[shader("miss")] void miss(inout HitPayload payload)
{
  payload.hit_value = daxa_f32vec3(0.0, 0.0, 0.05); // Background color
}
[shader("miss")] void miss_shadows(inout ShadowRayPayload payload)
{
  payload.shadow = 1.0f;
}

static const daxa_u32 LIGHT_NUMBER = 4;
static const daxa_f32vec3 light_position[LIGHT_NUMBER] = {daxa_f32vec3(2, 1, 2), daxa_f32vec3(-2, 1, 2), daxa_f32vec3(2, 1, -2) , daxa_f32vec3(-2, 1, -2)};
static const daxa_f32vec3 light_intensity = daxa_f32vec3(2.5);
static const daxa_f32vec3 face_X = daxa_f32vec3(1.0, 0.0, 0.0); // red
static const daxa_f32vec3 face_neg_X = daxa_f32vec3(1.0, 1.0, 0.0); // yellow
static const daxa_f32vec3 face_Y = daxa_f32vec3(0.0, 1.0, 0.0); // green
static const daxa_f32vec3 face_neg_Y = daxa_f32vec3(0.0, 1.0, 1.0); // light blue
static const daxa_f32vec3 face_Z = daxa_f32vec3(0.0, 0.0, 1.0); // blue
static const daxa_f32vec3 face_neg_Z = daxa_f32vec3(1.0, 0.0, 1.0); // purple
static const daxa_f32vec3 material_color = daxa_f32vec3(0.0, 0.0, 0.0); // black
static const daxa_f32vec3 collision_material_color = daxa_f32vec3(1.0, 1.0, 1.0); // white
static const daxa_f32vec3 point_color = daxa_f32vec3(0.5, 0.5, 0.5); // grey
static const daxa_f32vec3 material_specular = daxa_f32vec3(0.0, 0.5, 1.0);
static const daxa_f32 shininess = 10.0;

BoxFace get_face_by_normal(daxa_f32vec3 normal)
{
  daxa_f32vec3 abs_normal = abs(normal);
  daxa_f32 max_component = max(max(abs_normal.x, abs_normal.y), abs_normal.z);
  let sign = sign(normal);
  return (max_component == abs_normal.x) ? (sign.x > 0 ? BoxFace::RIGHT : BoxFace::LEFT) :
         (max_component == abs_normal.y) ? (sign.y > 0 ? BoxFace::TOP : BoxFace::BOTTOM) :
         (sign.z > 0 ? BoxFace::FRONT : BoxFace::BACK);
}

daxa_f32vec3 get_face_color(BoxFace face)
{
  switch (face)
  {
  case BoxFace::RIGHT:
    return face_X;
  case BoxFace::LEFT:
    return face_neg_X;
  case BoxFace::TOP:
    return face_Y;
  case BoxFace::BOTTOM:
    return face_neg_Y;
  case BoxFace::FRONT:
    return face_Z;
  case BoxFace::BACK:
    return face_neg_Z;
  default:
    return daxa_f32vec3(0.0);
  }
}

[shader("closesthit")] void closest_hit(inout HitPayload payload,
                                            in MyAttributes attr)
{
  daxa_f32vec3 hit_location = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();

  payload.position = hit_location;

  daxa_u32 instance_index = InstanceID();
  daxa_u32 prim_index = PrimitiveIndex();
  
  Ptr<RigidBody> r = Ptr<RigidBody>(&p.task_head.rigid_bodies[instance_index]);
  
  Ptr<Aabb> aabbs = Ptr<Aabb>(p.task_head.aabbs);
  Aabb aabb = r->get_aabb_by_index(prim_index, aabbs);

  daxa_f32vec3 center = aabb.center();
  center = r->object_to_world(center);

  daxa_f32vec3 color = material_color;


  // Computing the normal at hit position
  daxa_f32vec3 normal = normalize(hit_location - center);

  normal = r->rotate_vector_inverse(normal);

  // cube normal
  {
    daxa_f32vec3  abs_normal = abs(normal);
    daxa_f32 max_component = max(max(abs_normal.x, abs_normal.y), abs_normal.z);
    normal     = (max_component == abs_normal.x) ?
                 daxa_f32vec3(sign(normal.x), 0, 0) :
                 (max_component == abs_normal.y) ? daxa_f32vec3(0, sign(normal.y), 0) : daxa_f32vec3(0, 0, sign(normal.z));
  }

  // get face by normal
  BoxFace face = get_face_by_normal(normal);
#if defined(BB_RT_DEBUG)
  if(r->has_flag(RigidBodyFlag::COLLIDING))
  {
    if(r->is_face_colliding(face))
    {
      color = collision_material_color;
    } else {
      color = get_face_color(face);
    }
  } else {
    color = get_face_color(face);
  }
#else
    color = get_face_color(face);
#endif

  normal = r->rotate_vector(normal);

  payload.normal = normal;

  daxa_f32vec3 hit_value = daxa_f32vec3(0.0);

  for(daxa_u32 i = 0; i < LIGHT_NUMBER; i++) {
    // Vector toward the light
    daxa_f32vec3 L = normalize(light_position[i] - daxa_f32vec3(0));

    // Diffuse
    daxa_f32vec3 diffuse = compute_diffuse(color, normal, L);
    daxa_f32vec3 specular = daxa_f32vec3(0);
    daxa_f32 attenuation = 1.0f;

    // Tracing shadow ray only if the light is visible from the surface
    if (dot(normal, L) > 0)
    {
      daxa_f32vec3 ray_dir = L;
      RayDesc ray = {hit_location, T_MIN, ray_dir, T_MAX};
      daxa_u32 flags = RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_FORCE_OPAQUE  | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER;
      ShadowRayPayload shadowPayload = { 0.0f };

      TraceRay(RaytracingAccelerationStructure::get(p.task_head.tlas), flags, ~0, 1, 0, 0, ray, shadowPayload);

      let shadow_result = (shadowPayload.shadow);

      attenuation =  1.0 - shadow_result;
      // specular = compute_specular(shininess, material_specular, WorldRayDirection(), normal, L);
    }

    hit_value += daxa_f32vec3(light_intensity * attenuation * (diffuse + specular));
  }

  payload.hit_value = hit_value;
}

[shader("intersection")] void intersection() {
  daxa_f32 THit = RayTCurrent();
  MyAttributes params = {};
  
  daxa_u32 instance_index = InstanceID();
  daxa_u32 prim_index = PrimitiveIndex();

  Ptr<RigidBody> r = Ptr<RigidBody>(&p.task_head.rigid_bodies[instance_index]);

  Ptr<Aabb> aabbs = Ptr<Aabb>(p.task_head.aabbs);
  Aabb aabb = r->get_aabb_by_index(prim_index, aabbs);

  Ray ray = {WorldRayOrigin(), WorldRayDirection()};
  ray.origin = r->world_to_object(ray.origin);
  ray.direction = normalize(r->rotate_vector_inverse(ray.direction));

  THit = hitAabb(aabb, ray);

  if(THit > 0.0)
  {
      ReportHit(THit, /*hitKind*/ 0, params);
  }
}


[shader("closesthit")] void closest_hit_point(inout HitPayload payload,
                                            in MyAttributes attr)
{
  daxa_f32vec3 hit_location = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();

  payload.position = hit_location;

  daxa_u32 instance_index = InstanceID();
  daxa_u32 prim_index = PrimitiveIndex();
  
  Aabb aabb = p.task_head.point_aabbs[prim_index];

  daxa_f32vec3 center = aabb.center();
  // center = r->object_to_world(center);

  daxa_f32vec3 color = point_color;


  // Computing the normal at hit position
  daxa_f32vec3 normal = normalize(hit_location - center);

  // normal = r->rotate_vector_inverse(normal);

  // normal = r->rotate_vector(normal);

  payload.normal = normal;

  payload.hit_value = point_color;
}



[shader("intersection")] void intersection_point() {
  daxa_f32 THit = RayTCurrent();
  MyAttributes params = {};
  
  daxa_u32 prim_index = PrimitiveIndex();


  Aabb aabb = p.task_head.point_aabbs[prim_index];

  Ray ray = {WorldRayOrigin(), WorldRayDirection()};

  daxa_f32vec3 center = aabb.center();

  daxa_f32 radius = POINT_SIZE;

  THit = hitSphere(center, radius, ray);

  if(THit > 0.0)
  {
      ReportHit(THit, /*hitKind*/ 0, params);
  }
}