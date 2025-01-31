#include "shared.inl"
#include "lighting.slang"

[[vk::push_constant]] RTPushConstants p;

[shader("raygeneration")] void ray_generation()
{
  const daxa::u32vec2 thread_idx = DispatchRaysIndex().xy;
  const daxa::u32vec2 rt_size = DispatchRaysDimensions().xy;

  // Initialize variables
  daxa_u32 depth = 0;
  // daxa::i32vec2 sample_shift = daxa::i32vec2(5, 7);

  let ray_tracing_config = p.task_head.ray_tracing_config;

  var max_bounces = ray_tracing_config.max_bounces;
  var frame_count = ray_tracing_config.frame_count;
  var current_frame_index = ray_tracing_config.current_frame_index;
  var accumulate_flag = (ray_tracing_config.flags & 0x1u) != 0u;

  // Initialize the random seed for the pixel
  daxa::u32 seed = init_random_seed(thread_idx, daxa::u32(current_frame_index));

  // Camera setup
  daxa::f32mat4x4 inv_view = Ptr<CameraView>(p.task_head.camera).inv_view;
  daxa::f32mat4x4 inv_proj = Ptr<CameraView>(p.task_head.camera).inv_proj;
  daxa::f32 tMin = 0.0001f;
  daxa::f32 tMax = 10000.0f;

  // Light setup
  daxa::u32 light_count = ray_tracing_config.light_count;
  let lights = (Light*)(p.task_head.lights);

  // Rigid body entries setup
  let rigid_body_map = (RigidBodyEntry*)(p.task_head.rigid_body_map);

  // Rigid body setup
  let rigid_bodies = (RigidBody*)(p.task_head.rigid_bodies);

  // Material setup
  let materials = (Material*)(p.task_head.materials);

  // // Island setup
  // let islands = (Island*)(p.task_head.islands);

  // // Contact Island setup
  // let contact_islands = (ContactIsland*)(p.task_head.contact_islands);

  // Texture3D<daxa::f32vec4> bluenoise_texture = Texture3D<daxa::f32vec4>.get(p.task_head.stbn_texture);

  // Generate the primary ray
  RayDesc ray = create_ray(inv_view, inv_proj, thread_idx, rt_size, tMin, tMax);

  // Get the top-level acceleration structure
  var tlas = RaytracingAccelerationStructure::get(p.task_head.tlas);
    
  HitPayload payload = {daxa::f32vec3(0), daxa::f32vec3(0), MAX_U32, MAX_U32, daxa::f32vec3(1.0), daxa::f32vec3(0.0)};

  // **Path Tracing Loop**
  while (depth < max_bounces)
  {
    // payload.seed = seed; // Pass the seed to the payload
    payload.instance_index = MAX_U32;
    payload.primitive_index = MAX_U32;

    TraceRay(tlas, RAY_FLAG_NONE, ~0, 0, 0, 0, ray, payload);

    // seed = payload.seed; // Update the seed after tracing

    if (payload.instance_index == MAX_U32 || payload.primitive_index == MAX_U32)
    {
      // If no hit, accumulate background color and terminate
      payload.radiance += payload.throughput * daxa_f32vec3(0.95, 0.28, 0.059); // Background color
      break;
    }

    let r = &rigid_bodies[rigid_body_map[payload.instance_index].index];
    Material m = materials[r->material_index];

    // Update throughput
    daxa_f32vec3 position = payload.position;
    daxa_f32vec3 normal = payload.normal;
    daxa_f32vec3 albedo = get_color(ray_tracing_config, normal, m.albedo, r);
    daxa::f32vec3 emission = m.emission;
    daxa_u32 instance_index = payload.instance_index;

    // **Accumulate Emission**
    if (length(emission) > 0.0f)
    {
      payload.radiance += payload.throughput * emission;
    }

    // **Next Event Estimation with MIS**
    daxa::f32 pdf_light;
    daxa::f32vec3 light_dir;

    daxa::f32vec3 direct_lighting = compute_direct_lighting_cubes(light_count, lights, instance_index, rigid_body_map, rigid_bodies, materials, tlas, position, normal, albedo, seed, pdf_light, light_dir);

    if (pdf_light > 0.0f)
    {
      // Compute the BRDF PDF at the light direction
      daxa::f32 cos_theta = max(0.0f, dot(normal, light_dir));
      daxa::f32 pdf_brdf_light = cos_theta / PI;

      // Compute MIS weight using the power heuristic
      daxa::f32 mis_weight = (pdf_light) / (pdf_light + pdf_brdf_light);

      // Accumulate radiance with MIS weight
      payload.radiance += payload.throughput * direct_lighting * mis_weight;
    }

    // Sample a new direction (BRDF sampling)
    daxa::f32vec3 new_direction;
    daxa::f32 pdf_brdf;
    daxa::f32vec3 brdf;

    sample_lambertian(normal, albedo, new_direction, pdf_brdf, brdf, seed);

    // daxa::i32 slice_index = daxa::i32(current_frame_index & 63);
    // daxa::i32vec2 pixel_index = (daxa::i32vec2(thread_idx.x, thread_idx.y) +  daxa::i32vec2(sample_shift * depth)) & daxa::i32vec2(127, 127);
    // daxa::f32vec4 bluenoise = bluenoise_texture[daxa::i32vec3(pixel_index, slice_index)];
    // // printf("bluenoise: %f %f %f %f pixel: %u, %u, slice: %d\n", bluenoise.x, bluenoise.y, bluenoise.z, bluenoise.w, thread_idx.x, thread_idx.y, slice_index);
    // sample_lambertian(normal, albedo, bluenoise.x, bluenoise.y, new_direction, pdf_brdf, brdf);

    // Update throughput for indirect lighting
    daxa::f32 cos_theta = max(0.0f, dot(normal, new_direction));
    payload.throughput *= (brdf * cos_theta) / pdf_brdf;

    // Prepare the next ray
    ray.Origin = position + normal * 0.001f; // Offset to avoid self-intersection
    ray.Direction = normalize(new_direction);
    ray.TMin = 0.0001f;
    ray.TMax = 10000.0f;

    // Increment depth
    depth++;

    // Russian roulette termination after a certain depth
    if (depth > 2)
    {
      daxa::f32 max_throughput = max(payload.throughput.r, max(payload.throughput.g, payload.throughput.b));
      daxa::f32 continue_probability = clamp(max_throughput, 0.1f, 1.0f);

      if (random_float(seed) > continue_probability)
      {
        break; // Terminate the path
      }

      payload.throughput /= continue_probability;
    }
  }

  // Get the swapchain texture
  let swapchain = RWTexture2D<daxa::f32vec4>::get(p.task_head.swapchain);

  if (accumulate_flag)
  {
    // Get the accumulation buffer
    let accumulation_buffer = RWTexture2D<daxa::f32vec4>::get(p.task_head.accumulation_buffer);

    // Get the accumulated color
    daxa::f32vec4 accumulated_color = accumulation_buffer[thread_idx.xy];

    // Update the accumulated color by frame_count and add the new radiance
    daxa::f32vec3 average_radiance = (accumulated_color.rgb * daxa::f32(frame_count) + payload.radiance) / daxa::f32(frame_count + 1);

    // Write the averaged color to the accumulation buffer
    accumulation_buffer[thread_idx.xy] = daxa::f32vec4(average_radiance, 1.0);

    // Apply gamma correction to the average radiance
    daxa::f32vec3 gamma_corrected_average = pow(average_radiance, daxa::f32vec3(1.0 / 2.2));

    // Write the averaged color to the output texture (swapchain)
    swapchain[thread_idx.xy] = daxa::f32vec4(gamma_corrected_average, 1.0);
  }
  else
  {
    // Apply gamma correction
    daxa::f32vec3 gamma_corrected_radiance = pow(payload.radiance, daxa::f32vec3(1.0 / 2.2));

    // For real-time rendering without accumulation
    swapchain[thread_idx.xy] = daxa::f32vec4(gamma_corrected_radiance, 1.0);
  }
}

    [shader("miss")] void miss(inout HitPayload payload)
{
  payload.instance_index = MAX_U32;
  payload.primitive_index = MAX_U32;
}

[shader("miss")] void miss_shadows(inout ShadowPayload payload)
{
  payload.hit = false;
}

[shader("closesthit")] void closest_hit(inout HitPayload payload, in MyAttributes attr)
{
  // Get the hit information
  daxa::f32vec3 hit_location = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();
  daxa::u32 instance_index = InstanceID();
  daxa::u32 prim_index = PrimitiveIndex();

  RigidBodyEntry* rigid_body_map = (RigidBodyEntry*)(p.task_head.rigid_body_map);

  RigidBody* r = (RigidBody*)(&p.task_head.rigid_bodies[rigid_body_map[instance_index].index]);
  // Material m = p.task_head.materials[r->material_index];

  Ptr<Aabb> aabbs = Ptr<Aabb>(p.task_head.aabbs);

  // Compute the normal at the hit position
  daxa::f32vec3 normal = compute_normal(hit_location, r, prim_index, aabbs);

  // Set the payload
  payload.position = hit_location;
  payload.normal = normalize(normal);
  payload.instance_index = instance_index;
  payload.primitive_index = prim_index;

  // No need to update the seed here
}

    [shader("intersection")] void intersection()
{
  daxa::f32 THit = RayTCurrent();
  MyAttributes params = {};

  daxa::u32 instance_index = InstanceID();
  daxa::u32 prim_index = PrimitiveIndex();

  RigidBodyEntry* rigid_body_map = (RigidBodyEntry*)(p.task_head.rigid_body_map);

  RigidBody* r = (RigidBody*)(&p.task_head.rigid_bodies[rigid_body_map[instance_index].index]);

  Ptr<Aabb> aabbs = Ptr<Aabb>(p.task_head.aabbs);
  Aabb aabb = r->get_aabb_by_index(prim_index, aabbs);

  Ray ray = {WorldRayOrigin(), WorldRayDirection()};
  ray.origin = r->world_to_object(ray.origin);
  ray.direction = normalize(r->rotate_vector_inverse(ray.direction));

  THit = hitAabb(aabb, ray);

  if (THit > 0.0)
  {
    ReportHit(THit, /*hitKind*/ 0, params);
  }
}

static const daxa::f32vec3 BVH_TINT = daxa::f32vec3(0.1, 0.3, 1.0);
static const daxa::f32vec3 BVH_LEAF_TINT = daxa::f32vec3(1.0, 0.3, 0.1);
static const daxa::f32vec3 BVH_ROOT_TINT = daxa::f32vec3(0.1, 1.0, 0.3);

[shader("anyhit")] void any_hit_LBVH(inout HitPayload payload, in MyAttributes attr)
{
  let instance_count = p.task_head.ray_tracing_config.instance_count;

  var color = BVH_TINT;
  if(attr.primitive_index == 0) {
    color = BVH_ROOT_TINT;
  }
  else if(attr.primitive_index >= instance_count - 1) {
    color = BVH_LEAF_TINT;
  }
  // printf("instance_index: %d, primitive_index: %d, instance_count: %d\n", attr.instance_index, attr.primitive_index, instance_count);

  payload.throughput *= 0.95f;
  payload.radiance += payload.throughput * color * attr.factor;

  IgnoreHit();
}

daxa::f32 get_opaque_factor(daxa::f32vec3 hit_pos, Aabb aabb)
{
    daxa::f32 factor = 0.0f;
    // find how close we are to each face
    daxa::f32 dist_x_min = abs(hit_pos.x - aabb.minimum.x);
    daxa::f32 dist_x_max = abs(aabb.maximum.x - hit_pos.x);
    daxa::f32 dist_y_min = abs(hit_pos.y - aabb.minimum.y);
    daxa::f32 dist_y_max = abs(aabb.maximum.y - hit_pos.y);
    daxa::f32 dist_z_min = abs(hit_pos.z - aabb.minimum.z);
    daxa::f32 dist_z_max = abs(aabb.maximum.z - hit_pos.z);

    daxa::u32 boundary_count = 0;
    daxa::f32 epsilon = 0.01f;
    if (dist_x_min < epsilon || dist_x_max < epsilon) boundary_count++;
    if (dist_y_min < epsilon || dist_y_max < epsilon) boundary_count++;
    if (dist_z_min < epsilon || dist_z_max < epsilon) boundary_count++;

    // Then pick a factor based on boundary_count (1=face, 2=edge, 3=corner).
    if (boundary_count == 1) factor = 0.05f;
    if (boundary_count == 2) factor = 0.35f;
    if (boundary_count == 3) factor = 0.5f;

    return factor;
}

[shader("intersection")] void intersection_LBVH()
{
  daxa_f32 THit = RayTCurrent();
  MyAttributes params = {InstanceID(), PrimitiveIndex()};
  // printf("instance_index: %d, primitive_index: %d\n", InstanceID(), PrimitiveIndex());

  let lbvh_nodes = (LBVHNode*)(p.task_head.lbvh_nodes);
  Aabb aabb = lbvh_nodes[params.primitive_index].aabb;

  Ray ray = {WorldRayOrigin(), WorldRayDirection()};
  // ray.origin = r->world_to_object(ray.origin);
  // ray.direction = normalize(r->rotate_vector_inverse(ray.direction));

  THit = hitAabb(aabb, ray);

  if (THit > 0.0)
  {
    daxa::f32vec3 hit_pos = ObjectRayOrigin() + ObjectRayDirection() * THit;
    params.factor = get_opaque_factor(hit_pos, aabb);

    ReportHit(THit, /*hitKind*/ 0, params);
  }
}