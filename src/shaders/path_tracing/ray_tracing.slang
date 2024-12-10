#include "shared.inl"
#include "lighting.slang"

[[vk::push_constant]] RTPushConstants p;

[shader("raygeneration")] void ray_generation()
{
  const daxa_u32vec2 thread_idx = DispatchRaysIndex().xy;
  const daxa_u32vec2 rt_size = DispatchRaysDimensions().xy;

  // Initialize variables
  daxa_f32vec3 radiance = daxa_f32vec3(0.0, 0.0, 0.0);
  daxa_f32vec3 throughput = daxa_f32vec3(1.0, 1.0, 1.0);
  daxa_u32 depth = 0;

  var max_bounces = p.task_head.ray_tracing_config.max_bounces;
  var frame_count = p.task_head.ray_tracing_config.frame_count;
  var current_frame_index = p.task_head.ray_tracing_config.current_frame_index;
  var accumulate_flag = (p.task_head.ray_tracing_config.flags & 0x1u) != 0u;

  // Initialize the random seed for the pixel
  daxa_u32 seed = init_random_seed(thread_idx, daxa_u32(current_frame_index));

  // Camera setup
  daxa_f32mat4x4 inv_view = Ptr<CameraView>(p.task_head.camera).inv_view;
  daxa_f32mat4x4 inv_proj = Ptr<CameraView>(p.task_head.camera).inv_proj;
  daxa_f32 tMin = 0.0001f;
  daxa_f32 tMax = 10000.0f;

  // Light setup
  daxa_u32 light_count = p.task_head.ray_tracing_config.light_count;
  Ptr<Light> lights = Ptr<Light>(p.task_head.lights);

  // Rigid body setup
  Ptr<RigidBody> rigid_bodies = Ptr<RigidBody>(p.task_head.rigid_bodies);

  // Material setup
  Ptr<Material> materials = Ptr<Material>(p.task_head.materials);

  // Generate the primary ray
  RayDesc ray = create_ray(inv_view, inv_proj, thread_idx, rt_size, tMin, tMax);

  // Get the top-level acceleration structure
  var tlas = RaytracingAccelerationStructure::get(p.task_head.tlas);

  // **Path Tracing Loop**
  while (depth < max_bounces)
  {
    HitPayload payload = {};
    payload.seed = seed; // Pass the seed to the payload

    TraceRay(tlas, RAY_FLAG_NONE, ~0, 0, 0, 0, ray, payload);

    seed = payload.seed; // Update the seed after tracing

    if (!payload.hit)
    {
      // If no hit, accumulate background color and terminate
      radiance += throughput * daxa_f32vec3(0.95, 0.28, 0.059); // Background color
      break;
    }

    // Update throughput
    daxa_f32vec3 position = payload.position;
    daxa_f32vec3 normal = payload.normal;
    daxa_f32vec3 albedo = payload.albedo;
    daxa_u32 instance_index = payload.instance_index;

    // **Accumulate Emission**
    if (length(payload.emission) > 0.0f)
    {
      radiance += throughput * payload.emission;
    }

    // **Next Event Estimation with MIS**
    daxa_f32 pdf_light;
    daxa_f32vec3 light_dir;

    daxa_f32vec3 direct_lighting = compute_direct_lighting_cubes(light_count, lights, instance_index, rigid_bodies, materials, tlas, position, normal, albedo, seed, pdf_light, light_dir);

    if (pdf_light > 0.0f)
    {
      // Compute the BRDF PDF at the light direction
      daxa_f32 cos_theta = max(0.0f, dot(normal, light_dir));
      daxa_f32 pdf_brdf_light = cos_theta / PI;

      // Compute MIS weight using the power heuristic
      daxa_f32 mis_weight = (pdf_light) / (pdf_light + pdf_brdf_light);

      // Accumulate radiance with MIS weight
      radiance += throughput * direct_lighting * mis_weight;
    }

    // Sample a new direction (BRDF sampling)
    daxa_f32vec3 new_direction;
    daxa_f32 pdf_brdf;
    daxa_f32vec3 brdf;

    sample_lambertian(normal, albedo, new_direction, pdf_brdf, brdf, seed);

    // Update throughput for indirect lighting
    daxa_f32 cos_theta = max(0.0f, dot(normal, new_direction));
    throughput *= (brdf * cos_theta) / pdf_brdf;

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
      daxa_f32 max_throughput = max(throughput.r, max(throughput.g, throughput.b));
      daxa_f32 continue_probability = clamp(max_throughput, 0.1f, 1.0f);

      if (random_float(seed) > continue_probability)
      {
        break; // Terminate the path
      }

      throughput /= continue_probability;
    }
  }

  // Get the swapchain texture
  let swapchain = RWTexture2D<daxa_f32vec4>::get(p.task_head.swapchain);

  if (accumulate_flag)
  {
    // Get the accumulation buffer
    let accumulation_buffer = RWTexture2D<daxa_f32vec4>::get(p.task_head.accumulation_buffer);

    // Get the accumulated color
    daxa_f32vec4 accumulated_color = accumulation_buffer[thread_idx.xy];

    // Update the accumulated color by frame_count and add the new radiance
    daxa_f32vec3 average_radiance = (accumulated_color.rgb * daxa_f32(frame_count) + radiance) / daxa_f32(frame_count + 1);

    // Write the averaged color to the accumulation buffer
    accumulation_buffer[thread_idx.xy] = daxa_f32vec4(average_radiance, 1.0);

    // Apply gamma correction to the average radiance
    daxa_f32vec3 gamma_corrected_average = pow(average_radiance, daxa_f32vec3(1.0 / 2.2));

    // Write the averaged color to the output texture (swapchain)
    swapchain[thread_idx.xy] = daxa_f32vec4(gamma_corrected_average, 1.0);
  }
  else
  {
    // Apply gamma correction
    daxa_f32vec3 gamma_corrected_radiance = pow(radiance, daxa_f32vec3(1.0 / 2.2));

    // For real-time rendering without accumulation
    swapchain[thread_idx.xy] = daxa_f32vec4(gamma_corrected_radiance, 1.0);
  }
}

    [shader("miss")] void miss(inout HitPayload payload)
{
  payload.hit = false;
}

[shader("miss")] void miss_shadows(inout ShadowPayload payload)
{
  payload.hit = false;
}

static const daxa_f32vec3 light_intensity = daxa_f32vec3(2.5);
static const daxa_f32vec3 face_X = daxa_f32vec3(1.0, 0.0, 0.0);                   // red
static const daxa_f32vec3 face_neg_X = daxa_f32vec3(1.0, 1.0, 0.0);               // yellow
static const daxa_f32vec3 face_Y = daxa_f32vec3(0.0, 1.0, 0.0);                   // green
static const daxa_f32vec3 face_neg_Y = daxa_f32vec3(0.0, 1.0, 1.0);               // light blue
static const daxa_f32vec3 face_Z = daxa_f32vec3(0.0, 0.0, 1.0);                   // blue
static const daxa_f32vec3 face_neg_Z = daxa_f32vec3(1.0, 0.0, 1.0);               // purple
static const daxa_f32vec3 material_color = daxa_f32vec3(0.0, 0.0, 0.0);           // black
static const daxa_f32vec3 collision_material_color = daxa_f32vec3(1.0, 1.0, 1.0); // white

BoxFace get_face_by_normal(daxa_f32vec3 normal)
{
  daxa_f32vec3 abs_normal = abs(normal);
  daxa_f32 max_component = max(max(abs_normal.x, abs_normal.y), abs_normal.z);
  let sign = sign(normal);
  return (max_component == abs_normal.x) ? (sign.x > 0 ? BoxFace::RIGHT : BoxFace::LEFT) : (max_component == abs_normal.y) ? (sign.y > 0 ? BoxFace::TOP : BoxFace::BOTTOM)
                                                                                                                           : (sign.z > 0 ? BoxFace::FRONT : BoxFace::BACK);
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

daxa_f32vec3 get_color_by_island_index(daxa_u32 island_index)
{
  island_index %= 15;

  daxa_f32vec3 color = daxa_f32vec3(0.0);
  switch (island_index)
  {
  case 0:
    color = daxa_f32vec3(1.0, 0.0, 0.0); // red
    break;
  case 1:
    color = daxa_f32vec3(0.0, 1.0, 0.0); // green
    break;
  case 2:
    color = daxa_f32vec3(0.0, 0.0, 1.0); // blue
    break;
  case 3:
    color = daxa_f32vec3(1.0, 1.0, 0.0); // yellow
    break;
  case 4:
    color = daxa_f32vec3(1.0, 0.0, 1.0); // purple
    break;
  case 5:
    color = daxa_f32vec3(0.0, 1.0, 1.0); // light blue
  case 6:
    color = daxa_f32vec3(1.0, 0.5, 0.0); // orange
    break;
  case 7:
    color = daxa_f32vec3(0.5, 1.0, 0.0); // lime
    break;
  case 8:
    color = daxa_f32vec3(0.0, 1.0, 0.5); // mint
    break;
  case 9:
    color = daxa_f32vec3(0.0, 0.5, 1.0); // sky
    break;
  case 10:
    color = daxa_f32vec3(1.0, 0.0, 0.5); // pink
    break;
  case 11:
    color = daxa_f32vec3(0.5, 0.0, 1.0); // purple
    break;
  case 12:
    color = daxa_f32vec3(1.0, 1.0, 0.5); // yellow
    break;
  case 13:
    color = daxa_f32vec3(0.5, 1.0, 1.0); // light blue
    break;
  case 14:
    color = daxa_f32vec3(1.0, 0.5, 1.0); // purple
    break;
  default:
    color = daxa_f32vec3(0.0);
    break;
  }
  return color;
}

[shader("closesthit")] void closest_hit(inout HitPayload payload, in MyAttributes attr)
{
  // Get the hit information
  daxa_f32vec3 hit_location = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();
  daxa_u32 instance_index = InstanceID();
  daxa_u32 prim_index = PrimitiveIndex();

  Ptr<RigidBody> r = Ptr<RigidBody>(&p.task_head.rigid_bodies[instance_index]);
  Material m = p.task_head.materials[r->material_index];

  Ptr<Aabb> aabbs = Ptr<Aabb>(p.task_head.aabbs);

  // Compute the normal at the hit position
  daxa_f32vec3 normal = compute_normal(hit_location, r, prim_index, aabbs);

  // Set the payload
  payload.position = hit_location;
  payload.normal = normalize(normal);
  var ray_tracing_config = p.task_head.ray_tracing_config;

  if (ray_tracing_config.has_flag(RayTracingFlag::RT_SHOW_NORMALS))
  {
    // get face by normal
    BoxFace face = get_face_by_normal(normal);
    if (r->has_flag(RigidBodyFlag::COLLIDING))
    {
      if (r->is_face_colliding(face))
      {
        payload.albedo = collision_material_color;
      }
      else
      {
        payload.albedo = get_face_color(face);
      }
    }
    else
    {
      payload.albedo = get_face_color(face);
    }
  }
  else if (ray_tracing_config.has_flag(RayTracingFlag::RT_SHOW_ISLANDS))
  {
    if (r->island_index != MAX_U32)
    {
      daxa_u32 island_index = r->island_index;
      if (island_index != MAX_U32)
      {
        let contact_island_index = p.task_head.islands[island_index].contact_island_index;
        if (contact_island_index != MAX_U32)
        {
          // if(contact_island_index < 229) {
          let key = p.task_head.contact_islands[contact_island_index].key;
          if (key != MAX_U32)
          {
            payload.albedo = get_color_by_island_index(key);
          }
          else
          {
            payload.albedo = m.albedo;
          }
        }
        else
        {
          payload.albedo = m.albedo;
        }
      }
      else
      {
        payload.albedo = m.albedo;
      }
    }
    else
    {
      payload.albedo = m.albedo;
    }
  }
  else
  {
    payload.albedo = m.albedo;
  }
  payload.instance_index = instance_index;
  payload.primitive_index = prim_index;
  payload.emission = m.emission; // Add emission to the payload

  // No need to update the seed here

  payload.hit = true;
}

    [shader("intersection")] void intersection()
{
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

  if (THit > 0.0)
  {
    ReportHit(THit, /*hitKind*/ 0, params);
  }
}