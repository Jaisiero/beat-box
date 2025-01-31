#include "shared.inl"

// Wang Hash Function
daxa_u32 wang_hash(daxa_u32 seed)
{
    seed = (seed ^ 61u) ^ (seed >> 16u);
    seed *= 9u;
    seed = seed ^ (seed >> 4u);
    seed *= 0x27d4eb2du;
    seed = seed ^ (seed >> 15u);
    return seed;
}

// Function to initialize the random seed
daxa_u32 init_random_seed(daxa_u32vec2 px, daxa_u32 frame_number)
{
    // Example: combine all bits so no simple linear correlation:
    daxa_u32 seed = px.x ^ (px.y * 0x9e3779b1u);  // 0x9e3779b1 = golden ratio
    seed ^= (frame_number + 0x7f4a7c15u) * 0x85ebca6bu;  // arbitrary prime combos
    return wang_hash(seed);
}

// XorShift RNG
daxa_u32 xorshift32(inout daxa_u32 state)
{
    if (state == 0) state = 1;
    state ^= state << 13u;
    state ^= state >> 17u;
    state ^= state << 5u;
    return state;
}

// Function to generate a random float between 0 and 1
daxa_f32 random_float(inout daxa_u32 state)
{
    daxa_u32 rnd = xorshift32(state);
    return rnd * 2.3283064365386963e-10f; // 1/2^32
}

// Function to sample a Lambertian diffuse reflection
void sample_lambertian(daxa_f32vec3 normal, daxa_f32vec3 albedo, out daxa_f32vec3 direction, out daxa_f32 pdf, out daxa_f32vec3 brdf, inout daxa_u32 seed)
{
    // Generate two random numbers
    daxa_f32 u1 = random_float(seed);
    daxa_f32 u2 = random_float(seed);

    // Generate a random direction in the hemisphere
    direction = random_in_hemisphere(normal, u1, u2);
    daxa_f32 cos_theta = max(0.0f, dot(normal, direction));
    pdf = cos_theta / PI; // Cosine-weighted PDF
    brdf = albedo / PI;   // Lambertian BRDF
}

// Function to sample a Lambertian diffuse reflection
void sample_lambertian(daxa_f32vec3 normal, daxa_f32vec3 albedo, daxa::f32 u1, daxa::f32 u2, out daxa_f32vec3 direction, out daxa_f32 pdf, out daxa_f32vec3 brdf)
{
    // Generate a random direction in the hemisphere
    direction = random_in_hemisphere(normal, u1, u2);
    daxa_f32 cos_theta = max(0.0f, dot(normal, direction));
    pdf = cos_theta / PI; // Cosine-weighted PDF
    brdf = albedo / PI;   // Lambertian BRDF
}

// Function to sample a cosine-weighted direction on the sphere
daxa_f32vec3 random_in_hemisphere(daxa_f32vec3 normal, daxa::f32 u1, daxa::f32 u2)
{

    daxa_f32 r = sqrt(u1);
    daxa_f32 theta = 2.0f * PI * u2;

    daxa_f32 x = r * cos(theta);
    daxa_f32 y = r * sin(theta);
    daxa_f32 z = sqrt(1.0f - u1);

    // Create an orthonormal basis (Tangent Space)
    daxa_f32vec3 up = abs(normal.z) < 0.999f ? daxa_f32vec3(0.0f, 0.0f, 1.0f) : daxa_f32vec3(1.0f, 0.0f, 0.0f);
    daxa_f32vec3 tangent = normalize(cross(up, normal));
    daxa_f32vec3 bitangent = cross(normal, tangent);

    // Transform sample to world space
    return normalize(tangent * x + bitangent * y + normal * z);
}

// Function to sample a random point on a unit sphere
daxa_f32vec3 random_point_on_sphere(inout daxa_u32 seed)
{
    daxa_f32 u = random_float(seed);
    daxa_f32 v = random_float(seed);

    daxa_f32 theta = 2.0f * PI * u;
    daxa_f32 phi = acos(2.0f * v - 1.0f);

    daxa_f32 x = sin(phi) * cos(theta);
    daxa_f32 y = sin(phi) * sin(theta);
    daxa_f32 z = cos(phi);

    return daxa_f32vec3(x, y, z);
}

daxa::f32vec3 compute_cube_normal(daxa::f32vec3 normal) {
    // Cube normal calculation
    daxa_f32vec3 abs_normal = abs(normal);
    daxa_f32 max_component = max(max(abs_normal.x, abs_normal.y), abs_normal.z);
    return (max_component == abs_normal.x) ? daxa_f32vec3(sign(normal.x), 0, 0) :
             (max_component == abs_normal.y) ? daxa_f32vec3(0, sign(normal.y), 0) :
                                               daxa_f32vec3(0, 0, sign(normal.z));
}

// Function to compute the normal at the hit point
daxa_f32vec3 compute_normal(daxa_f32vec3 hit_location, RigidBody* r, daxa_u32 prim_index, Ptr<Aabb> aabbs)
{
    Aabb aabb = r->get_aabb_by_index(prim_index, aabbs);

    daxa_f32vec3 center = aabb.center();
    center = r->object_to_world(center);

    daxa_f32vec3 normal = normalize(hit_location - center);
    normal = r->rotate_vector_inverse(normal);

    // Cube normal calculation
    normal = compute_cube_normal(normal);

    normal = r->rotate_vector(normal);
    return normal;
}

// Function to compute the normal at the hit point
daxa_f32vec3 compute_normal(daxa_f32vec3 hit_location, Aabb aabb)
{
    daxa_f32vec3 center = aabb.center();

    daxa_f32vec3 normal = normalize(hit_location - center);

    // Cube normal calculation
    normal = compute_cube_normal(normal);

    return normal;
}

// Define the AreaLight struct and initialize it
struct AreaLight
{
    daxa_f32vec3 position;
    daxa_f32 radius;
    daxa_f32vec3 intensity; // Emission intensity
};

static AreaLight area_light = { daxa_f32vec3(0.0f, 10.0f, 0.0f), 1.0f, daxa_f32vec3(100.0f) };

// Function to compute direct lighting with MIS for Area Light
daxa_f32vec3 compute_direct_lighting_area(
    RaytracingAccelerationStructure tlas,
    daxa_f32vec3 position,
    daxa_f32vec3 normal,
    daxa_f32vec3 albedo,
    inout daxa_u32 seed,
    out daxa_f32 pdf_light,
    out daxa_f32vec3 light_dir)
{
    daxa_f32vec3 light_radiance = daxa_f32vec3(0.0);

    // Sample a point on the area light (sphere for simplicity)
    daxa_f32vec3 light_sample_position = area_light.position + area_light.radius * random_point_on_sphere(seed);

    // Compute the direction to the light sample
    light_dir = light_sample_position - position;
    daxa_f32 distance_squared = dot(light_dir, light_dir);
    daxa_f32 distance = sqrt(distance_squared);
    light_dir /= distance; // Normalize

    // Compute the light's normal at the sampled point
    daxa_f32vec3 light_normal = normalize(light_sample_position - area_light.position);

    // Compute the geometric terms
    daxa_f32 N_dot_L = max(0.0f, dot(normal, light_dir));
    daxa_f32 cos_theta_light = max(0.0f, dot(-light_dir, light_normal));

    if (N_dot_L > 0.0f && cos_theta_light > 0.0f)
    {
        // Compute the PDF for area light sampling
        daxa_f32 area = 4.0f * PI * area_light.radius * area_light.radius; // Surface area of the sphere
        pdf_light = distance_squared / (area * cos_theta_light);

        // Check if the light is visible (shadow ray)
        RayDesc shadow_ray;
        shadow_ray.Origin = position + normal * 0.001f;
        shadow_ray.Direction = light_dir;
        shadow_ray.TMin = 0.001f;
        shadow_ray.TMax = distance - 0.002f;

        ShadowPayload shadow_payload = {};
        TraceRay(tlas, RAY_FLAG_NONE, ~0, 1, 0, 0, shadow_ray, shadow_payload);

        if (!shadow_payload.hit)
        {
            // Compute the BRDF
            daxa_f32vec3 brdf = albedo / PI; // Lambertian BRDF

            // Light emission
            daxa_f32vec3 emission = area_light.intensity;

            // Compute the final radiance contribution
            light_radiance = emission * brdf * N_dot_L * cos_theta_light / (distance_squared);
        }
        else
        {
            // Light is occluded
            light_radiance = daxa_f32vec3(0.0);
            pdf_light = 0.0f;
        }
    }
    else
    {
        // Light or surface normal facing away
        light_radiance = daxa_f32vec3(0.0);
        pdf_light = 0.0f;
    }

    return light_radiance;
}

// Function to compute direct lighting with a single point light
daxa_f32vec3 compute_direct_lighting_point(
    RaytracingAccelerationStructure tlas,
    daxa_f32vec3 position,
    daxa_f32vec3 normal,
    daxa_f32vec3 albedo,
    inout daxa_u32 seed,
    out daxa_f32 pdf_light,
    out daxa_f32vec3 light_dir)
{
    daxa_f32vec3 light_radiance = daxa_f32vec3(0.0);

    // Define the point light position and intensity
    daxa_f32vec3 point_light_position = daxa_f32vec3(0.0f, 5.0f, 0.0f);
    daxa_f32vec3 point_light_intensity = daxa_f32vec3(100.0f); // Radiance

    // Compute the direction to the point light
    light_dir = point_light_position - position;
    daxa_f32 distance_squared = dot(light_dir, light_dir);
    daxa_f32 distance = sqrt(distance_squared);
    light_dir /= distance; // Normalize

    // PDF for point light (delta function)
    pdf_light = 1.0f; // Not used in MIS for delta lights, but kept for consistency

    // Check if the light is visible (shadow ray)
    RayDesc shadow_ray;
    shadow_ray.Origin = position + normal * 0.001f;
    shadow_ray.Direction = light_dir;
    shadow_ray.TMin = 0.001f;
    shadow_ray.TMax = distance - 0.002f;

    ShadowPayload shadow_payload = {};
    TraceRay(tlas, RAY_FLAG_NONE, ~0, 1, 0, 0, shadow_ray, shadow_payload);

    if (!shadow_payload.hit)
    {
        // Compute the contribution
        daxa_f32 N_dot_L = max(0.0f, dot(normal, light_dir));
        daxa_f32vec3 brdf = albedo / PI; // Lambertian BRDF

        // Point light emission
        daxa_f32vec3 emission = point_light_intensity;

        // Direct lighting contribution
        light_radiance = emission * brdf * N_dot_L / pdf_light;
    }
    else
    {
        light_radiance = daxa_f32vec3(0.0);
        pdf_light = 0.0f;
    }

    return light_radiance;
}

// Function to select a random light from the light buffer, try to get a light that is not the current instance
daxa_u32 select_random_light(inout daxa_u32 seed, daxa_u32 light_count, daxa_u32 instance_index)
{
    daxa_f32 u = random_float(seed);
    daxa_f32 index_f = u * daxa_f32(light_count-1);

    // Try to get a light that is not the current instance
    daxa_u32 index = daxa_u32(floor(index_f));
    if (index == instance_index && light_count > 1)
    {
        if(index < light_count - 1)
        {
            index++;
        }
        else
        {
            index--;
        }
    }

    if(index == instance_index) {
        index = MAX_U32;
    }

    return index;
}

// Sample a point on the cube's surface with a "cosine-weighted" distribution
// with respect to the shading position. In other words, faces more orthogonal
// to the view vector get picked more often.
daxa_f32vec3 cosine_weighted_sample_on_cube(
    inout daxa_u32 seed,
    RigidBody* r,
    daxa_f32vec3 shading_position,
    out daxa_f32vec3 light_normal,
    daxa_f32 half_size)
{
    // 1) Define the 6 faces of the unit cube in local space
    const daxa_u32 FACE_COUNT = 6u;
    struct FaceDef
    {
        daxa_f32vec3 normal_local;
        daxa_f32vec3 center_local;
    };

    FaceDef faces[FACE_COUNT] =
    {
        // +X
        { daxa_f32vec3(+1.0f,  0.0f,  0.0f), daxa_f32vec3(+half_size,  0.0f,        0.0f) },
        // -X
        { daxa_f32vec3(-1.0f,  0.0f,  0.0f), daxa_f32vec3(-half_size,  0.0f,        0.0f) },
        // +Y
        { daxa_f32vec3( 0.0f, +1.0f,  0.0f), daxa_f32vec3( 0.0f,       +half_size,  0.0f) },
        // -Y
        { daxa_f32vec3( 0.0f, -1.0f,  0.0f), daxa_f32vec3( 0.0f,       -half_size,  0.0f) },
        // +Z
        { daxa_f32vec3( 0.0f,  0.0f, +1.0f), daxa_f32vec3( 0.0f,       0.0f,        +half_size) },
        // -Z
        { daxa_f32vec3( 0.0f,  0.0f, -1.0f), daxa_f32vec3( 0.0f,       0.0f,        -half_size) },
    };

    // 2) Compute the "weight" for each face = max(0, dot(face_normal, view_dir)) * face_area
    //    face_area for one face = side^2 = (2*half_size)^2 = 4 * half_size^2
    const daxa_f32 face_area = 4.0f * (half_size * half_size);

    daxa_f32 weights[FACE_COUNT];
    daxa_f32 total_weight = 0.0f;

    for(daxa_u32 i = 0; i < FACE_COUNT; i++)
    {
        // Rotate local normal to world space
        daxa_f32vec3 normal_w = normalize(r->rotate_vector(faces[i].normal_local));

        // Face center in world space
        daxa_f32vec3 center_w = r->object_to_world(faces[i].center_local);

        // Vector from face center to shading position
        daxa_f32vec3 to_shading = shading_position - center_w;
        daxa_f32 length_shading = length(to_shading);

        if(length_shading < 1e-9f)
        {
            // Degenerate: shading point is basically *on* the face center
            // Weight = 0 in this scenario
            weights[i] = 0.0f;
            continue;
        }

        daxa_f32vec3 dir = to_shading / length_shading;
        daxa_f32 dot_val  = dot(normal_w, dir);

        // If dot_val <= 0, face is back-facing => weight = 0
        // Otherwise weight = dot_val * face_area
        daxa_f32 w = max(0.0f, dot_val) * face_area;
        weights[i] = w;
        total_weight += w;
    }

    // 3) If all weights are zero => entire cube is back-facing or very close => return zero
    if(total_weight < 1e-12f)
    {
        light_normal = daxa_f32vec3(0.0f);
        return daxa_f32vec3(0.0f);
    }

    // 4) Pick a face by sampling a random in [0 .. total_weight)
    daxa_f32 rrand = random_float(seed) * total_weight;
    daxa_u32 chosen_index = 0u;

    // Walk through the face weights until we find the face
    daxa_f32 accum = 0.0f;
    for(daxa_u32 i = 0; i < FACE_COUNT; i++)
    {
        accum += weights[i];
        if(rrand <= accum)
        {
            chosen_index = i;
            break;
        }
    }

    // 5) Uniformly sample the chosen face in local coords
    daxa_f32 u2 = random_float(seed);
    daxa_f32 u3 = random_float(seed);

    daxa_f32vec3 sample_local;
    switch(chosen_index)
    {
    case 0: // +X
        sample_local = daxa_f32vec3(
            +half_size,
            (u2 - 0.5f) * 2.0f * half_size,
            (u3 - 0.5f) * 2.0f * half_size
        );
        break;
    case 1: // -X
        sample_local = daxa_f32vec3(
            -half_size,
            (u2 - 0.5f) * 2.0f * half_size,
            (u3 - 0.5f) * 2.0f * half_size
        );
        break;
    case 2: // +Y
        sample_local = daxa_f32vec3(
            (u2 - 0.5f) * 2.0f * half_size,
            +half_size,
            (u3 - 0.5f) * 2.0f * half_size
        );
        break;
    case 3: // -Y
        sample_local = daxa_f32vec3(
            (u2 - 0.5f) * 2.0f * half_size,
            -half_size,
            (u3 - 0.5f) * 2.0f * half_size
        );
        break;
    case 4: // +Z
        sample_local = daxa_f32vec3(
            (u2 - 0.5f) * 2.0f * half_size,
            (u3 - 0.5f) * 2.0f * half_size,
            +half_size
        );
        break;
    default: // 5 => -Z
        sample_local = daxa_f32vec3(
            (u2 - 0.5f) * 2.0f * half_size,
            (u3 - 0.5f) * 2.0f * half_size,
            -half_size
        );
        break;
    }

    // 6) Transform the sample point & normal back to world space
    daxa_f32vec3 normal_local = faces[chosen_index].normal_local;
    light_normal = normalize(r->rotate_vector(normal_local));
    daxa_f32vec3 sample_point = r->object_to_world(sample_local);

    return sample_point;
}

// Function to compute direct lighting with MIS for multiple Cube Lights using cosine-weighted sampling
daxa_f32vec3 compute_direct_lighting_cubes(
    daxa_u32 light_count,
    Ptr<Light> lights,
    daxa_u32 instance_index,
    RigidBodyEntry* rigid_body_map,
    RigidBody* rigid_bodies,
    Ptr<Material> materials,
    RaytracingAccelerationStructure tlas,
    daxa_f32vec3 position, // world space position
    daxa_f32vec3 normal,
    daxa_f32vec3 albedo,
    inout daxa_u32 seed,
    out daxa_f32 pdf_light,
    out daxa_f32vec3 light_dir)
{
    daxa_f32vec3 light_radiance = daxa_f32vec3(0.0);
    pdf_light = 0.0f; // Initialize PDF
    
    if (light_count == 0)
    {
        pdf_light = 0.0f;
        return light_radiance;
    }
    
    // Select a random light
    daxa_u32 selected_light_index = select_random_light(seed, light_count, instance_index);
    if(selected_light_index == MAX_U32)
    {
        // printf("No valid light found. Instance Index: %u, Light Count: %u\n", instance_index, light_count);
        pdf_light = 0.0f;
        return light_radiance;
    }

    daxa_u32 rigid_body_id = lights[selected_light_index].rigid_body_id;
    daxa::u32 rigid_body_index = rigid_body_map[rigid_body_id].index;
    
    // Step 2: Retrieve rigid body properties (position, rotation, material index)rigid_bodies
    RigidBody* r = (RigidBody*)(&rigid_bodies[rigid_body_index]);
    Material m = materials[r->material_index];

    if(all(m.emission <= 0.0f))
    {
        printf("Light is non-emissive. Instance Index: %u, Light Count: %u, Rigid Body Index: %u emission: %f %f %f\n", instance_index, light_count, rigid_body_index, m.emission.x, m.emission.y, m.emission.z);
        // Light is non-emissive; skip processing
        pdf_light = 0.0f;
        return daxa_f32vec3(0.0);
    }
    
    daxa_f32 half_size = r->get_half_size(0);  // Assuming uniform scaling
    
    
    // Sample a point on the cube's surface with cosine-weighted distribution & compute the light's normal at the sampled point
    daxa_f32vec3 light_normal = daxa_f32vec3(0.0f);
    daxa_f32vec3 light_sample_position =
        cosine_weighted_sample_on_cube(seed, r, position, light_normal, half_size);

    // If uniform_sample_on_cube() found no forward-facing face, skip
    if (all(light_normal == 0.0f))
    {
        // printf("No forward-facing face found. Instance Index: %u, Light Count: %u, Rigid Body Index: %u\n", instance_index, light_count, rigid_body_index);
        pdf_light = 0.0f;
        return daxa_f32vec3(0.0f);
    }

    // Compute direction to the sampled point on the light
    daxa_f32vec3 light_dir = light_sample_position - position;
    daxa_f32 distance_squared = dot(light_dir, light_dir);
    daxa_f32 distance = sqrt(distance_squared);
    light_dir /= distance; // normalize

    // Geometry terms
    daxa_f32 N_dot_L        = max(0.0f, dot(normal, light_dir));
    daxa_f32 cos_theta_light= max(0.0f, dot(-light_dir, light_normal));

    // If either side is facing away too far, no contribution
    if (N_dot_L < EPSILON || cos_theta_light < EPSILON)
    {
        // printf("N_dot_L or cos_theta_light too small. Instance Index: %u, Light Count: %u, Rigid Body Index: %u\n", instance_index, light_count, rigid_body_index);
        pdf_light = 0.0f;
        return daxa_f32vec3(0.0f);
    }

    // Compute area & PDF
    // The total surface area of the cube is 6 * side^2, with side=2*half_size
    daxa_f32 side    = 2.0f * half_size;
    daxa_f32 area    = 6.0f * (side * side);
    // We are sampling exactly one light among 'light_count' lights
    // AND sampling the cube’s surface uniformly. The usual area-lights formula gives:
    // PDF = (1 / light_count) * (distance^2 / (area * cos_theta_light))
    pdf_light = (1.0f / daxa_f32(light_count)) * (distance_squared / (area * cos_theta_light));

    // Shadow test (ray from shading point to the light sample)
    RayDesc shadow_ray;
    shadow_ray.Origin    = position + normal * 0.001f;
    shadow_ray.Direction = light_dir;
    shadow_ray.TMin      = 0.001f;
    shadow_ray.TMax      = distance - 0.002f;

    // If 'hit' is false, that means the path is clear; if 'hit' is true, we are occluded
    ShadowPayload shadow_payload = { false };
    TraceRay(
        tlas,
        RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER,
        ~0,          // cull mask
        1,           // sbtRecordOffset
        0,           // sbtRecordStride
        0,           // missShaderIndex
        shadow_ray,
        shadow_payload
    );

    // If the ray is blocked, skip accumulation
    if (shadow_payload.hit)
    {
        printf("Shadow ray is occluded. Instance Index: %u, Light Count: %u, Rigid Body Index: %u\n", instance_index, light_count, rigid_body_index);
        // occluded => no direct light
        // pdf is still valid but the contribution is zero
        return daxa_f32vec3(0.0f);
    }

    // Otherwise, accumulate direct lighting from the sampled point
    //    - Lambertian BRDF = (albedo / PI)
    //    - Emission from the cube's material
    daxa_f32vec3 brdf     = albedo / PI;
    daxa_f32vec3 emission = m.emission;

    // geometry factor = (N_dot_L * cos_theta_light) / distance^2
    daxa_f32 geometry_factor = (N_dot_L * cos_theta_light) / distance_squared;

    // final contribution
    daxa_f32vec3 contribution = emission * brdf * geometry_factor;

    return contribution;
}

static const daxa::f32vec3 face_X = daxa::f32vec3(1.0, 0.0, 0.0);                   // red
static const daxa::f32vec3 face_neg_X = daxa::f32vec3(1.0, 1.0, 0.0);               // yellow
static const daxa::f32vec3 face_Y = daxa::f32vec3(0.0, 1.0, 0.0);                   // green
static const daxa::f32vec3 face_neg_Y = daxa::f32vec3(0.0, 1.0, 1.0);               // light blue
static const daxa::f32vec3 face_Z = daxa::f32vec3(0.0, 0.0, 1.0);                   // blue
static const daxa::f32vec3 face_neg_Z = daxa::f32vec3(1.0, 0.0, 1.0);               // purple
static const daxa::f32vec3 material_color = daxa::f32vec3(0.0, 0.0, 0.0);           // black
static const daxa::f32vec3 collision_material_color = daxa::f32vec3(0.1, 0.1, 0.1); // white

BoxFace get_face_by_normal(daxa::f32vec3 normal)
{
  daxa::f32vec3 abs_normal = abs(normal);
  daxa::f32 max_component = max(max(abs_normal.x, abs_normal.y), abs_normal.z);
  let sign = sign(normal);
  return (max_component == abs_normal.x) ? (sign.x > 0 ? BoxFace::RIGHT : BoxFace::LEFT) : (max_component == abs_normal.y) ? (sign.y > 0 ? BoxFace::TOP : BoxFace::BOTTOM)
                                                                                                                           : (sign.z > 0 ? BoxFace::FRONT : BoxFace::BACK);
}

daxa::f32vec3 get_face_color(BoxFace face)
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
    return daxa::f32vec3(0.0);
  }
}

daxa::f32vec3 get_color_by_island_index(daxa::u32 island_index)
{
  island_index %= 15;

  daxa::f32vec3 color = daxa::f32vec3(0.0);
  switch (island_index)
  {
  case 0:
    color = daxa::f32vec3(1.0, 0.0, 0.0); // red
    break;
  case 1:
    color = daxa::f32vec3(0.0, 1.0, 0.0); // green
    break;
  case 2:
    color = daxa::f32vec3(0.0, 0.0, 1.0); // blue
    break;
  case 3:
    color = daxa::f32vec3(1.0, 1.0, 0.0); // yellow
    break;
  case 4:
    color = daxa::f32vec3(1.0, 0.0, 1.0); // purple
    break;
  case 5:
    color = daxa::f32vec3(0.0, 1.0, 1.0); // light blue
  case 6:
    color = daxa::f32vec3(1.0, 0.5, 0.0); // orange
    break;
  case 7:
    color = daxa::f32vec3(0.5, 1.0, 0.0); // lime
    break;
  case 8:
    color = daxa::f32vec3(0.0, 1.0, 0.5); // mint
    break;
  case 9:
    color = daxa::f32vec3(0.0, 0.5, 1.0); // sky
    break;
  case 10:
    color = daxa::f32vec3(1.0, 0.0, 0.5); // pink
    break;
  case 11:
    color = daxa::f32vec3(0.5, 0.0, 1.0); // purple
    break;
  case 12:
    color = daxa::f32vec3(1.0, 1.0, 0.5); // yellow
    break;
  case 13:
    color = daxa::f32vec3(0.5, 1.0, 1.0); // light blue
    break;
  case 14:
    color = daxa::f32vec3(1.0, 0.5, 1.0); // purple
    break;
  default:
    color = daxa::f32vec3(0.0);
    break;
  }
  return color;
}

daxa::f32vec3 get_color(RayTracingConfig* config, daxa::f32vec3 normal, daxa::f32vec3 albedo, RigidBody* r)
{
    daxa::f32vec3 color = albedo;

    if (config->has_flag(RayTracingFlag::RT_SHOW_NORMALS))
    {
        // get face by normal
        BoxFace face = get_face_by_normal(normal);
        color = get_face_color(face);
    }
    else if(config->has_flag(RayTracingFlag::RT_SHOW_COLLISIONS))
    {
        if (r->has_flag(RigidBodyFlag::COLLIDING))
        {
            // get face by normal
            BoxFace face = get_face_by_normal(normal);
            if (r->is_face_colliding(face))
            {
                color = collision_material_color;
            }
        }
    }
    else if (config->has_flag(RayTracingFlag::RT_SHOW_ISLANDS))
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
                        color = get_color_by_island_index(key);
                    }
                }
            }
        }
    }

    return color;
}