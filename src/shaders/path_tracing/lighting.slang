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
daxa_u32 init_random_seed(daxa_u32vec2 pixel_coords, daxa_u32 frame_number)
{
    daxa_u32 seed = pixel_coords.x + pixel_coords.y * 12345u + frame_number * 6789u;
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
    // Generate a random direction in the hemisphere
    direction = random_in_hemisphere(normal, seed);
    daxa_f32 cos_theta = max(0.0f, dot(normal, direction));
    pdf = cos_theta / PI; // Cosine-weighted PDF
    brdf = albedo / PI;   // Lambertian BRDF
}

// Function to sample a cosine-weighted direction on the sphere
daxa_f32vec3 random_in_hemisphere(daxa_f32vec3 normal, inout daxa_u32 seed)
{
    daxa_f32 u1 = random_float(seed);
    daxa_f32 u2 = random_float(seed);

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

// Function to compute the normal at the hit point
daxa_f32vec3 compute_normal(daxa_f32vec3 hit_location, Ptr<RigidBody> r, daxa_u32 prim_index, Ptr<Aabb> aabbs)
{
    Aabb aabb = r->get_aabb_by_index(prim_index, aabbs);

    daxa_f32vec3 center = aabb.center();
    center = r->object_to_world(center);

    daxa_f32vec3 normal = normalize(hit_location - center);
    normal = r->rotate_vector_inverse(normal);

    // Cube normal calculation
    daxa_f32vec3 abs_normal = abs(normal);
    daxa_f32 max_component = max(max(abs_normal.x, abs_normal.y), abs_normal.z);
    normal = (max_component == abs_normal.x) ? daxa_f32vec3(sign(normal.x), 0, 0) :
             (max_component == abs_normal.y) ? daxa_f32vec3(0, sign(normal.y), 0) :
                                               daxa_f32vec3(0, 0, sign(normal.z));

    normal = r->rotate_vector(normal);
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
        bool approved = false;
        daxa_u32 max_attempts = 10;
        while(!approved && max_attempts > 0)
        {
            index = daxa_u32(floor(random_float(seed) * daxa_f32(light_count-1)));
            if(index != instance_index)
            {
                approved = true;
            }
            max_attempts--;
        }
    }

    if(index == instance_index) {
        index = MAX_U32;
    }

    return index;
}

// Function to sample a point on the cube's surface with cosine-weighted distribution towards shading point
daxa_f32vec3 uniform_sample_on_cube(
    inout daxa_u32 seed,
    Ptr<RigidBody> r,
    daxa_f32vec3 shading_position,
    out daxa_f32vec3 light_normal,
    daxa_f32 half_size)
{
    const daxa_u32 max_attempts = 4; // Maximum number of sampling attempts
    daxa_f32vec3 sample_point = daxa_f32vec3(0.0f);
    light_normal = daxa_f32vec3(0.0f);
    
    // Compute the vector from the shading point to the light's center
    daxa_f32vec3 light_center = r->position;
    
    for(daxa_u32 i = 0; i < max_attempts; i++)
    {
        // Step 1: Select a face uniformly
        daxa_f32 u1 = random_float(seed);
        daxa_f32 face_selection = u1 * 6.0f; // 6 faces
        daxa_u32 face = daxa_u32(floor(face_selection));
        
        // Step 2: Get face normal based on the selected face
        daxa_f32vec3 face_normal = daxa_f32vec3(0.0f);
        daxa_f32vec3 center_face = daxa_f32vec3(0.0f);
        
        switch (face)
        {
            case 0: // +X
                face_normal = daxa_f32vec3(1.0f, 0.0f, 0.0f);
                center_face = daxa_f32vec3(half_size, 0.0f, 0.0f);
                break;
            case 1: // -X
                face_normal = daxa_f32vec3(-1.0f, 0.0f, 0.0f);
                center_face = daxa_f32vec3(-half_size, 0.0f, 0.0f);
                break;
            case 2: // +Y
                face_normal = daxa_f32vec3(0.0f, 1.0f, 0.0f);
                center_face = daxa_f32vec3(0.0f, half_size, 0.0f);
                break;
            case 3: // -Y
                face_normal = daxa_f32vec3(0.0f, -1.0f, 0.0f);
                center_face = daxa_f32vec3(0.0f, -half_size, 0.0f);
                break;
            case 4: // +Z
                face_normal = daxa_f32vec3(0.0f, 0.0f, 1.0f);
                center_face = daxa_f32vec3(0.0f, 0.0f, half_size);
                break;
            case 5: // -Z
                face_normal = daxa_f32vec3(0.0f, 0.0f, -1.0f);
                center_face = daxa_f32vec3(0.0f, 0.0f, -half_size);
                break;
            default:
                face_normal = daxa_f32vec3(0.0f, 0.0f, 0.0f);
                center_face = daxa_f32vec3(0.0f);
                break;
        }
        // Step 3: Transform face_normal to world space
        daxa_f32vec3 face_normal_world = normalize(r->rotate_vector(face_normal));

        daxa_f32vec3 world_position = r->object_to_world(center_face);
        
        // Compute the vector from the shading point to the light's center
        daxa_f32vec3 V = normalize(shading_position - world_position);
        
        // Recompute dot_product in world space
        daxa_f32 dot_product = dot(face_normal_world, V);

        if(dot_product > 0.0f)
        {
            // Step 4: Sample a point on the selected face uniformly
            daxa_f32 u2 = random_float(seed);
            daxa_f32 u3 = random_float(seed);
            
            switch (face)
            {
                case 0: // +X
                    sample_point = daxa_f32vec3(half_size, (u2 - 0.5f) * 2.0f * half_size, (u3 - 0.5f) * 2.0f * half_size);
                    break;
                case 1: // -X
                    sample_point = daxa_f32vec3(-half_size, (u2 - 0.5f) * 2.0f * half_size, (u3 - 0.5f) * 2.0f * half_size);
                    break;
                case 2: // +Y
                    sample_point = daxa_f32vec3((u2 - 0.5f) * 2.0f * half_size, half_size, (u3 - 0.5f) * 2.0f * half_size);
                    break;
                case 3: // -Y
                    sample_point = daxa_f32vec3((u2 - 0.5f) * 2.0f * half_size, -half_size, (u3 - 0.5f) * 2.0f * half_size);
                    break;
                case 4: // +Z
                    sample_point = daxa_f32vec3((u2 - 0.5f) * 2.0f * half_size, (u3 - 0.5f) * 2.0f * half_size, half_size);
                    break;
                case 5: // -Z
                    sample_point = daxa_f32vec3((u2 - 0.5f) * 2.0f * half_size, (u3 - 0.5f) * 2.0f * half_size, -half_size);
                    break;
                default:
                    sample_point = daxa_f32vec3(0.0f);
                    break;
            }
            
            // Step 5: Transform the sample point to world space
            sample_point = r->object_to_world(sample_point);
            
            // Step 6: Assign the rotated normal based on the face
            light_normal = face_normal_world;
            
            return sample_point;
        }
    }
    
    // After the loop: If no valid face is found, return a zero vector
    light_normal = daxa_f32vec3(0.0f);
    return daxa_f32vec3(0.0f);
}

// Function to compute direct lighting with MIS for multiple Cube Lights using cosine-weighted sampling
daxa_f32vec3 compute_direct_lighting_cubes(
    daxa_u32 light_count,
    Ptr<Light> lights,
    daxa_u32 instance_index,
    Ptr<RigidBody> rigid_bodies,
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
    
    // Step 1: Select a random light
    daxa_u32 selected_light_index = select_random_light(seed, light_count, instance_index);
    if(selected_light_index == MAX_U32)
    {
        // printf("No valid light found. Instance Index: %u, Light Count: %u\n", instance_index, light_count);
        pdf_light = 0.0f;
        return light_radiance;
    }

    daxa_u32 rigid_body_index = lights[selected_light_index].rigid_body_index;
    
    // Step 2: Retrieve rigid body properties (position, rotation, material index)rigid_bodies
    Ptr<RigidBody> r = Ptr<RigidBody>(&rigid_bodies[rigid_body_index]);
    Material m = materials[r->material_index];

    if(all(m.emission <= 0.0f))
    {
        printf("Light is non-emissive. Instance Index: %u, Light Count: %u, Rigid Body Index: %u emission: %f %f %f\n", instance_index, light_count, rigid_body_index, m.emission.x, m.emission.y, m.emission.z);
        // Light is non-emissive; skip processing
        pdf_light = 0.0f;
        return daxa_f32vec3(0.0);
    }
    
    daxa_f32 half_size = r->get_half_size(0);  // Assuming uniform scaling
    
    
    // Step 3: Sample a point on the cube's surface with cosine-weighted distribution & compute the light's normal at the sampled point
    daxa_f32vec3 light_normal = daxa_f32vec3(0.0f);
    daxa_f32vec3 light_sample_position = uniform_sample_on_cube(seed, r, position, light_normal, half_size);

    // If no valid face was found, skip radiance accumulation
    if(all(light_normal == 0.0f))
    {
        pdf_light = 0.0f;
        return light_radiance;
    }
    
    // Step 4: Compute the direction to the light sample
    light_dir = light_sample_position - position;
    daxa_f32 distance_squared = dot(light_dir, light_dir);
    daxa_f32 distance = sqrt(distance_squared);
    light_dir /= distance; // Normalize
    
     // Step 5: Compute geometric terms
    daxa_f32 N_dot_L = max(0.0f, dot(normal, light_dir));
    daxa_f32 cos_theta_light = max(0.0f, dot(-light_dir, light_normal));
    
    if (N_dot_L > EPSILON && cos_theta_light > EPSILON)
    {
        daxa_f32 area = 6.0f * (2.0f * half_size) * (2.0f * half_size); // Surface area of the cube: 6 * side^2
        pdf_light = (1.0f / daxa_f32(light_count)) * (distance_squared / (area * cos_theta_light));
        
        // Check if the light is visible (shadow ray)
        RayDesc shadow_ray;
        shadow_ray.Origin = position + normal * 0.001f;
        shadow_ray.Direction = light_dir;
        shadow_ray.TMin = 0.001f;
        shadow_ray.TMax = distance - 0.002f;
        
        ShadowPayload shadow_payload = {true};
        TraceRay(
            tlas,
            RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER,
            ~0,
            1,
            0,
            0,
            shadow_ray,
            shadow_payload
        );
        
        if (shadow_payload.hit)
        {
            // Compute the BRDF
            daxa_f32vec3 brdf = albedo / PI; // Lambertian BRDF
            
            // Light emission
            daxa_f32vec3 emission = m.emission; // Assuming emission is stored in material
            
            // Compute the final radiance contribution
            daxa_f32vec3 contribution = emission * brdf * N_dot_L * cos_theta_light / pdf_light;
            
            // Accumulate radiance
            light_radiance += contribution;
        } 
        else
        {
            // Light is occluded
            printf("Shadow Hit - Light occluded. N_dot_L: %f, Cos_theta_light: %f, Distance_Squared: %f\n", N_dot_L, cos_theta_light, distance_squared);
        }
    }
    else
    {
        // Light or surface normal facing away
        pdf_light = 0.0f;
        // printf("Invalid Geometry: N_dot_L: %f, Cos_theta_light: %f, Distance_Squared: %f, light_normal: %f %f %f, light_dir: %f %f %f\n", N_dot_L, cos_theta_light, distance_squared, light_normal.x, light_normal.y, light_normal.z, light_dir.x, light_dir.y, light_dir.z);
    }
    
    return light_radiance;
}