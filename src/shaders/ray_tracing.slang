#include "shared.inl"


[shader("raygeneration")]
void ray_generation()
{
  const daxa_u32vec2 threadIdx = DispatchRaysIndex().xy;
  const daxa_u32vec2 rt_size = DispatchRaysDimensions().xy;

  HitPayload payload = {daxa_f32vec3(0.0)};

  payload.hit_value = daxa_f32vec3(1.0, 0.0, 0.0);

  // TODO: trace rays here
  
  p.swapchain.get()[threadIdx.xy] = daxa_f32vec4(payload.hit_value, 1.0);
}

[shader("miss")]
void miss(inout HitPayload payload)
{
    payload.hit_value = daxa_f32vec3(0.0, 0.0, 0.05); // Background color
}
[shader("miss")]
void miss_shadows(inout ShadowRayPayload payload)
{
    payload.shadow = 1.0f;
}

[shader("closesthit")]
void closest_hit(inout HitPayload payload, 
in MyAttributes attr)
{
    payload.hit_value = daxa_f32vec3(0.0, 1.0, 0.0);
}

[shader("intersection")]
void intersection()
{
  
}