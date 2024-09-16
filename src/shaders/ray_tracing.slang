#include "shared.inl"

[shader("raygeneration")] void ray_generation()
{
  const daxa_u32vec2 thread_idx = DispatchRaysIndex().xy;
  const daxa_u32vec2 rt_size = DispatchRaysDimensions().xy;

  HitPayload payload = {daxa_f32vec3(0.0)};

  payload.hit_value = daxa_f32vec3(1.0, 0.0, 0.0);

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

[shader("closesthit")] void closest_hit(inout HitPayload payload,
                                            in MyAttributes attr)
{
  payload.hit_value = daxa_f32vec3(0.0, 1.0, 0.0);
}

[shader("intersection")] void intersection() {
  daxa_f32 THit = RayTCurrent();
  MyAttributes params = {};

  daxa_u32 prim_index = PrimitiveIndex();
  // Get Aabb
  Aabb aabb = p.task_head.aabbs[prim_index];

  daxa_f32mat4x4 obj_to_world = transpose(Convert3x4To4x4(ObjectToWorld4x3()));

  aabb.min = mul(obj_to_world, daxa_f32vec4(aabb.min, 1.0)).xyz;
  aabb.max = mul(obj_to_world, daxa_f32vec4(aabb.max, 1.0)).xyz;

  THit = hitAabb(aabb, {WorldRayOrigin(), WorldRayDirection()});

  if(THit > 0.0)
  {
      ReportHit(THit, /*hitKind*/ 0, params);
  }
}