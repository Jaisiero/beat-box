#include "fracture_setup.inl"
[[vk::push_constant]] FractureSetupPushConstants P;

// mt19937 matches the existing material seeding stream; fracture topology is
// procedural, with no precomputed break planes. Only a handful of draws are used.
struct SiteRandom
{
  uint state[624];
  uint index;
  [mutating] void init(uint seed)
  {
    state[0] = seed;
    for (uint i = 1u; i < 624u; ++i)
      state[i] = 1812433253u * (state[i-1u] ^ (state[i-1u] >> 30u)) + i;
    index = 624u;
  }
  [mutating] void twist()
  {
    for (uint i = 0u; i < 624u; ++i)
    {
      uint y = (state[i] & 0x80000000u) | (state[(i+1u)%624u] & 0x7fffffffu);
      state[i] = state[(i+397u)%624u] ^ (y >> 1u) ^ ((y & 1u) != 0u ? 0x9908b0dfu : 0u);
    }
    index = 0u;
  }
  [mutating] float sample()
  {
    if (index == 624u) twist();
    uint y = state[index++];
    y ^= y >> 11u; y ^= (y << 7u) & 0x9d2c5680u;
    y ^= (y << 15u) & 0xefc60000u; y ^= y >> 18u;
    precise float unit = min(float(y) * (1.0f / 4294967296.0f), asfloat(0x3f7fffffu));
    precise float value = unit * 2.0f - 1.0f;
    return value;
  }
};

[shader("compute")]
[numthreads(1,1,1)]
void entry_fracture_setup(uint3 tid : SV_DispatchThreadID)
{
  FractureParentContext* context = &((FractureParentContext*)P.contexts_addr)[P.body_id];
  context->site_count = 0u;
  RigidBody* bodies = (RigidBody*)P.bodies_addr;
  uint row = MAX_U32;
  for (uint i = 0u; i < P.body_count; ++i) if (bodies[i].id == P.body_id) { row = i; break; }
  if (row == MAX_U32 || bodies[row].shape_index == 0u) return;
  context->parent = bodies[row];
  context->shape = ((VoxelShape*)P.shapes_addr)[bodies[row].shape_index - 1u];
  RigidBody* body = &context->parent;
  VoxelShape* shape = &context->shape;
  FractureEvent event = ((FractureEventBuffer*)P.events_addr)->events[P.event_slot];
  float3 local = body->rotate_vector_inverse(event.position - body->position);
  context->center = (local - shape->grid_origin) / shape->voxel_size;
  float overkill = event.impulse / max(body->fracture_impulse, 1e-3f);
  float radius = clamp(1.0f + 0.6f*overkill, 1.6f, 2.6f);
  context->radius = clamp(radius*(1.4f + 0.5f*overkill), radius+1.0f, 6.0f);
  SiteRandom rng;
  rng.init(body->id * 2654435761u ^ asuint(event.impulse));
  if (body->fracture_material == 1u)
  {
    uint3 dims = shape->dims;
    uint grain = dims.x >= dims.y && dims.x >= dims.z ? 0u : (dims.y >= dims.z ? 1u : 2u);
    uint a = (grain+1u)%3u, b = (grain+2u)%3u;
    uint count = clamp(uint(floor(overkill*1.2f + 0.5f)), 2u, 6u);
    context->site_count = count;
    for (uint i = 0u; i < count; ++i)
    {
      precise float angle = 6.2831853f * float(i) / float(count);
      float3 site = context->center;
      site[a] += cos(angle)*context->radius*0.6f + rng.sample()*0.4f;
      site[b] += sin(angle)*context->radius*0.6f + rng.sample()*0.4f;
      site[grain] += rng.sample()*0.4f;
      context->sites[i] = float4(site, 0.0f);
    }
  }
  else
  {
    uint count = clamp(uint(floor(overkill*3.0f + 0.5f)), 3u, 12u);
    context->site_count = count;
    for (uint i = 0u; i < count; ++i)
    {
      float3 d;
      do { d = float3(rng.sample(),rng.sample(),rng.sample()); } while (dot(d,d) > 1.0f);
      context->sites[i] = float4(context->center + d*context->radius*0.8f,0.0f);
    }
  }
}
