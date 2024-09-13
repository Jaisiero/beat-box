#pragma once

#define DAXA_RAY_TRACING 1
#include <daxa/daxa.inl>
#include <math.hpp>


struct PushConstants
{
  daxa_u32vec2 size;
  daxa::RWTexture2DId<daxa_f32vec4> swapchain;
};


// RT STRUCTS
struct HitPayload
{
  daxa_f32vec3 hit_value;
};

struct ShadowRayPayload
{
  daxa_f32 shadow;
};

struct MyAttributes {
    daxa_f32vec3 normal;
    daxa_f32vec3 position;
};

[[vk::push_constant]] PushConstants p;