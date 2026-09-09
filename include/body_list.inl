#pragma once
#include "shared.inl"
struct BodyListPushConstants
{
  daxa_u64 bodies_addr, active_addr, entries_addr;
  daxa_u32 count;
};
