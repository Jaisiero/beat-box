#pragma once
#include "shared.inl"

static const daxa_u32 GPU_POOL_RANGE_CAPACITY = 1025u;
static const daxa_u32 GPU_POOL_COUNT = 6u; // occupancy, SDF, surface, primitives, shapes, bodies
static const daxa_u32 GPU_POOL_INVALID = 0xffffffffu;
struct GpuPoolValidationPushConstants { daxa_u64 transaction_addr, failures_addr; };
struct GpuPoolRange { daxa_u32 offset, size; };

// One control invocation owns a transaction (host admission or GPU controller).
// Parallel geometry kernels only consume accepted reservations; they never
// modify these free lists. Range capacity exceeds the maximum live body count.
struct GpuFreeList
{
  daxa_u32 capacity, high_water, live_units, range_count;
  GpuPoolRange ranges[GPU_POOL_RANGE_CAPACITY];

  BB_REF daxa_u32 allocate(daxa_u32 size)
  {
    if (size == 0u) return high_water;
    for (daxa_u32 i = 0u; i < range_count; ++i)
    {
      if (ranges[i].size < size) continue;
      daxa_u32 offset = ranges[i].offset;
      ranges[i].offset += size;
      ranges[i].size -= size;
      if (ranges[i].size == 0u)
      {
        for (daxa_u32 j = i + 1u; j < range_count; ++j) ranges[j-1u] = ranges[j];
        --range_count;
      }
      live_units += size;
      return offset;
    }
    if (high_water > capacity || size > capacity - high_water) return GPU_POOL_INVALID;
    daxa_u32 offset = high_water;
    high_water += size;
    live_units += size;
    return offset;
  }

  BB_REF bool release(daxa_u32 offset, daxa_u32 size)
  {
    if (size == 0u) return true;
    if (offset > high_water || size > high_water - offset || size > live_units) return false;
    daxa_u32 end = offset + size;
    daxa_u32 i = 0u;
    while (i < range_count && ranges[i].offset < offset) ++i;
    if (i > 0u && ranges[i-1u].offset + ranges[i-1u].size > offset) return false;
    if (i < range_count && ranges[i].offset < end) return false;
    bool left = i > 0u && ranges[i-1u].offset + ranges[i-1u].size == offset;
    bool right = i < range_count && ranges[i].offset == end;
    if (!left && !right && end != high_water && range_count == GPU_POOL_RANGE_CAPACITY) return false;
    daxa_u32 lo = left ? ranges[i-1u].offset : offset;
    daxa_u32 hi = right ? ranges[i].offset + ranges[i].size : end;
    daxa_u32 first = left ? i-1u : i;
    daxa_u32 erased = (left ? 1u : 0u) + (right ? 1u : 0u);
    for (daxa_u32 j = first + erased; j < range_count; ++j) ranges[j-erased] = ranges[j];
    range_count -= erased;
    live_units -= size;
    if (hi == high_water) high_water = lo;
    else
    {
      for (daxa_u32 j = range_count; j > first; --j) ranges[j] = ranges[j-1u];
      ranges[first].offset = lo;
      ranges[first].size = hi-lo;
      ++range_count;
    }
    return true;
  }

  BB_REF bool valid()
  {
    if (range_count > GPU_POOL_RANGE_CAPACITY || high_water > capacity || live_units > high_water) return false;
    daxa_u32 free_units = 0u, previous_end = 0u;
    for (daxa_u32 i = 0u; i < range_count; ++i)
    {
      if (ranges[i].size == 0u || ranges[i].offset >= high_water ||
          ranges[i].size >= high_water - ranges[i].offset ||
          (i != 0u && ranges[i].offset <= previous_end)) return false;
      previous_end = ranges[i].offset + ranges[i].size;
      free_units += ranges[i].size;
    }
    return free_units == high_water - live_units;
  }
};

struct GpuPoolReservation { daxa_u32 offsets[GPU_POOL_COUNT]; };
struct GpuPoolRequest { daxa_u32 units[GPU_POOL_COUNT]; };
struct GpuPoolTransaction
{
  // 0 = idle, 1 = reserved, 2 = geometry ready, 3 = aborted.
  daxa_u32 phase, epoch, failed_pool, reservation_count;
  GpuFreeList committed[GPU_POOL_COUNT];
  GpuFreeList proposed[GPU_POOL_COUNT];

  BB_REF bool begin()
  {
    if (phase == 1u || phase == 2u) return false;
    for (daxa_u32 i = 0u; i < GPU_POOL_COUNT; ++i)
    {
      proposed[i].capacity = committed[i].capacity;
      proposed[i].high_water = committed[i].high_water;
      proposed[i].live_units = committed[i].live_units;
      proposed[i].range_count = committed[i].range_count;
      for (daxa_u32 j = 0u; j < committed[i].range_count; ++j)
        proposed[i].ranges[j] = committed[i].ranges[j];
    }
    phase = 1u; failed_pool = GPU_POOL_INVALID; reservation_count = 0u;
    return true;
  }

  BB_REF GpuPoolReservation reserve(GpuPoolRequest request)
  {
    GpuPoolReservation result;
    for (daxa_u32 i = 0u; i < GPU_POOL_COUNT; ++i) result.offsets[i] = GPU_POOL_INVALID;
    if (phase != 1u) return result;
    for (daxa_u32 i = 0u; i < GPU_POOL_COUNT; ++i)
    {
      result.offsets[i] = proposed[i].allocate(request.units[i]);
      if (result.offsets[i] == GPU_POOL_INVALID)
      {
        failed_pool = i; phase = 3u;
        for (daxa_u32 j = 0u; j < GPU_POOL_COUNT; ++j) result.offsets[j] = GPU_POOL_INVALID;
        return result;
      }
    }
    ++reservation_count;
    return result;
  }

  BB_REF bool retire(daxa_u32 pool, daxa_u32 offset, daxa_u32 size)
  {
    // Retire only after construction: reservations must never reuse parent
    // storage while geometry kernels may still be reading it.
    if (phase != 2u || pool >= GPU_POOL_COUNT) return false;
    if (!proposed[pool].release(offset, size)) { phase = 3u; failed_pool = pool; return false; }
    return true;
  }

  BB_REF bool mark_ready()
  {
    if (phase != 1u) return false;
    for (daxa_u32 i = 0u; i < GPU_POOL_COUNT; ++i)
      if (!proposed[i].valid()) { phase = 3u; failed_pool = i; return false; }
    phase = 2u;
    return true;
  }

  // Caller must order this AFTER geometry/AS preparation succeeds, and AFTER
  // all readers of retired parent allocations complete. This is not a barrier.
  BB_REF bool publish()
  {
    if (phase != 2u) return false;
    for (daxa_u32 i = 0u; i < GPU_POOL_COUNT; ++i)
    {
      committed[i].capacity = proposed[i].capacity;
      committed[i].high_water = proposed[i].high_water;
      committed[i].live_units = proposed[i].live_units;
      committed[i].range_count = proposed[i].range_count;
      for (daxa_u32 j = 0u; j < proposed[i].range_count; ++j)
        committed[i].ranges[j] = proposed[i].ranges[j];
    }
    ++epoch; phase = 0u;
    return true;
  }

  BB_REF void abort() { phase = 3u; }
};

#if defined(__cplusplus)
static_assert(sizeof(GpuFreeList) == 16u + 8u*GPU_POOL_RANGE_CAPACITY);
static_assert(sizeof(GpuPoolTransaction) == 16u + 2u*GPU_POOL_COUNT*sizeof(GpuFreeList));
#endif
