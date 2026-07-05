#pragma once

#include "defines.hpp"
#include <map>
#include <string>
#include <algorithm>

BB_NAMESPACE_BEGIN

// A coalescing FREE-LIST of OFFSETS into a shared backing resource (a std::vector for the
// fracture memory pools, or a GPU buffer for the incremental-AS BLAS/prim regions). Unlike a
// per-size bucket stack (exact-size reuse only), this handles ARBITRARY sizes: alloc is
// first-fit with split (a large hole yields the exact front and keeps the remainder free),
// free merges adjacent ranges (coalescing) so scattered holes recombine, and the top shrinks
// when its tail is freed. Offsets, not pointers, so it survives a backing vector reallocating.
// free_ranges: offset -> size, kept sorted (std::map) and always fully coalesced (verify()
// enforces both). Units are caller-defined (u32 slots, bytes, Aabb count, ...); for aligned
// GPU allocations, pass alignment-multiple sizes and the offsets stay aligned.
struct FreeListPool
{
  daxa_u32 high_water = 0;                   // one-past the last used slot (== backing size)
  std::map<daxa_u32, daxa_u32> free_ranges;  // offset -> size, sorted + coalesced
  daxa_u32 live_bytes = 0;                    // sum of live allocation sizes (diagnostic)

  // returns an offset for `size` slots, or MAX_U32 if capacity is exhausted (no fitting hole
  // and no room to bump). ok reports success.
  daxa_u32 alloc(daxa_u32 size, daxa_u32 cap, bool &ok)
  {
    if (size == 0u) { ok = true; return high_water; } // degenerate: no slots needed
    for (auto it = free_ranges.begin(); it != free_ranges.end(); ++it) // first-fit
    {
      if (it->second >= size)
      {
        daxa_u32 const off = it->first;
        daxa_u32 const rem = it->second - size;
        free_ranges.erase(it);
        if (rem > 0u) { free_ranges[off + size] = rem; } // keep the remainder free
        live_bytes += size;
        ok = true;
        return off;
      }
    }
    if (high_water + size > cap) { ok = false; return 0xFFFFFFFFu; }
    daxa_u32 const off = high_water;
    high_water += size;
    live_bytes += size;
    ok = true;
    return off;
  }
  void free(daxa_u32 off, daxa_u32 size)
  {
    if (size == 0u) { return; }
    live_bytes -= size;
    daxa_u32 lo = off, hi = off + size;
    if (!free_ranges.empty()) // merge with the range immediately BEFORE (ends exactly at lo)
    {
      auto it = free_ranges.lower_bound(off);
      if (it != free_ranges.begin())
      {
        auto prev = std::prev(it);
        if (prev->first + prev->second == lo) { lo = prev->first; free_ranges.erase(prev); }
      }
    }
    { // merge with the range immediately AFTER (starts exactly at hi)
      auto it = free_ranges.find(hi);
      if (it != free_ranges.end()) { hi = it->first + it->second; free_ranges.erase(it); }
    }
    if (hi == high_water) { high_water = lo; } // freed the tail: shrink instead of holing
    else { free_ranges[lo] = hi - lo; }        // otherwise record the merged hole
  }
  bool can_alloc(daxa_u32 size, daxa_u32 cap) const
  {
    if (size == 0u) { return true; }
    for (auto const &r : free_ranges) { if (r.second >= size) { return true; } }
    return high_water + size <= cap;
  }
  void reset()
  {
    high_water = 0;
    live_bytes = 0;
    free_ranges.clear();
  }
  daxa_u32 largest_free_block() const
  {
    daxa_u32 m = 0u;
    for (auto const &r : free_ranges) { m = std::max(m, r.second); }
    return m;
  }
  // SELF-CHECK: free ranges sorted, in-bounds, FULLY COALESCED (no two touching), and the
  // accounting closes: Sigma(free) + live == high_water. Returns "" on success, else the
  // first broken invariant.
  std::string verify_free() const
  {
    daxa_u32 sum_free = 0u, prev_end = 0u;
    bool first = true;
    for (auto const &r : free_ranges)
    {
      if (r.second == 0u) { return "zero-size free range at " + std::to_string(r.first); }
      if (r.first + r.second > high_water) { return "free range past high_water at " + std::to_string(r.first); }
      if (!first && r.first <= prev_end)
      {
        return "unsorted or un-coalesced free ranges near " + std::to_string(r.first) +
               " (prev end " + std::to_string(prev_end) + ")";
      }
      first = false;
      prev_end = r.first + r.second;
      sum_free += r.second;
    }
    if (sum_free + live_bytes != high_water)
    {
      return "accounting mismatch: free " + std::to_string(sum_free) + " + live " +
             std::to_string(live_bytes) + " != high_water " + std::to_string(high_water);
    }
    return "";
  }
};

BB_NAMESPACE_END
