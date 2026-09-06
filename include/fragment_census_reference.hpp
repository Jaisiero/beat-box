#pragma once
#include "fragment_census.inl"
#include <map>
#include <vector>
#include <span>
#include <algorithm>

// Exact integer oracle and overflow fallback. GPU compact output is sorted by
// label on the host so tie-breaking remains independent of dispatch scheduling.
inline std::vector<FragmentComponent> fragment_census_reference(
    std::span<daxa_u32 const> labels, daxa_u32vec3 dims)
{
  std::map<daxa_u32, FragmentComponent> stats;
  for (daxa_u32 c = 0; c < labels.size(); ++c)
  {
    auto l = labels[c];
    if (l == 0xffffffffu) continue;
    daxa_u32 x = c % dims.x, y = (c / dims.x) % dims.y, z = c / (dims.x * dims.y);
    auto [it, inserted] = stats.try_emplace(l, FragmentComponent{l,0,0,0,0,x,y,z,x,y,z});
    auto &s = it->second;
    ++s.count; s.sum_x += x; s.sum_y += y; s.sum_z += z;
    s.lo_x = std::min(s.lo_x,x); s.hi_x = std::max(s.hi_x,x);
    s.lo_y = std::min(s.lo_y,y); s.hi_y = std::max(s.hi_y,y);
    s.lo_z = std::min(s.lo_z,z); s.hi_z = std::max(s.hi_z,z);
  }
  std::vector<FragmentComponent> out;
  for (auto const &[label,s] : stats) out.push_back(s);
  return out;
}

inline std::vector<FragmentComponent> decode_fragment_census(FragmentCensusOutput const &summary,
    std::span<daxa_u32 const> labels, daxa_u32vec3 dims)
{
  if (summary.count > BB_FRAGMENT_CENSUS_CAPACITY) return fragment_census_reference(labels, dims);
  std::vector<FragmentComponent> result(summary.components, summary.components + summary.count);
  std::sort(result.begin(), result.end(), [](auto const &a, auto const &b) { return a.label < b.label; });
  return result;
}
