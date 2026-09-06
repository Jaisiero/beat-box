#include "fragment_census_reference.hpp"
#include <iostream>

int main()
{
  int failures = 0;
  auto check = [&](bool ok) { if (!ok) ++failures; };
  std::vector<daxa_u32> labels = {0,0,0xffffffffu,3,3,3};
  auto r = fragment_census_reference(labels, {3,2,1});
  check(r.size() == 2);
  check(r[0].label == 0 && r[0].count == 2 && r[0].sum_x == 1 && r[0].sum_y == 0);
  check(r[0].lo_x == 0 && r[0].hi_x == 1 && r[0].hi_y == 0);
  check(r[1].label == 3 && r[1].count == 3 && r[1].sum_x == 3 && r[1].sum_y == 3);
  check(r[1].lo_y == 1 && r[1].hi_x == 2);
  FragmentCensusOutput out{};
  out.count = 2; out.components[0] = r[1]; out.components[1] = r[0];
  auto sorted = decode_fragment_census(out, labels, {3,2,1});
  check(sorted[0].label == 0 && sorted[1].label == 3);
  // More components than the compact GPU capacity must be recovered in full.
  labels.resize(257);
  for (daxa_u32 i = 0; i < labels.size(); ++i) labels[i] = i;
  out.count = 257;
  auto fallback = decode_fragment_census(out, labels, {257,1,1});
  check(fallback.size() == 257 && fallback.back().count == 1 && fallback.back().sum_x == 256);
  // Maximum supported one-axis grid: sums must not overflow uint32.
  labels.assign(65536, 0);
  auto large = fragment_census_reference(labels, {65536,1,1});
  check(large.size() == 1 && large[0].sum_x == 2147450880u && large[0].hi_x == 65535);
  labels.assign(1, 0xffffffffu);
  check(fragment_census_reference(labels, {1,1,1}).empty());
  std::cout << failures << " fragment census failures\n";
  return failures ? 1 : 0;
}
