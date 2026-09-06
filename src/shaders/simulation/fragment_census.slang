#include "fragment_census.inl"
[[vk::push_constant]] FragmentCensusPushConstants C;

[shader("compute")]
[numthreads(64,1,1)]
void entry_fragment_census_init(uint3 tid : SV_DispatchThreadID)
{
  uint c = tid.x;
  if (c >= C.dims.x*C.dims.y*C.dims.z) return;
  FragmentComponent* s = &((FragmentComponent*)C.scratch_addr)[c];
  s->label = c;
  s->count = 0;
  s->sum_x = s->sum_y = s->sum_z = 0;
  s->lo_x = s->lo_y = s->lo_z = 0xffffffffu;
  s->hi_x = s->hi_y = s->hi_z = 0;
  if (c == 0) ((FragmentCensusOutput*)C.output_addr)->count = 0;
}

[shader("compute")]
[numthreads(64,1,1)]
void entry_fragment_census_accumulate(uint3 tid : SV_DispatchThreadID)
{
  uint c = tid.x;
  uint cells = C.dims.x*C.dims.y*C.dims.z;
  uint pending = c < cells ? ((uint*)C.labels_addr)[c] : 0xffffffffu;
  uint x = c%C.dims.x, y = (c/C.dims.x)%C.dims.y, z = c/(C.dims.x*C.dims.y);
  // Aggregate equal labels within the subgroup before updating shared component
  // records. Every lane participates, including the tail of a partial dispatch.
  while (WaveActiveAnyTrue(pending != 0xffffffffu))
  {
    uint label = WaveActiveMin(pending);
    bool match = pending == label;
    uint count = WaveActiveSum(match ? 1u : 0u);
    uint sx = WaveActiveSum(match ? x : 0u);
    uint sy = WaveActiveSum(match ? y : 0u);
    uint sz = WaveActiveSum(match ? z : 0u);
    uint lx = WaveActiveMin(match ? x : 0xffffffffu), hx = WaveActiveMax(match ? x : 0u);
    uint ly = WaveActiveMin(match ? y : 0xffffffffu), hy = WaveActiveMax(match ? y : 0u);
    uint lz = WaveActiveMin(match ? z : 0xffffffffu), hz = WaveActiveMax(match ? z : 0u);
    if (WaveIsFirstLane())
    {
      FragmentComponent* s = &((FragmentComponent*)C.scratch_addr)[label];
      uint old;
      InterlockedAdd(s->count, count, old);
      InterlockedAdd(s->sum_x, sx, old); InterlockedAdd(s->sum_y, sy, old); InterlockedAdd(s->sum_z, sz, old);
      InterlockedMin(s->lo_x, lx, old); InterlockedMax(s->hi_x, hx, old);
      InterlockedMin(s->lo_y, ly, old); InterlockedMax(s->hi_y, hy, old);
      InterlockedMin(s->lo_z, lz, old); InterlockedMax(s->hi_z, hz, old);
    }
    if (match) pending = 0xffffffffu;
  }
}

[shader("compute")]
[numthreads(64,1,1)]
void entry_fragment_census_compact(uint3 tid : SV_DispatchThreadID)
{
  uint c = tid.x;
  if (c >= C.dims.x*C.dims.y*C.dims.z) return;
  FragmentComponent s = ((FragmentComponent*)C.scratch_addr)[c];
  if (s.count == 0) return;
  FragmentCensusOutput* output = (FragmentCensusOutput*)C.output_addr;
  uint slot;
  InterlockedAdd(output->count, 1u, slot);
  // Overflow never truncates geometry: the host falls back to its label census.
  if (slot < BB_FRAGMENT_CENSUS_CAPACITY) output->components[slot] = s;
}
