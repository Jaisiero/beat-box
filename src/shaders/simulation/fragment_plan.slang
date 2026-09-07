#include "fragment_plan.inl"
#include "fracture_setup.inl"
[[vk::push_constant]] FragmentPlanPushConstants P;

// Correctly rounded positive integer ratio, without requiring shaderFloat64.
// Equivalent to rounding the old double centroid to float. Integer division
// resolves ties explicitly; uint64 intermediates cannot exceed 2^54 here.
float centroid_coordinate(uint sum, uint count)
{
  if (sum == 0u) return 0.0f;
  uint64_t numerator=sum, denominator=count;
  int exponent=0;
  while (numerator < denominator) { numerator <<= 1; --exponent; }
  while (numerator >= (denominator << 1)) { denominator <<= 1; ++exponent; }
  uint64_t scaled=numerator << 23;
  uint mantissa=uint(scaled/denominator);
  uint64_t remainder=scaled%denominator;
  if (remainder*2 > denominator || (remainder*2 == denominator && (mantissa&1u)!=0u)) ++mantissa;
  if (mantissa == (1u<<24)) { mantissa >>= 1; ++exponent; }
  return asfloat((uint(exponent+127)<<23) | (mantissa&0x7fffffu));
}
float3 centroid(FragmentComponent s)
{
  return float3(centroid_coordinate(s.sum_x,s.count),
                centroid_coordinate(s.sum_y,s.count),
                centroid_coordinate(s.sum_z,s.count));
}

uint plan_cells() {
  if (P.context_addr==0u) return P.cells;
  uint3 dims=((FractureParentContext*)P.context_addr)->shape.dims;
  return dims.x*dims.y*dims.z;
}

// Cooperatively fetch census tiles; lane zero retains the original label order,
// tie-breaking, centroid calculations, merges and final sort.
groupshared FragmentComponent plan_tile[64];
groupshared uint plan_failed;
[shader("compute")]
[numthreads(64,1,1)]
void entry_fragment_plan(uint3 tid : SV_DispatchThreadID)
{
  FragmentComponent* census=(FragmentComponent*)P.census_addr;
  uint* remap=(uint*)P.remap_addr;
  FragmentPlanManifest* out=(FragmentPlanManifest*)P.manifest_addr;
  uint lane=tid.x;
  if (lane==0u) {
    out->status=0u;out->count=0u;out->source_count=0u;out->solid_count=0u;
    plan_failed=0u;
  }
  uint largest=0xffffffffu, largest_count=0u;
  uint cells=plan_cells();
  // Dense roots are visited in label order, preserving all tie-breaking.
  for (uint tile=0u;tile<cells;tile+=64u)
  {
    if (tile+lane<cells) plan_tile[lane]=census[tile+lane];
    GroupMemoryBarrierWithGroupSync();
    if (lane==0u)
    for (uint j=0u;j<min(64u,cells-tile);++j)
    {
      uint i=tile+j;
      uint n=plan_tile[j].count;
      if (n==0u) continue;
      ++out->source_count;out->solid_count+=n;
      if (n>largest_count) { largest=i;largest_count=n; }
      if (n>=3u)
      {
        if (out->count==FRAGMENT_PLAN_CAPACITY) { out->status=1u;plan_failed=1u;break; }
        out->components[out->count++]=plan_tile[j];
      }
    }
    // Prevent the next tile from overwriting values still used by lane zero.
    GroupMemoryBarrierWithGroupSync();
    if (plan_failed!=0u) return;
  }
  if (lane==0u && out->count==0u && largest!=0xffffffffu) out->components[out->count++]=census[largest];
  for (uint tile=0u;tile<cells;tile+=64u)
  {
    if (tile+lane<cells) plan_tile[lane]=census[tile+lane];
    GroupMemoryBarrierWithGroupSync();
    if (lane==0u)
    for (uint j=0u;j<min(64u,cells-tile);++j)
    {
      uint i=tile+j;
      FragmentComponent source=plan_tile[j];
      if (source.count==0u) continue;
      if (source.count>=3u) { remap[i]=i;continue; }
      float3 cc=centroid(source);
      uint best=0u;
      float best_distance=3.402823466e38f;
      for (uint t=0u;t<out->count;++t)
      {
        // Targets use their ORIGINAL centroids, never the progressively merged one.
        uint label=out->components[t].label;
        precise float3 d=cc-centroid(census[label]);
        precise float distance=(d.x*d.x+d.y*d.y)+d.z*d.z;
        if (distance<best_distance) { best_distance=distance;best=t; }
      }
      uint target=out->components[best].label;
      remap[i]=target;
      if (i==target) continue; // largest fallback is already present
      FragmentComponent* dest=&out->components[best];
      dest->count+=source.count;
      dest->sum_x+=source.sum_x;dest->sum_y+=source.sum_y;dest->sum_z+=source.sum_z;
      dest->lo_x=min(dest->lo_x,source.lo_x);dest->lo_y=min(dest->lo_y,source.lo_y);dest->lo_z=min(dest->lo_z,source.lo_z);
      dest->hi_x=max(dest->hi_x,source.hi_x);dest->hi_y=max(dest->hi_y,source.hi_y);dest->hi_z=max(dest->hi_z,source.hi_z);
    }
    GroupMemoryBarrierWithGroupSync();
  }
  if (lane!=0u) return;
  // Fragment publication retains the largest component in the parent's body slot.
  for (uint i=1u;i<out->count;++i)
  {
    FragmentComponent value=out->components[i];uint j=i;
    while (j>0u)
    {
      FragmentComponent prev=out->components[j-1u];
      if (prev.count>value.count || (prev.count==value.count && prev.label<value.label)) break;
      out->components[j]=prev;--j;
    }
    out->components[j]=value;
  }
}
