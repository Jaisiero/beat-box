#include "fragment_packing.inl"
[[vk::push_constant]] FragmentPackingPushConstants P;

[shader("compute")]
[numthreads(64,1,1)]
void entry_fragment_pack(uint3 tid : SV_DispatchThreadID)
{
  uint cells = P.crop_dims.x * P.crop_dims.y * P.crop_dims.z;
  uint word = tid.x;
  if (word >= (cells + 31u)/32u) return;
  uint bits = 0u;
  FragmentLabelRemap* remap = (FragmentLabelRemap*)P.remap_addr;
  // One invocation owns a whole output word, including zero padding. No atomic
  // updates or separate clear dispatch; reused pool slices cannot retain bits.
  for (uint bit = 0u; bit < 32u && word*32u+bit < cells; ++bit)
  {
    uint c = word*32u+bit;
    uint3 p = uint3(c%P.crop_dims.x, (c/P.crop_dims.x)%P.crop_dims.y,
                    c/(P.crop_dims.x*P.crop_dims.y)) + P.crop_min;
    uint source = ((uint*)P.labels_addr)[p.x+p.y*P.parent_dims.x+p.z*P.parent_dims.x*P.parent_dims.y];
    uint lo=0u, hi=P.remap_count;
    while (lo < hi)
    {
      uint mid=lo+(hi-lo)/2u;
      if (remap[mid].source < source) lo=mid+1u; else hi=mid;
    }
    if (lo < P.remap_count && remap[lo].source == source && remap[lo].target == P.target_label)
      bits |= 1u << bit;
  }
  ((uint*)P.occupancy_addr)[P.occ_offset+word] = bits;
}
