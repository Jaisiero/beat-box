#include "fracture_setup.inl"

// ----- FRACTURE kernels (carve + connected components), VoxelFracturePushConstants -----
// One-off dispatches from the host orchestrator at fracture events. Labels live in the
// EDT scratch buffer (voxel_sdf_scratch[0] reinterpreted as u32 - both users are
// fracture-rate one-offs, never concurrent).

[[vk::push_constant]] VoxelFracturePushConstants VFR;

uint3 vfr_dims() { return ((FractureParentContext*)VFR.context_addr)->shape.dims; }
uint vfr_occ_offset() { return ((FractureParentContext*)VFR.context_addr)->shape.occ_offset; }

daxa_b32 vfr_cell_solid(daxa_u32* occ, daxa_i32 x, daxa_i32 y, daxa_i32 z, uint3 dims, uint occ_offset)
{
  if (x < 0 || y < 0 || z < 0 ||
      x >= daxa_i32(dims.x) || y >= daxa_i32(dims.y) || z >= daxa_i32(dims.z))
  {
    return false;
  }
  daxa_u32 bit = daxa_u32(x) + daxa_u32(y) * dims.x + daxa_u32(z) * dims.x * dims.y;
  return ((occ[occ_offset + bit / 32u] >> (bit % 32u)) & 1u) != 0u;
}

// CARVE: clear every solid cell whose center lies inside the impact sphere (grid units).
// Atomic AND per bit - parallel-safe; each thread owns its own bit, so deterministic.
[shader("compute")]
[numthreads(64, 1, 1)] void entry_voxel_carve(daxa_u32vec3 tid : SV_DispatchThreadID)
{
  uint3 dims=vfr_dims();
  uint occ_offset=vfr_occ_offset();
  FractureParentContext* context = (FractureParentContext*)VFR.context_addr;
  daxa_u32 bit_count = dims.x * dims.y * dims.z;
  daxa_u32 bit = tid.x;
  if (bit >= bit_count) { return; }
  daxa_u32 x = bit % dims.x;
  daxa_u32 y = (bit / dims.x) % dims.y;
  daxa_u32 z = bit / (dims.x * dims.y);
  daxa_f32vec3 c = daxa_f32vec3(daxa_f32(x) + 0.5f, daxa_f32(y) + 0.5f, daxa_f32(z) + 0.5f) - context->center;
  if (dot(c, c) > VFR.carve_radius * VFR.carve_radius) { return; }
  daxa_u32* occ = (daxa_u32*)(VFR.occupancy_addr);
  daxa_u32 _old;
  InterlockedAnd(occ[occ_offset + bit / 32u], ~(1u << (bit % 32u)), _old);
}

// VORONOI ASSIGN: per solid cell, the index of the nearest fracture site (grid-space
// Euclidean). Cells beyond voronoi_radius of the impact share a single "remainder" site
// (index = site_count) so only the impact zone shatters; the rest stays one piece. The
// constrained flood below then splits the impact zone along Voronoi boundaries. Empty
// cells get MAX_U32. Sites are GPU-seeded per material (stone = isotropic cloud, wood =
// strung along the grain) - see the orchestrator.
[shader("compute")]
[numthreads(64, 1, 1)] void entry_voxel_voronoi_assign(daxa_u32vec3 tid : SV_DispatchThreadID)
{
  uint3 dims=vfr_dims();
  uint occ_offset=vfr_occ_offset();
  FractureParentContext* context = (FractureParentContext*)VFR.context_addr;
  daxa_u32 bit_count = dims.x * dims.y * dims.z;
  daxa_u32 bit = tid.x;
  if (bit >= bit_count) { return; }
  daxa_u32* occ = (daxa_u32*)(VFR.occupancy_addr);
  daxa_u32* site_labels = (daxa_u32*)(VFR.site_labels_addr);
  bool solid = ((occ[occ_offset + bit / 32u] >> (bit % 32u)) & 1u) != 0u;
  if (!solid) { site_labels[bit] = 0xFFFFFFFFu; return; }
  daxa_u32 x = bit % dims.x;
  daxa_u32 y = (bit / dims.x) % dims.y;
  daxa_u32 z = bit / (dims.x * dims.y);
  daxa_f32vec3 p = daxa_f32vec3(daxa_f32(x) + 0.5f, daxa_f32(y) + 0.5f, daxa_f32(z) + 0.5f);
  daxa_f32vec3 d0 = p - context->center;
  if (dot(d0, d0) > context->radius * context->radius)
  {
    site_labels[bit] = context->site_count; // the shared remainder (intact chunk beyond the blast)
    return;
  }
  daxa_f32vec4* sites = &context->sites[0];
  daxa_u32 best = 0u;
  daxa_f32 bestd = 1.0e30f;
  for (daxa_u32 i = 0u; i < context->site_count; ++i)
  {
    daxa_f32vec3 s = sites[i].xyz;
    daxa_f32vec3 dd = p - s;
    daxa_f32 l2 = dot(dd, dd);
    if (l2 < bestd) { bestd = l2; best = i; }
  }
  site_labels[bit] = best;
}

// CONNECTED COMPONENTS, phase 1: label init (own cell index for solid, MAX_U32 empty)
[shader("compute")]
[numthreads(64, 1, 1)] void entry_voxel_flood_init(daxa_u32vec3 tid : SV_DispatchThreadID)
{
  uint3 dims=vfr_dims();
  uint occ_offset=vfr_occ_offset();
  FractureParentContext* context = (FractureParentContext*)VFR.context_addr;
  daxa_u32 bit_count = dims.x * dims.y * dims.z;
  daxa_u32 bit = tid.x;
  if (bit >= bit_count) { return; }
  daxa_u32* occ = (daxa_u32*)(VFR.occupancy_addr);
  daxa_u32* labels = (daxa_u32*)(VFR.labels_addr);
  bool solid = ((occ[occ_offset + bit / 32u] >> (bit % 32u)) & 1u) != 0u;
  labels[bit] = solid ? bit : 0xFFFFFFFFu;
}

// Small cropped fragments fit in a 64-cell forest. One invocation owns all
// shared entries: no atomics or cross-invocation synchronization is needed.
// Always link the larger root to the smaller root, matching flood labels.
groupshared uint small_roots[64];
uint small_root(uint cell)
{
  while (small_roots[cell]!=cell) cell=small_roots[cell];
  return cell;
}
void label_small_fragment(uint cells)
{
  uint3 dims=vfr_dims();
  uint* labels=(uint*)VFR.labels_addr;
  uint* sites=(uint*)VFR.site_labels_addr;
  for (uint i=0u;i<cells;++i) small_roots[i]=labels[i];
  for (uint i=0u;i<cells;++i)
  {
    if (small_roots[i]==MAX_U32) continue;
    uint x=i%dims.x, y=(i/dims.x)%dims.y, z=i/(dims.x*dims.y);
    uint neighbors[3]={x>0u ? i-1u : MAX_U32,
                       y>0u ? i-dims.x : MAX_U32,
                       z>0u ? i-dims.x*dims.y : MAX_U32};
    for (uint d=0u;d<3u;++d)
    {
      uint n=neighbors[d];
      if (n==MAX_U32 || small_roots[n]==MAX_U32) continue;
      if (VFR.use_voronoi!=0u && sites[n]!=sites[i]) continue;
      uint a=small_root(i), b=small_root(n);
      small_roots[max(a,b)]=min(a,b);
    }
  }
  for (uint i=0u;i<cells;++i)
    labels[i]=small_roots[i]==MAX_U32 ? MAX_U32 : small_root(i);
}

// CONNECTED COMPONENTS, phase 2 (host-iterated): min-label propagation over solid
// 6-neighbors PLUS pointer jumping (label = labels[label]) so long serpentine paths
// collapse logarithmically within the host's fixed iteration budget. min() over a fixed
// stencil converges monotonically; atomic accesses make the in-place relaxation
// race-free, but intermediate iterations can still depend on invocation order.
[shader("compute")]
[numthreads(64, 1, 1)] void entry_voxel_flood_step(daxa_u32vec3 tid : SV_DispatchThreadID)
{
  uint3 dims=vfr_dims();
  uint occ_offset=vfr_occ_offset();
  FractureParentContext* context = (FractureParentContext*)VFR.context_addr;
  daxa_u32 bit_count = dims.x * dims.y * dims.z;
  if (bit_count<=64u) {
    if (tid.x==0u) label_small_fragment(bit_count);
    return;
  }
  daxa_u32 bit = tid.x;
  if (bit >= bit_count) { return; }
  daxa_u32* labels = (daxa_u32*)(VFR.labels_addr);
  daxa_u32 lbl;
  InterlockedAdd(labels[bit], 0u, lbl);
  if (lbl == 0xFFFFFFFFu) { return; }
  daxa_u32* occ = (daxa_u32*)(VFR.occupancy_addr);
  daxa_i32 x = daxa_i32(bit % dims.x);
  daxa_i32 y = daxa_i32((bit / dims.x) % dims.y);
  daxa_i32 z = daxa_i32(bit / (dims.x * dims.y));
  // VORONOI CONSTRAINT: when enabled, only merge across a neighbor that shares my Voronoi
  // site, so a fragment = one connected region of one Voronoi cell. use_voronoi==0 is the
  // byte-identical legacy path (a hit severs into natural connected components).
  daxa_u32* site_labels = VFR.use_voronoi != 0u ? (daxa_u32*)(VFR.site_labels_addr) : (daxa_u32*)0;
  daxa_u32 my_site = VFR.use_voronoi != 0u ? site_labels[bit] : 0u;
  daxa_i32 nb[6][3] = {{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
  daxa_u32 best = lbl;
  [unroll]
  for (daxa_u32 d = 0u; d < 6u; ++d)
  {
    daxa_i32 nx = x + nb[d][0], ny = y + nb[d][1], nz = z + nb[d][2];
    if (!vfr_cell_solid(occ, nx, ny, nz, dims, occ_offset)) { continue; }
    daxa_u32 nbit = daxa_u32(nx) + daxa_u32(ny) * dims.x + daxa_u32(nz) * dims.x * dims.y;
    if (VFR.use_voronoi != 0u && site_labels[nbit] != my_site) { continue; } // different Voronoi cell: no merge
    daxa_u32 neighbor_label;
    InterlockedAdd(labels[nbit], 0u, neighbor_label);
    best = min(best, neighbor_label);
  }
  // pointer jump: adopt the (lower) label of the cell my label points at. Safe under the
  // Voronoi constraint: `best` only ever came from same-site cells, so labels[best] is in
  // the same site+component.
  daxa_u32 parent_label;
  InterlockedAdd(labels[best], 0u, parent_label);
  best = min(best, parent_label);
  daxa_u32 previous_label;
  InterlockedMin(labels[bit], best, previous_label);
}
