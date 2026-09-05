#include "shared.inl"

// GPU node-SDF build (GPU-first directive + destructibility prerequisite): computes each
// voxel shape's node signed-distance field FROM the occupancy bitmask, on the GPU, exactly.
//
// EXACTNESS ARGUMENT: the solid is a union of lattice-aligned cell boxes and the query
// points are lattice NODES, so the closest point of the solid (or empty) region to any
// node is itself a lattice node (per-axis clamp of integer coords onto an integer
// interval). Distance-to-region therefore equals distance-to-nearest-ZERO-NODE, and a
// separable per-axis Euclidean distance transform over the node grid reproduces the CPU
// brute force bit-for-bit (up to sqrt rounding).
//
// Passes (one dispatch chain per shape, load-time / on shape edit only):
//   init      : Z_solid / Z_empty membership from the bitmask (adjacent-8 cells; out of
//               range counts as empty = the outside region) -> squared distances 0 / INF
//   axis x3   : per-column combine  out(i) = min_j in(j) + (i-j)^2   (grid units^2)
//   finalize  : sdf = (ds > 0 ? +sqrt(ds) : -sqrt(de)) * voxel_size  -> the shared pool
//
// No task graph: raw device addresses via push constants + explicit barriers, the same
// bypass convention the voxel pools already use (written before the sim ever reads them).

[[vk::push_constant]] VoxelSdfBuildPushConstants VSB;

daxa_b32 vsb_cell_solid(daxa_u32* occ, daxa_i32 x, daxa_i32 y, daxa_i32 z)
{
  if (x < 0 || y < 0 || z < 0 ||
      x >= daxa_i32(VSB.cell_dims.x) || y >= daxa_i32(VSB.cell_dims.y) || z >= daxa_i32(VSB.cell_dims.z))
  {
    return false; // outside the grid = the (empty) outside region
  }
  daxa_u32 bit = daxa_u32(x) + daxa_u32(y) * VSB.cell_dims.x + daxa_u32(z) * VSB.cell_dims.x * VSB.cell_dims.y;
  return ((occ[VSB.occ_offset + bit / 32u] >> (bit % 32u)) & 1u) != 0u;
}

daxa_u32 vsb_node_index(daxa_u32 x, daxa_u32 y, daxa_u32 z)
{
  daxa_u32 nx = VSB.cell_dims.x + 1u;
  daxa_u32 ny = VSB.cell_dims.y + 1u;
  return x + y * nx + z * nx * ny;
}

static const daxa_f32 VSB_INF = 1.0e30f;

[shader("compute")]
[numthreads(64, 1, 1)] void entry_voxel_sdf_init(daxa_u32vec3 tid : SV_DispatchThreadID)
{
  daxa_u32 nx = VSB.cell_dims.x + 1u;
  daxa_u32 ny = VSB.cell_dims.y + 1u;
  daxa_u32 nz = VSB.cell_dims.z + 1u;
  daxa_u32 total = nx * ny * nz;
  daxa_u32 i = tid.x;
  if (i >= total) { return; }
  daxa_u32 x = i % nx;
  daxa_u32 y = (i / nx) % ny;
  daxa_u32 z = i / (nx * ny);

  daxa_u32* occ = (daxa_u32*)(VSB.occupancy_addr);
  // the up-to-8 cells this node touches
  bool any_solid = false;
  bool any_empty = false;
  for (daxa_i32 dz = -1; dz <= 0; ++dz)
  {
    for (daxa_i32 dy = -1; dy <= 0; ++dy)
    {
      for (daxa_i32 dx = -1; dx <= 0; ++dx)
      {
        if (vsb_cell_solid(occ, daxa_i32(x) + dx, daxa_i32(y) + dy, daxa_i32(z) + dz)) { any_solid = true; }
        else { any_empty = true; }
      }
    }
  }
  daxa_f32* ds = (daxa_f32*)(VSB.scratch_solid_addr);
  daxa_f32* de = (daxa_f32*)(VSB.scratch_empty_addr);
  ds[i] = any_solid ? 0.0f : VSB_INF; // squared distance to the solid region (grid units^2)
  de[i] = any_empty ? 0.0f : VSB_INF; // squared distance to the empty/outside region
}

// one thread per COLUMN of the current axis; the column (<= 64 nodes) lives in registers
[shader("compute")]
[numthreads(64, 1, 1)] void entry_voxel_sdf_axis(daxa_u32vec3 tid : SV_DispatchThreadID)
{
  daxa_u32 nd[3];
  nd[0] = VSB.cell_dims.x + 1u;
  nd[1] = VSB.cell_dims.y + 1u;
  nd[2] = VSB.cell_dims.z + 1u;
  daxa_u32 axis = VSB.axis;
  daxa_u32 u_dim = nd[(axis + 1u) % 3u];
  daxa_u32 v_dim = nd[(axis + 2u) % 3u];
  daxa_u32 col = tid.x;
  if (col >= u_dim * v_dim) { return; }
  daxa_u32 u = col % u_dim;
  daxa_u32 v = col / u_dim;
  daxa_u32 L = nd[axis];

  // node coords for column element i: coord[axis]=i, coord[axis+1]=u, coord[axis+2]=v
  daxa_u32 c[3];
  c[(axis + 1u) % 3u] = u;
  c[(axis + 2u) % 3u] = v;

  daxa_f32* fields[2];
  fields[0] = (daxa_f32*)(VSB.scratch_solid_addr);
  fields[1] = (daxa_f32*)(VSB.scratch_empty_addr);
  [unroll]
  for (daxa_u32 f = 0u; f < 2u; ++f)
  {
    daxa_f32 colv[64];
    for (daxa_u32 i = 0u; i < L; ++i)
    {
      c[axis] = i;
      colv[i] = fields[f][vsb_node_index(c[0], c[1], c[2])];
    }
    for (daxa_u32 i = 0u; i < L; ++i)
    {
      daxa_f32 best = colv[i];
      for (daxa_u32 j = 0u; j < L; ++j)
      {
        daxa_f32 d = daxa_f32(daxa_i32(i) - daxa_i32(j));
        daxa_f32 cand = colv[j] + d * d;
        best = min(best, cand);
      }
      c[axis] = i;
      // write to the OTHER scratch? No: per-axis combine is order-safe only with a temp;
      // keep the result in a second local array and write back after the loop
      // (colv stays the unmodified input while `best` accumulates)
      fields[f][vsb_node_index(c[0], c[1], c[2])] = best; // safe: this thread OWNS the column
    }
  }
}

// SURFACE-VOXEL LIST on the GPU (the other fracture-path pool): solid cells with at least
// one empty 6-neighbor, packed x | y<<8 | z<<16 | first_empty_dir<<24. SINGLE THREAD per
// shape, scanning cells in canonical bit order (x-fastest) - BYTE-IDENTICAL to the CPU
// oracle's list, so the narrow phase's iteration order (and thus every order-sensitive
// FP accumulation) is unchanged. Serial is deliberate: determinism first; cells <= 32k
// scan in microseconds at load, optimize with a scan-compaction only if fracture-scale
// grids ever make this hot. Also patches shapes[shape_index].surf_count.
[shader("compute")]
[numthreads(64, 1, 1)] void entry_voxel_surface_build(daxa_u32vec3 tid : SV_DispatchThreadID)
{
  if (tid.x != 0u) { return; }
  daxa_u32* occ = (daxa_u32*)(VSB.occupancy_addr);
  daxa_u32* surf = (daxa_u32*)(VSB.surface_addr);
  VoxelShape* shapes = (VoxelShape*)(VSB.shapes_addr);
  daxa_u32 n = 0u;
  for (daxa_u32 z = 0u; z < VSB.cell_dims.z; ++z)
  {
    for (daxa_u32 y = 0u; y < VSB.cell_dims.y; ++y)
    {
      for (daxa_u32 x = 0u; x < VSB.cell_dims.x; ++x)
      {
        if (!vsb_cell_solid(occ, daxa_i32(x), daxa_i32(y), daxa_i32(z))) { continue; }
        // first empty 6-neighbor, same probe order as the CPU oracle (-x,+x,-y,+y,-z,+z)
        daxa_i32 nb[6][3] = {{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
        for (daxa_u32 d = 0u; d < 6u; ++d)
        {
          if (!vsb_cell_solid(occ, daxa_i32(x) + nb[d][0], daxa_i32(y) + nb[d][1], daxa_i32(z) + nb[d][2]))
          {
            surf[VSB.surf_offset + n] = x | (y << 8u) | (z << 16u) | (d << 24u);
            ++n;
            break;
          }
        }
      }
    }
  }
  shapes[VSB.shape_index].surf_count = n;
}

// MASS-PROPERTY REDUCTION on the GPU (fracture primitive): count, CoM and unit inertia
// tensor of a shape from its bitmask. Single thread, canonical order (same accumulation
// order as the CPU authoring code; CoM in f32 vs the CPU's f64 is the only divergence,
// bounded by the verify oracle). Two sweeps: CoM first, then inertia about it.
[shader("compute")]
[numthreads(64, 1, 1)] void entry_voxel_inertia_reduce(daxa_u32vec3 tid : SV_DispatchThreadID)
{
  if (tid.x != 0u) { return; }
  daxa_u32* occ = (daxa_u32*)(VSB.occupancy_addr);
  VoxelShapeDerived* derived = (VoxelShapeDerived*)(VSB.derived_addr);
  daxa_f32 vs = VSB.voxel_size;

  daxa_u32 cnt = 0u;
  daxa_f32vec3 csum = daxa_f32vec3(0.0f);
  for (daxa_u32 z = 0u; z < VSB.cell_dims.z; ++z)
    for (daxa_u32 y = 0u; y < VSB.cell_dims.y; ++y)
      for (daxa_u32 x = 0u; x < VSB.cell_dims.x; ++x)
        if (vsb_cell_solid(occ, daxa_i32(x), daxa_i32(y), daxa_i32(z)))
        {
          csum += daxa_f32vec3(daxa_f32(x) + 0.5f, daxa_f32(y) + 0.5f, daxa_f32(z) + 0.5f);
          ++cnt;
        }
  daxa_f32vec3 com = csum / daxa_f32(max(cnt, 1u)) * vs;

  // parallel-axis voxel inertia per unit voxel mass: point term + own-cube term vs^2/6
  daxa_f32mat3x3 I = daxa_f32mat3x3(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  for (daxa_u32 z = 0u; z < VSB.cell_dims.z; ++z)
    for (daxa_u32 y = 0u; y < VSB.cell_dims.y; ++y)
      for (daxa_u32 x = 0u; x < VSB.cell_dims.x; ++x)
        if (vsb_cell_solid(occ, daxa_i32(x), daxa_i32(y), daxa_i32(z)))
        {
          daxa_f32vec3 c = daxa_f32vec3(daxa_f32(x) + 0.5f, daxa_f32(y) + 0.5f, daxa_f32(z) + 0.5f) * vs - com;
          daxa_f32 r2 = dot(c, c);
          daxa_f32 own = vs * vs / 6.0f;
          I[0][0] += r2 - c.x * c.x + own; I[0][1] += -c.x * c.y;       I[0][2] += -c.x * c.z;
          I[1][0] += -c.y * c.x;           I[1][1] += r2 - c.y * c.y + own; I[1][2] += -c.y * c.z;
          I[2][0] += -c.z * c.x;           I[2][1] += -c.z * c.y;       I[2][2] += r2 - c.z * c.z + own;
        }
  derived[VSB.shape_index].count = cnt;
  derived[VSB.shape_index].com = com;
  derived[VSB.shape_index].unit_inertia = I;
}

// BLAS AABB PRIMITIVES on the GPU: one Aabb per solid cell, written straight into the AS
// manager's primitive scratch at the body's range. Parallel over cells; the destination
// slot is the cell's canonical RANK (popcount of preceding occupancy bits) - exact,
// order-independent, and identical to the CPU oracle's push order. No accumulation, so
// full parallelism costs no determinism.
[shader("compute")]
[numthreads(64, 1, 1)] void entry_voxel_prims_build(daxa_u32vec3 tid : SV_DispatchThreadID)
{
  daxa_u32 bit_count = VSB.cell_dims.x * VSB.cell_dims.y * VSB.cell_dims.z;
  daxa_u32 bit = tid.x;
  if (bit >= bit_count) { return; }
  daxa_u32* occ = (daxa_u32*)(VSB.occupancy_addr);
  daxa_u32 word = occ[VSB.occ_offset + bit / 32u];
  if (((word >> (bit % 32u)) & 1u) == 0u) { return; }

  daxa_u32 rank = countbits(word & ((1u << (bit % 32u)) - 1u));
  for (daxa_u32 w = 0u; w < bit / 32u; ++w) { rank += countbits(occ[VSB.occ_offset + w]); }

  daxa_u32 x = bit % VSB.cell_dims.x;
  daxa_u32 y = (bit / VSB.cell_dims.x) % VSB.cell_dims.y;
  daxa_u32 z = bit / (VSB.cell_dims.x * VSB.cell_dims.y);
  VoxelShape* shapes = (VoxelShape*)(VSB.shapes_addr);
  daxa_f32vec3 origin = shapes[VSB.shape_index].grid_origin;
  daxa_f32 vs = VSB.voxel_size;
  daxa_f32vec3 center = origin + daxa_f32vec3(daxa_f32(x) + 0.5f, daxa_f32(y) + 0.5f, daxa_f32(z) + 0.5f) * vs;
  Aabb* prims = (Aabb*)(VSB.prims_addr);
  prims[VSB.prims_offset + rank].minimum = center - daxa_f32vec3(0.5f * vs);
  prims[VSB.prims_offset + rank].maximum = center + daxa_f32vec3(0.5f * vs);
}

[shader("compute")]
[numthreads(64, 1, 1)] void entry_voxel_sdf_finalize(daxa_u32vec3 tid : SV_DispatchThreadID)
{
  daxa_u32 nx = VSB.cell_dims.x + 1u;
  daxa_u32 ny = VSB.cell_dims.y + 1u;
  daxa_u32 nz = VSB.cell_dims.z + 1u;
  daxa_u32 total = nx * ny * nz;
  daxa_u32 i = tid.x;
  if (i >= total) { return; }
  daxa_f32 ds = ((daxa_f32*)(VSB.scratch_solid_addr))[i];
  daxa_f32 de = ((daxa_f32*)(VSB.scratch_empty_addr))[i];
  daxa_f32 sd = ds > 0.0f ? sqrt(ds) : -sqrt(de); // on-surface nodes: both 0 -> 0
  ((daxa_f32*)(VSB.sdf_addr))[VSB.sdf_offset + i] = sd * VSB.voxel_size;
}

// ----- FRACTURE kernels (carve + connected components), VoxelFracturePushConstants -----
// One-off dispatches from the host orchestrator at fracture events. Labels live in the
// EDT scratch buffer (voxel_sdf_scratch[0] reinterpreted as u32 - both users are
// fracture-rate one-offs, never concurrent).

[[vk::push_constant]] VoxelFracturePushConstants VFR;

daxa_b32 vfr_cell_solid(daxa_u32* occ, daxa_i32 x, daxa_i32 y, daxa_i32 z)
{
  if (x < 0 || y < 0 || z < 0 ||
      x >= daxa_i32(VFR.cell_dims.x) || y >= daxa_i32(VFR.cell_dims.y) || z >= daxa_i32(VFR.cell_dims.z))
  {
    return false;
  }
  daxa_u32 bit = daxa_u32(x) + daxa_u32(y) * VFR.cell_dims.x + daxa_u32(z) * VFR.cell_dims.x * VFR.cell_dims.y;
  return ((occ[VFR.occ_offset + bit / 32u] >> (bit % 32u)) & 1u) != 0u;
}

// CARVE: clear every solid cell whose center lies inside the impact sphere (grid units).
// Atomic AND per bit - parallel-safe; each thread owns its own bit, so deterministic.
[shader("compute")]
[numthreads(64, 1, 1)] void entry_voxel_carve(daxa_u32vec3 tid : SV_DispatchThreadID)
{
  daxa_u32 bit_count = VFR.cell_dims.x * VFR.cell_dims.y * VFR.cell_dims.z;
  daxa_u32 bit = tid.x;
  if (bit >= bit_count) { return; }
  daxa_u32 x = bit % VFR.cell_dims.x;
  daxa_u32 y = (bit / VFR.cell_dims.x) % VFR.cell_dims.y;
  daxa_u32 z = bit / (VFR.cell_dims.x * VFR.cell_dims.y);
  daxa_f32vec3 c = daxa_f32vec3(daxa_f32(x) + 0.5f, daxa_f32(y) + 0.5f, daxa_f32(z) + 0.5f) - VFR.carve_center;
  if (dot(c, c) > VFR.carve_radius * VFR.carve_radius) { return; }
  daxa_u32* occ = (daxa_u32*)(VFR.occupancy_addr);
  daxa_u32 _old;
  InterlockedAnd(occ[VFR.occ_offset + bit / 32u], ~(1u << (bit % 32u)), _old);
}

// VORONOI ASSIGN: per solid cell, the index of the nearest fracture site (grid-space
// Euclidean). Cells beyond voronoi_radius of the impact share a single "remainder" site
// (index = site_count) so only the impact zone shatters; the rest stays one piece. The
// constrained flood below then splits the impact zone along Voronoi boundaries. Empty
// cells get MAX_U32. Sites are host-seeded per material (stone = isotropic cloud, wood =
// strung along the grain) - see the orchestrator.
[shader("compute")]
[numthreads(64, 1, 1)] void entry_voxel_voronoi_assign(daxa_u32vec3 tid : SV_DispatchThreadID)
{
  daxa_u32 bit_count = VFR.cell_dims.x * VFR.cell_dims.y * VFR.cell_dims.z;
  daxa_u32 bit = tid.x;
  if (bit >= bit_count) { return; }
  daxa_u32* occ = (daxa_u32*)(VFR.occupancy_addr);
  daxa_u32* site_labels = (daxa_u32*)(VFR.site_labels_addr);
  bool solid = ((occ[VFR.occ_offset + bit / 32u] >> (bit % 32u)) & 1u) != 0u;
  if (!solid) { site_labels[bit] = 0xFFFFFFFFu; return; }
  daxa_u32 x = bit % VFR.cell_dims.x;
  daxa_u32 y = (bit / VFR.cell_dims.x) % VFR.cell_dims.y;
  daxa_u32 z = bit / (VFR.cell_dims.x * VFR.cell_dims.y);
  daxa_f32vec3 p = daxa_f32vec3(daxa_f32(x) + 0.5f, daxa_f32(y) + 0.5f, daxa_f32(z) + 0.5f);
  daxa_f32vec3 d0 = p - VFR.carve_center;
  if (dot(d0, d0) > VFR.voronoi_radius * VFR.voronoi_radius)
  {
    site_labels[bit] = VFR.site_count; // the shared remainder (intact chunk beyond the blast)
    return;
  }
  daxa_f32vec4* sites = (daxa_f32vec4*)(VFR.site_pos_addr);
  daxa_u32 best = 0u;
  daxa_f32 bestd = 1.0e30f;
  for (daxa_u32 i = 0u; i < VFR.site_count; ++i)
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
  daxa_u32 bit_count = VFR.cell_dims.x * VFR.cell_dims.y * VFR.cell_dims.z;
  daxa_u32 bit = tid.x;
  if (bit >= bit_count) { return; }
  daxa_u32* occ = (daxa_u32*)(VFR.occupancy_addr);
  daxa_u32* labels = (daxa_u32*)(VFR.labels_addr);
  bool solid = ((occ[VFR.occ_offset + bit / 32u] >> (bit % 32u)) & 1u) != 0u;
  labels[bit] = solid ? bit : 0xFFFFFFFFu;
}

// CONNECTED COMPONENTS, phase 2 (host-iterated): min-label propagation over solid
// 6-neighbors PLUS pointer jumping (label = labels[label]) so long serpentine paths
// collapse logarithmically within the host's fixed iteration budget. min() over a fixed
// stencil converges monotonically; atomic accesses make the in-place relaxation
// race-free, but intermediate iterations can still depend on invocation order.
[shader("compute")]
[numthreads(64, 1, 1)] void entry_voxel_flood_step(daxa_u32vec3 tid : SV_DispatchThreadID)
{
  daxa_u32 bit_count = VFR.cell_dims.x * VFR.cell_dims.y * VFR.cell_dims.z;
  daxa_u32 bit = tid.x;
  if (bit >= bit_count) { return; }
  daxa_u32* labels = (daxa_u32*)(VFR.labels_addr);
  daxa_u32 lbl;
  InterlockedAdd(labels[bit], 0u, lbl);
  if (lbl == 0xFFFFFFFFu) { return; }
  daxa_u32* occ = (daxa_u32*)(VFR.occupancy_addr);
  daxa_i32 x = daxa_i32(bit % VFR.cell_dims.x);
  daxa_i32 y = daxa_i32((bit / VFR.cell_dims.x) % VFR.cell_dims.y);
  daxa_i32 z = daxa_i32(bit / (VFR.cell_dims.x * VFR.cell_dims.y));
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
    if (!vfr_cell_solid(occ, nx, ny, nz)) { continue; }
    daxa_u32 nbit = daxa_u32(nx) + daxa_u32(ny) * VFR.cell_dims.x + daxa_u32(nz) * VFR.cell_dims.x * VFR.cell_dims.y;
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
