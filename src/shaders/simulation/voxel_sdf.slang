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
