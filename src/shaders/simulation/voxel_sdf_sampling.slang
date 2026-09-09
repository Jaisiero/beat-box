#pragma once
#include "shared.inl"

// Surface indices remain in the prefix; float4 raw gradients share their slot
// numbers in the tail. Keep the host voxel_surface allocation in sync.
daxa_f32vec4* voxel_surface_gradients(daxa_u32* surface)
{
  return (daxa_f32vec4*)(surface + BB_MAX_VOXEL_SURF_COUNT);
}

// node-SDF fetch: node coords clamped into the (dims+1)^3 node grid; outside it the true
// field can only be LARGER than the border value, so extend by the clamped node distance
// (far samples read safely positive instead of a flat border copy)
daxa_f32 voxel_sdf_node(daxa_f32* sdf, VoxelShape* s, daxa_i32vec3 n)
{
  daxa_i32vec3 nc = daxa_i32vec3(clamp(n.x, 0, daxa_i32(s->dims.x)),
                                 clamp(n.y, 0, daxa_i32(s->dims.y)),
                                 clamp(n.z, 0, daxa_i32(s->dims.z)));
  daxa_u32 nx = s->dims.x + 1u;
  daxa_u32 nxy = nx * (s->dims.y + 1u);
  daxa_f32 v = sdf[s->sdf_offset + daxa_u32(nc.x) + daxa_u32(nc.y) * nx + daxa_u32(nc.z) * nxy];
  daxa_f32vec3 ext = daxa_f32vec3(daxa_f32(n.x - nc.x), daxa_f32(n.y - nc.y), daxa_f32(n.z - nc.z));
  return v + length(ext) * s->voxel_size;
}

// trilinear node-SDF sample at a body2-LOCAL point: TRUE signed distance to the shape's
// surface (world units, negative inside) plus the analytic trilinear gradient (body2-local,
// ~unit for planar surfaces). The field lives at voxel CORNERS, so a 2-voxel-thick feature
// keeps its midplane fold (0, -vs, 0) and the gradient points at the real nearest exit even
// deep inside thin features - exactly the information every cell-local depth heuristic
// (hover band / exit feasibility / dominance band) structurally lacked.
daxa_f32 voxel_sdf_sample(daxa_f32* sdf, VoxelShape* s, daxa_f32vec3 l2, out daxa_f32vec3 grad)
{
  daxa_f32 vs = s->voxel_size;
  daxa_f32vec3 gs = (l2 - s->grid_origin) / vs; // node space: node (i,j,k) at integer coords
  daxa_f32vec3 fl = floor(gs);
  daxa_i32vec3 b = daxa_i32vec3(daxa_i32(fl.x), daxa_i32(fl.y), daxa_i32(fl.z));
  daxa_f32vec3 f = gs - fl;
  daxa_f32 v000 = voxel_sdf_node(sdf, s, b);
  daxa_f32 v100 = voxel_sdf_node(sdf, s, b + daxa_i32vec3(1, 0, 0));
  daxa_f32 v010 = voxel_sdf_node(sdf, s, b + daxa_i32vec3(0, 1, 0));
  daxa_f32 v110 = voxel_sdf_node(sdf, s, b + daxa_i32vec3(1, 1, 0));
  daxa_f32 v001 = voxel_sdf_node(sdf, s, b + daxa_i32vec3(0, 0, 1));
  daxa_f32 v101 = voxel_sdf_node(sdf, s, b + daxa_i32vec3(1, 0, 1));
  daxa_f32 v011 = voxel_sdf_node(sdf, s, b + daxa_i32vec3(0, 1, 1));
  daxa_f32 v111 = voxel_sdf_node(sdf, s, b + daxa_i32vec3(1, 1, 1));
  daxa_f32 v00 = lerp(v000, v100, f.x);
  daxa_f32 v10 = lerp(v010, v110, f.x);
  daxa_f32 v01 = lerp(v001, v101, f.x);
  daxa_f32 v11 = lerp(v011, v111, f.x);
  daxa_f32 v0 = lerp(v00, v10, f.y);
  daxa_f32 v1 = lerp(v01, v11, f.y);
  daxa_f32 gx = lerp(lerp(v100 - v000, v110 - v010, f.y), lerp(v101 - v001, v111 - v011, f.y), f.z);
  daxa_f32 gy = lerp(v10 - v00, v11 - v01, f.z);
  grad = daxa_f32vec3(gx, gy, v1 - v0) / vs;
  return lerp(v0, v1, f.z);
}

