#pragma once
#include "math.hpp"

// Exposed-face mask: +X, -X, +Y, -Y, +Z, -Z. Preserve opposite faces separately;
// their average normal can be zero even though both are real contact surfaces.
FORCE_INLINE bool voxel_face_faces_direction(daxa_u32 mask, daxa_f32vec3 direction)
{
  return ((mask & 1u) != 0u && direction.x > 1e-6f) ||
         ((mask & 2u) != 0u && direction.x < -1e-6f) ||
         ((mask & 4u) != 0u && direction.y > 1e-6f) ||
         ((mask & 8u) != 0u && direction.y < -1e-6f) ||
         ((mask & 16u) != 0u && direction.z > 1e-6f) ||
         ((mask & 32u) != 0u && direction.z < -1e-6f);
}

