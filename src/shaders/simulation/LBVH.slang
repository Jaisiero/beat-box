#include "shared.inl"

// credits: https://developer.nvidia.com/blog/thinking-parallel-part-iii-tree-construction-gpu/
struct MortonCodeGenerator {
    // Expands a 10-bit integer into 30 bits
  // by inserting 2 zeros after each bit.
  static daxa_u32 expand_bits(daxa_u32 v) {
      v = (v * 0x00010001u) & 0xFF0000FFu;
      v = (v * 0x00000101u) & 0x0F00F00Fu;
      v = (v * 0x00000011u) & 0xC30C30C3u;
      v = (v * 0x00000005u) & 0x49249249u;
      return v;
  }

  // Calculates a 30-bit Morton code for the
  // given 3D point located within the unit cube [0,1].
  static daxa_u32 morton3D(daxa_f32vec3 p) {
      daxa_f32 x = p.x;
      daxa_f32 y = p.y;
      daxa_f32 z = p.z;
      x = min(max(x * 1024.0f, 0.0f), 1023.0f);
      y = min(max(y * 1024.0f, 0.0f), 1023.0f);
      z = min(max(z * 1024.0f, 0.0f), 1023.0f);
      daxa_u32 xx = expand_bits(daxa_u32(x));
      daxa_u32 yy = expand_bits(daxa_u32(y));
      daxa_u32 zz = expand_bits(daxa_u32(z));
      return xx * 4 + yy * 2 + zz;
  }
};