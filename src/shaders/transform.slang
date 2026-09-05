#pragma once
#include "shared.inl"

extension Transform {

    daxa_f32mat3x3 get_rotation_matrix() {
      return rotation;
    }

    daxa_f32mat3x3 get_inverse_rotation_matrix() {
      return transpose(rotation);
    }

    daxa_f32vec3 get_position() {
      return position;
    }

    daxa_f32vec3 rotate(daxa_f32vec3 v) {
      return mul(v, rotation);
    }

    daxa_f32vec3 rotate_transpose(daxa_f32vec3 v) {
      return mul(v, transpose(rotation));
    }

    daxa_f32vec3 transform(daxa_f32vec3 v) {
      return mul(v, rotation) + position;
    }

    daxa_f32vec3 transform_transpose(daxa_f32vec3 v) {
      return mul(v - position, transpose(rotation));
    }

    daxa_f32vec3 get_axis(daxa_u32 i) {
      // Return the i-th row of the rotation matrix
      return daxa_f32vec3(rotation[i][0], rotation[i][1], rotation[i][2]);
    }

    daxa_f32vec3 get_inverse_axis(daxa_u32 i) {
        // For rotation matrices, the inverse is the transpose
        // So the inverse axes are the columns of the transpose
        daxa_f32mat3x3 inv_rotation = transpose(rotation);
        return daxa_f32vec3(inv_rotation[i][0], inv_rotation[i][1], inv_rotation[i][2]);
    }

    daxa_f32 get_rotation_element(daxa_u32 i, daxa_u32 j) {
      return rotation[i][j];
    }
};
