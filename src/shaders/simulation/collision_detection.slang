#include "voxel_contact_math.hpp"
#include "rigid_body_math.slang"
#include "transform.slang"
#include "state_copy.slang"
#include "contact_history.slang"
#include "islands.slang"

// credits: https://github.com/RandyGaul/qu3e
struct OBBvsOBBCollision {
  
  static const daxa_i32 EDGE_COUNT = 9;
  static const daxa_i32 A_X_AXIS = 0;
  static const daxa_i32 A_Y_AXIS = 1;
  static const daxa_i32 A_Z_AXIS = 2;
  static const daxa_i32 A_AXIS_NUM = A_Z_AXIS;
  static const daxa_i32 B_X_AXIS = 3;
  static const daxa_i32 B_Y_AXIS = 4;
  static const daxa_i32 B_Z_AXIS = 5;
  static const daxa_i32 MAX_AXIS_NUM = B_Z_AXIS;
  static const daxa_i32 A_X_B_X_EDGE = 6;
  static const daxa_i32 A_X_B_Y_EDGE = 7;
  static const daxa_i32 A_X_B_Z_EDGE = 8;
  static const daxa_i32 A_Y_B_X_EDGE = 9;
  static const daxa_i32 A_Y_B_Y_EDGE = 10;
  static const daxa_i32 A_Y_B_Z_EDGE = 11;
  static const daxa_i32 A_Z_B_X_EDGE = 12;
  static const daxa_i32 A_Z_B_Y_EDGE = 13;
  static const daxa_i32 A_Z_B_Z_EDGE = 14;

  enum Face {
      A_X = 0,
      A_Y = 1,
      A_Z = 2,
      B_X = 3,
      B_Y = 4,
      B_Z = 5
  };

  enum EdgeManifold {
      A_X_B_X = 6,
      A_X_B_Y = 7,
      A_X_B_Z = 8,
      A_Y_B_X = 9,
      A_Y_B_Y = 10,
      A_Y_B_Z = 11,
      A_Z_B_X = 12,
      A_Z_B_Y = 13,
      A_Z_B_Z = 14
  };

  enum Edge : daxa_u32 {
      POS_X_POS_Y = 0,
      POS_X_NEG_Y = 1,
      POS_X_POS_Z = 2,
      POS_X_NEG_Z = 3,
      NEG_X_POS_Y = 4,
      NEG_X_NEG_Y = 5,
      NEG_X_POS_Z = 6,
      NEG_X_NEG_Z = 7,
      POS_Y_POS_Z = 8,
      POS_Y_NEG_Z = 9,
      NEG_Y_POS_Z = 10,
      NEG_Y_NEG_Z = 11
  };

  struct Axis {
      daxa_i32 index;
      daxa_f32vec3 normal;
  };

  // NOTE: a Box2D-style speculative margin was attempted THREE times here and reverted
  // each time (see project log): hover placeholders stole lambdas via proximity
  // inheritance (v1), latched spurious sticking anchors and doubled the manifold
  // population (v3), and moving the existence cliff to the fattened broad boundary just
  // relocated the pair-level flicker (v3). The flicker pockets need a stateful grace
  // mechanism, not a wider stateless band.
  static daxa_b32 track_face_axis(inout daxa_i32 axis, daxa_i32 n, daxa_f32 s, inout daxa_f32 s_max, const daxa_f32vec3 normal, inout daxa_f32vec3 axis_normal) {
    if( s > 0.0f)
    {
      return true;
    }

    if (s > s_max) {
      s_max = s;
      axis = n;
      axis_normal = normal;
    }

    return false;
  }

  static daxa_b32 track_edge_axis(inout daxa_i32 axis, daxa_i32 n, daxa_f32 s, inout daxa_f32 s_max, const daxa_f32vec3 normal, inout daxa_f32vec3 axis_normal) {
    if( s > 0.0f)
    {
      return true;
    }

    daxa_f32 l = 1 / length(normal);
    s *= l;

    if (s > s_max) {
      s_max = s;
      axis = n;
      axis_normal = normal * l;
    }

    return false;
  }

  // TODO: check if this is correct
  static daxa_i32 get_axis_enum(daxa_u32 number) {
    switch(number) {
      case 0: return A_X_B_X_EDGE;
      case 1: return A_X_B_Y_EDGE;
      case 2: return A_X_B_Z_EDGE;
      case 3: return A_Y_B_X_EDGE;
      case 4: return A_Y_B_Y_EDGE;
      case 5: return A_Y_B_Z_EDGE;
      case 6: return A_Z_B_X_EDGE;
      case 7: return A_Z_B_Y_EDGE;
      case 8: return A_Z_B_Z_EDGE;
      default: return -1;
    }
  }


  static void compute_incident_face(const Transform itx, const daxa_f32vec3 e, daxa_f32vec3 n, out ClipVertex incident[MAX_INCIDENT_VERTEX_COUNT], out BoxFace incident_face) {
    // Transform the incident face to local space and flip the normal because we are interested in the face of the incident box that is facing away from the reference box
    n = -itx.rotate_transpose(n);
    // Abs the normal used to determine which axis the normal is most aligned with
    daxa_f32vec3 abs_n = abs(n);

    // Identify the dominant axis of the normal
    if(abs_n.x > abs_n.y && abs_n.x > abs_n.z) {
      // Positive x face
      if(n.x > 0.0f) {
        incident[0].v = daxa_f32vec3(e.x, e.y, -e.z);
        incident[1].v = daxa_f32vec3(e.x, e.y, e.z);
        incident[2].v = daxa_f32vec3(e.x, -e.y, e.z);
        incident[3].v = daxa_f32vec3(e.x, -e.y, -e.z);

        incident[0].f.in_incident = Edge::POS_Y_NEG_Z;
        incident[0].f.out_incident = Edge::POS_X_NEG_Y;
        incident[1].f.in_incident = Edge::POS_X_NEG_Y;
        incident[1].f.out_incident = Edge::POS_Y_POS_Z;
        incident[2].f.in_incident = Edge::POS_Y_POS_Z;
        incident[2].f.out_incident = Edge::NEG_X_NEG_Z;
        incident[3].f.in_incident = Edge::NEG_X_NEG_Z;
        incident[3].f.out_incident = Edge::POS_Y_NEG_Z;

        incident_face = BoxFace::RIGHT;
      } else { // Negative x face
        incident[0].v = daxa_f32vec3(-e.x, -e.y, e.z);
        incident[1].v = daxa_f32vec3(-e.x, e.y, e.z);
        incident[2].v = daxa_f32vec3(-e.x, e.y, -e.z);
        incident[3].v = daxa_f32vec3(-e.x, -e.y, -e.z);

        incident[0].f.in_incident = Edge::NEG_X_NEG_Y;
        incident[0].f.out_incident = Edge::NEG_Y_NEG_Z;
        incident[1].f.in_incident = Edge::NEG_Y_NEG_Z;
        incident[1].f.out_incident = Edge::POS_X_NEG_Z;
        incident[2].f.in_incident = Edge::POS_X_NEG_Z;
        incident[2].f.out_incident = Edge::NEG_Y_POS_Z;
        incident[3].f.in_incident = Edge::NEG_Y_POS_Z;
        incident[3].f.out_incident = Edge::NEG_X_NEG_Y;
        incident_face = BoxFace::LEFT;
      }
    } else if(abs_n.y > abs_n.x && abs_n.y > abs_n.z) {
      // Positive y face
      if(n.y > 0.0f) {
        incident[0].v = daxa_f32vec3(-e.x, e.y, e.z);
        incident[1].v = daxa_f32vec3(e.x, e.y, e.z);
        incident[2].v = daxa_f32vec3(e.x, e.y, -e.z);
        incident[3].v = daxa_f32vec3(-e.x, e.y, -e.z);

        incident[0].f.in_incident = Edge::POS_X_NEG_Z;
        incident[0].f.out_incident = Edge::POS_X_POS_Y;
        incident[1].f.in_incident = Edge::POS_X_POS_Y;
        incident[1].f.out_incident = Edge::POS_X_NEG_Y;
        incident[2].f.in_incident = Edge::POS_X_NEG_Y;
        incident[2].f.out_incident = Edge::POS_X_POS_Z;
        incident[3].f.in_incident = Edge::POS_X_POS_Z;
        incident[3].f.out_incident = Edge::POS_X_NEG_Z;

        incident_face = BoxFace::TOP;
      } else { // Negative y face
        incident[0].v = daxa_f32vec3(e.x, -e.y, e.z);
        incident[1].v = daxa_f32vec3(-e.x, -e.y, e.z);
        incident[2].v = daxa_f32vec3(-e.x, -e.y, -e.z);
        incident[3].v = daxa_f32vec3(e.x, -e.y, -e.z);

        // TODO: check if this is correct
        incident[0].f.in_incident = Edge::NEG_X_NEG_Z;
        incident[0].f.out_incident = Edge::NEG_X_POS_Y;
        incident[1].f.in_incident = Edge::NEG_X_POS_Y;
        incident[1].f.out_incident = Edge::NEG_X_NEG_Y;
        incident[2].f.in_incident = Edge::NEG_X_NEG_Y;
        incident[2].f.out_incident = Edge::NEG_X_POS_Z;
        // incident[3].f.in_incident = NEG_X_NEG_Y;
        incident[3].f.in_incident = Edge::NEG_X_POS_Z;
        // incident[3].f.out_incident = NEG_X_POS_Z;
        incident[3].f.out_incident = Edge::NEG_X_NEG_Z;

        incident_face = BoxFace::BOTTOM;
      }
    } else {
      // Positive z face
      if(n.z > 0.0f) {
        incident[0].v = daxa_f32vec3(-e.x, e.y, e.z);
        incident[1].v = daxa_f32vec3(-e.x, -e.y, e.z);
        incident[2].v = daxa_f32vec3(e.x, -e.y, e.z);
        incident[3].v = daxa_f32vec3(e.x, e.y, e.z);

        incident[0].f.in_incident = Edge::POS_X_POS_Y;
        incident[0].f.out_incident = Edge::NEG_Y_NEG_Z;
        incident[1].f.in_incident = Edge::NEG_Y_NEG_Z;
        incident[1].f.out_incident = Edge::NEG_X_POS_Y;
        incident[2].f.in_incident = Edge::NEG_X_POS_Y;
        incident[2].f.out_incident = Edge::POS_Y_POS_Z;
        incident[3].f.in_incident = Edge::POS_Y_POS_Z;
        incident[3].f.out_incident = Edge::POS_X_POS_Y;

        incident_face = BoxFace::FRONT;
      } else { // Negative z face
        incident[0].v = daxa_f32vec3(e.x, -e.y, -e.z);
        incident[1].v = daxa_f32vec3(-e.x, -e.y, -e.z);
        incident[2].v = daxa_f32vec3(-e.x, e.y, -e.z);
        incident[3].v = daxa_f32vec3(e.x, e.y, -e.z);

        incident[0].f.in_incident = Edge::POS_Y_NEG_Z;
        incident[0].f.out_incident = Edge::NEG_X_POS_Z;
        incident[1].f.in_incident = Edge::NEG_X_POS_Z;
        incident[1].f.out_incident = Edge::NEG_Y_POS_Z;
        incident[2].f.in_incident = Edge::NEG_Y_POS_Z;
        incident[2].f.out_incident = Edge::POS_X_POS_Z;
        incident[3].f.in_incident = Edge::POS_X_POS_Z;
        incident[3].f.out_incident = Edge::POS_Y_NEG_Z;

        incident_face = BoxFace::BACK;
      }
    }

    // Transform the incident face to world space
    for(int i = 0; i < MAX_INCIDENT_VERTEX_COUNT; ++i) {
      incident[i].v = itx.transform(incident[i].v);
      // No reference edge has clipped this incident vertex yet. Slang out arrays
      // do not zero-initialize these fields; clipping only overwrites one side,
      // and a reference/incident flip later promotes them into warm-start IDs.
      incident[i].f.in_reference = 0u;
      incident[i].f.out_reference = 0u;
    }
  }

  static void compute_reference_edges_and_basis(const daxa_f32vec3 e_r, const Transform rtx, daxa_f32vec3 n, daxa_i32 axis, out daxa_u32 clip_edges[MAX_INCIDENT_VERTEX_COUNT], out daxa_f32mat3x3 basis, out daxa_f32vec3 e, out BoxFace reference_face) {
    n = rtx.rotate_transpose(n);

    if(axis > A_AXIS_NUM) {
      axis -= (A_AXIS_NUM + 1);
    }

    daxa_f32mat3x3 r = rtx.rotation;

    switch(axis) {
      case A_X_AXIS: 
      {
        if(n.x > 0.0f) {
          clip_edges[0] = Edge::POS_X_NEG_Y;
          clip_edges[1] = Edge::POS_Y_POS_Z;
          clip_edges[2] = Edge::NEG_X_NEG_Z;
          clip_edges[3] = Edge::POS_Y_NEG_Z;

          e = daxa_f32vec3(e_r.y, e_r.z, e_r.x);
          basis = daxa_f32mat3x3(r[1], r[2], r[0]);

          reference_face = BoxFace::RIGHT;
        } else {
          clip_edges[0] = Edge::NEG_Y_NEG_Z;
          clip_edges[1] = Edge::POS_X_NEG_Z;
          clip_edges[2] = Edge::NEG_Y_POS_Z;
          clip_edges[3] = Edge::NEG_X_NEG_Y;

          e = daxa_f32vec3(e_r.z, e_r.y, e_r.x);
          basis = daxa_f32mat3x3(r[2], r[1], -r[0]);

          reference_face = BoxFace::LEFT;
        }
      }
      break;
      case A_Y_AXIS: 
      {
        if(n.y > 0.0f) {
          clip_edges[0] = Edge::POS_X_POS_Y;
          clip_edges[1] = Edge::POS_X_NEG_Y;
          clip_edges[2] = Edge::POS_X_POS_Z;
          clip_edges[3] = Edge::POS_X_NEG_Z;

          e = daxa_f32vec3(e_r.z, e_r.x, e_r.y);
          basis = daxa_f32mat3x3(r[2], r[0], r[1]);

          reference_face = BoxFace::TOP;
        } else {
          clip_edges[0] = Edge::NEG_X_POS_Y;
          clip_edges[1] = Edge::NEG_X_NEG_Y;
          clip_edges[2] = Edge::NEG_X_POS_Z;
          clip_edges[3] = Edge::NEG_X_NEG_Z;

          e = daxa_f32vec3(e_r.z, e_r.x, e_r.y);
          basis = daxa_f32mat3x3(r[2], -r[0], -r[1]);

          reference_face = BoxFace::BOTTOM;
        }

      }
      break;
      case A_Z_AXIS: 
      {
        if(n.z > 0.0f) {
          clip_edges[0] = Edge::NEG_Y_NEG_Z;
          clip_edges[1] = Edge::NEG_X_POS_Y;
          clip_edges[2] = Edge::POS_Y_POS_Z;
          clip_edges[3] = Edge::POS_X_POS_Y;

          e = daxa_f32vec3(e_r.y, e_r.x, e_r.z);
          basis = daxa_f32mat3x3(-r[1], r[0], r[2]);

          reference_face = BoxFace::FRONT;
        } else {
          clip_edges[0] = Edge::NEG_X_POS_Z;
          clip_edges[1] = Edge::NEG_Y_POS_Z;
          clip_edges[2] = Edge::POS_X_POS_Z;
          clip_edges[3] = Edge::POS_Y_NEG_Z;

          e = daxa_f32vec3(e_r.y, e_r.x, e_r.z);
          basis = daxa_f32mat3x3(-r[1], -r[0], -r[2]);

          reference_face = BoxFace::BACK;
        }

      }
      break;
      default: {
        // Invalid axis
      }
    };
  }

  static daxa_b32 in_front(daxa_f32 v) {
    return v < 0.0f;
  }

  static daxa_b32 behind(daxa_f32 v) {
    return v >= 0.0f;
  }

  static const daxa_f32 CLIP_THRESHOLD = 0.005f;

  static daxa_b32 on(daxa_f32 v) {
    return (v < CLIP_THRESHOLD) && (v > -CLIP_THRESHOLD);
  }

  static daxa_i32 ortographic_clip(daxa_f32 sign, daxa_f32 e, daxa_i32 axis, daxa_u32 clip_edge, const ClipVertex in[MAX_CONTACT_POINT_COUNT], daxa_i32 in_count, out ClipVertex out[MAX_CONTACT_POINT_COUNT]) 
  {
    daxa_i32 out_count = 0;
    ClipVertex a = in[in_count - 1];
    
    for(daxa_i32 i = 0; i < in_count; ++i) 
    {
      ClipVertex b = in[i];

      daxa_f32 da = sign * a.v[axis] - e;
      daxa_f32 db = sign * b.v[axis] - e;

      ClipVertex cv;
      
      if((in_front(da) && in_front(db)) || on(da) || on(db)) 
      {
        out[out_count++] = b;
      } 
      else if(in_front(da) && behind(db)) 
      {
        cv.f = b.f;
        cv.v = a.v + (b.v - a.v) * (da / (da - db));
        cv.f.out_reference = clip_edge;
        out[out_count++] = cv;
      } 
      else if(behind(da) && in_front(db)) 
      {
        cv.f = a.f;
        cv.v = a.v + (b.v - a.v) * (da / (da - db));
        cv.f.in_reference = clip_edge;
        out[out_count++] = cv;
        out[out_count++] = b;
      }
      a = b;
    }

    return out_count;
  }

  static daxa_i32 clip(const Transform tx, const daxa_f32vec3 e, const daxa_u32 clip_edges[MAX_INCIDENT_VERTEX_COUNT], const ClipVertex incident[MAX_INCIDENT_VERTEX_COUNT], out ClipVertex out_vertices[MAX_CONTACT_POINT_COUNT], out daxa_f32 depths[MAX_CONTACT_POINT_COUNT], out daxa_i32 error) {
    daxa_i32 in_count = MAX_INCIDENT_VERTEX_COUNT;
    daxa_i32 out_count = 0;
    ClipVertex in[MAX_CONTACT_POINT_COUNT];
    ClipVertex out[MAX_CONTACT_POINT_COUNT];

    error = 0;

    for(daxa_i32 i = 0; i < MAX_INCIDENT_VERTEX_COUNT; ++i) {
      in[i].v = tx.transform_transpose(incident[i].v);
      in[i].f = incident[i].f;
    }

    out_count = ortographic_clip(1.0f, e.x, 0, clip_edges[0], in, in_count, out);

    if(out_count == 0) {
      // DEBUGGING
      error = 10;
      return 0;
    }

    in_count = ortographic_clip(1.0f, e.y, 1, clip_edges[1], out, out_count, in);

    if(in_count == 0) {
      // DEBUGGING
      error = 11; 
      return 0;
    }

    out_count = ortographic_clip(-1.0f, e.x, 0, clip_edges[2], in, in_count, out);

    if(out_count == 0) {
      // DEBUGGING
      error = 12;
      return 0;
    }

    in_count = ortographic_clip(-1.0f, e.y, 1, clip_edges[3], out, out_count, in);

    if(in_count == 0) {
      // DEBUGGING
      error = 13;
      return 0;
    }

    // Keep incident vertices behind the reference face
    out_count = 0;
    for(daxa_i32 i = 0; i < in_count; ++i) {
      daxa_f32 d = in[i].v.z - e.z;
      if(d <= 0.0f) {
        out_vertices[out_count].v = tx.transform(in[i].v);
        out_vertices[out_count].f = in[i].f;
        depths[out_count] = d;
        ++out_count;
      }
    }

    return out_count;
  }

  static daxa_f32 c_sign(daxa_f32 x) {
    return x < 0.0f ? -1.0f : 1.0f;
  }


  static void support_edge(const Transform tx, const daxa::f32vec3 e, daxa::f32vec3 n, out daxa::f32vec3 CA, out daxa::f32vec3 CB) {
    n = tx.rotate_transpose(n);
    daxa_f32vec3 abs_n = abs(n);

    // Dominant axis
    int axis;
    if (abs_n.x <= abs_n.y && abs_n.x <= abs_n.z) {
        axis = 0; // X Axis
    } else if (abs_n.y <= abs_n.x && abs_n.y <= abs_n.z) {
        axis = 1; // Y Axis
    } else {
        axis = 2; // Z Axis
    }

    // Determine the sign of the normal
    daxa_f32vec3 v0, v1;
    daxa_f32 sign_x = c_sign(n.x);
    daxa_f32 sign_y = c_sign(n.y);
    daxa_f32 sign_z = c_sign(n.z);

    // Determine the support points
    v0 = daxa_f32vec3(sign_x * e.x, sign_y * e.y, sign_z * e.z);
    v1 = v0;

    // Adjust the support points based on the dominant axis
    if (axis == 0) {
        v0.x = e.x;
        v1.x = -e.x;
    } else if (axis == 1) {
        v0.y = e.y;
        v1.y = -e.y;
    } else {
        v0.z = e.z;
        v1.z = -e.z;
    }

    // Transform the support points to world space
    CA = tx.transform(v0);
    CB = tx.transform(v1);
  }

  static void edges_contact(
      daxa_f32vec3 PA, daxa_f32vec3 QA,
      daxa_f32vec3 PB, daxa_f32vec3 QB,
      out daxa_f32vec3 CA, out daxa_f32vec3 CB)
  {
    daxa_f32vec2 parameters = closest_segment_parameters(PA, QA, PB, QB);
    CA = PA + (QA - PA) * parameters.x;
    CB = PB + (QB - PB) * parameters.y;
  }

  static BoxFace get_flip_face(BoxFace face) {
    switch(face) {
      case BoxFace::RIGHT: return BoxFace::LEFT;
      case BoxFace::LEFT: return BoxFace::RIGHT;
      case BoxFace::TOP: return BoxFace::BOTTOM;
      case BoxFace::BOTTOM: return BoxFace::TOP;
      case BoxFace::BACK: return BoxFace::FRONT;
      case BoxFace::FRONT: return BoxFace::BACK;
      default: return BoxFace::NO_FACE;
    }
  }

  static daxa_b32 collision_detection(RigidBody* A, RigidBody* B, out Manifold manifold, out daxa_u32 obb1_faces, out daxa_u32 obb2_faces) {
    manifold.contact_count = 0;
    manifold.key = -1;
    manifold.error = 0;
    
    obb1_faces = 0;
    obb2_faces = 0;

    // Transform the OBBs to world space
    Transform tx_A = Transform(A->get_rotation_matrix(), A->position);
    Transform tx_B = Transform(B->get_rotation_matrix(), B->position);

    // Get the half extents of the OBBs
    daxa_f32vec3 e_A = A->half_extent();
    daxa_f32vec3 e_B = B->half_extent();


    Transform tx_C;
    Transform tx_abs_C;

    // SAT below indexes C[B axis][A axis]. With row-vector transforms this is
    // B * transpose(A), not transpose(A) * B (wrong for noncommuting rotations).
    daxa_f32mat3x3 C = relative_box_axes(
        tx_A.get_axis(0), tx_A.get_axis(1), tx_A.get_axis(2),
        tx_B.get_axis(0), tx_B.get_axis(1), tx_B.get_axis(2));
    daxa_f32mat3x3 abs_C = abs(C);

    // const daxa_f32 THRESHOLD_ERROR = 0.98f;
    // const daxa_f32 THRESHOLD_ERROR = 0.9999f;
    const daxa_f32 THRESHOLD_ERROR = 0.9998f;
    daxa_b32 parallel = false;
    {
      // r2 to r1
      for (daxa_u32 i = 0; i < 3; i++) {
        for (daxa_u32 j = 0; j < 3; j++) {
          if (abs_C[i][j] > THRESHOLD_ERROR) {
            parallel = true;
            break;
          }
        }
      }

      tx_C = Transform(C, daxa_f32vec3(0.0f));
      tx_abs_C = Transform(abs_C, daxa_f32vec3(0.0f));
    }

    // Calculate vector from center of OBB1 to center of OBB2 in OBB1's coordinate frame
    daxa_f32vec3 t = A->rotate_vector_inverse(B->position - A->position);
    // daxa_f32vec3 t = tx_A.rotate_transpose(B->position - A->position);

    Axis OBB1_Axes[3];
    Axis OBB2_Axes[3];

    {
      // Define axes for OBB1 and OBB2
      OBB1_Axes = {
          { A_X_AXIS, daxa_f32vec3(tx_A.get_axis(0)) }, // A_X_AXIS
          { A_Y_AXIS, daxa_f32vec3(tx_A.get_axis(1)) }, // A_Y_AXIS
          { A_Z_AXIS, daxa_f32vec3(tx_A.get_axis(2)) }  // A_Z_AXIS
      };

      OBB2_Axes= {
          { B_X_AXIS, daxa_f32vec3(tx_B.get_axis(0)) }, // B_X_AXIS
          { B_Y_AXIS, daxa_f32vec3(tx_B.get_axis(1)) }, // B_Y_AXIS
          { B_Z_AXIS, daxa_f32vec3(tx_B.get_axis(2)) }  // B_Z_AXIS
      };
    }

    // Initialize tracking variables
    daxa_f32 a_max = -FLT_MAX;
    daxa_i32 a_axis = -1;
    daxa_f32vec3 n_a = daxa_f32vec3(0.0f);

    daxa_f32 b_max = -FLT_MAX;
    daxa_i32 b_axis = -1;
    daxa_f32vec3 n_b = daxa_f32vec3(0.0f);

    // Iterate through OBB1's axes
    for(int i = 0; i < 3; ++i) {
        // Compute the separation distance
        float s = abs(t[i]) - (e_A[i] + dot(tx_abs_C.get_inverse_axis(i), e_B));

        if(track_face_axis(a_axis, 
                           OBB1_Axes[i].index, 
                           s, 
                           a_max, 
                           OBB1_Axes[i].normal, 
                           n_a)) {
            manifold.key = a_axis;
            return false; // Separating axis found
        }
    }

    // Iterate through OBB2's axes
    for(int i = 0; i < 3; ++i) {
        // Compute the projection using dot product
        float projection = dot(t, tx_C.get_axis(i));
        // Compute the separation distance
        float s = abs(projection) - (e_B[i] + dot(tx_abs_C.get_axis(i), e_A));

        if(track_face_axis(b_axis, 
                           OBB2_Axes[i].index, 
                           s, 
                           b_max, 
                           OBB2_Axes[i].normal, 
                           n_b)) {
            manifold.key = b_axis;
            return false; // Separating axis found
        }
    }

    daxa_f32 e_max = -FLT_MAX;
    daxa_i32 e_axis = -1;
    daxa_f32vec3 n_e = daxa_f32vec3(0.0f);

    // Edge cross product axes
    if(!parallel) {

      // Iterate over A's axes
      for (int i = 0; i < 3; ++i)
      {
        // Iterate over B's axes
        for (int j = 0; j < 3; ++j)
        {
          // Calculate the cross product of the axes
          int i1 = (i + 1) % 3;
          int i2 = (i + 2) % 3;
          int j1 = (j + 1) % 3;
          int j2 = (j + 2) % 3;

          // Calculate rA y rB
          daxa_f32 rA = e_A[i1] * abs_C[j][i2] + e_A[i2] * abs_C[j][i1];
          daxa_f32 rB = e_B[j1] * abs_C[j2][i] + e_B[j2] * abs_C[j1][i];

          // Calculate s
          daxa_f32 s = abs(t[i2] * C[j][i1] - t[i1] * C[j][i2]) - (rA + rB);

          // Axis vector
          daxa_f32vec3 axis = daxa_f32vec3(0.0f);
          axis[i1] = -C[j][i2];
          axis[i2] = C[j][i1];

          // NOTE: a near-parallel edge-axis guard (skip when cross_len2 < 1e-4) was tested
          // for the deep-MISS anomaly and FALSIFIED (dm events unchanged) - quantitatively
          // the cancellation noise can only flip the sign when |cross| ~ noise/|overlap|,
          // far below any sane threshold. Removed per the no-unproven-pipeline-changes law.

          // Track the axis
          if (track_edge_axis(e_axis, get_axis_enum(i * 3 + j), s, e_max, axis, n_e))
          {
            manifold.key = get_axis_enum(i * 3 + j);
            return false; // Separating axis found
          }
        }
      }
    }


    // Artificial axis bias to improve frame coherence
    const daxa_f32 rel_tolerance = 0.95f;
    const daxa_f32 abs_tolerance = 0.01f;

    daxa_i32 axis;
    daxa_f32 s_max;
    daxa_f32vec3 normal;
    daxa_f32 face_max = max(a_max, b_max);
    if(rel_tolerance * e_max > face_max + abs_tolerance) {
      axis = e_axis;
      s_max = e_max;
      normal = n_e;
    } else {
      if( rel_tolerance * b_max > a_max + abs_tolerance) {
        axis = b_axis;
        s_max = b_max;
        normal = n_b;
      } else {
        axis = a_axis;
        s_max = a_max;
        normal = n_a;
      }
    }

    // Reserve the normal direction which goes from A to B if it is not facing the same direction as the axis
    if( dot(normal, B.position - A.position) < 0.0f) {
      normal = -normal;
    }
    
    // Check if there's a collision
    if(axis == -1) {
      return false;
    }

    if(axis <= MAX_AXIS_NUM) {
      // Face axis
      Transform rtx;
      Transform itx;
      daxa_f32vec3 e_r;
      daxa_f32vec3 e_i;
      daxa_b32 flip = false;
      if(axis <= A_Z_AXIS) {
        rtx = tx_A;
        itx = tx_B;
        e_r = e_A;
        e_i = e_B;
      } else {
        rtx = tx_B;
        itx = tx_A;
        e_r = e_B;
        e_i = e_A;
        flip = true;
        normal = -normal;
      }

      BoxFace incident_face = BoxFace::NO_FACE;
      BoxFace reference_face = BoxFace::NO_FACE;

      // Compute reference and incident edge information necessary for clipping
      ClipVertex incident[MAX_INCIDENT_VERTEX_COUNT];
      compute_incident_face(itx, e_i, normal, incident, incident_face);
      daxa_u32 clip_edges[MAX_INCIDENT_VERTEX_COUNT];
      daxa_f32mat3x3 basis;
      daxa_f32vec3 e;
      compute_reference_edges_and_basis(e_r, rtx, normal, axis, clip_edges, basis, e, reference_face);

      // Clip the incident face against the reference face
      ClipVertex out_vertices[MAX_CONTACT_POINT_COUNT];
      daxa_f32 depths[MAX_CONTACT_POINT_COUNT];
      // DEBUGGING
      daxa_i32 error;
      Transform tx = Transform(basis, rtx.position);
      daxa_i32 out_num = clip(tx, e, clip_edges, incident, out_vertices, depths, error);
        
      manifold.error = error;

      // DEBUGGING
      if(error != 0) {
        manifold.key = axis;
        manifold.contact_count = 0;
        daxa_u32 mat_r = axis <= A_Z_AXIS ? A.material_index : B.material_index;
        daxa_u32 mat_i = axis <= A_Z_AXIS ? B.material_index : A.material_index;
        manifold.normal = flip ? -normal : normal;
        if(flip) {
          // flip the faces
          obb1_faces = incident_face;
          obb2_faces = reference_face;
        } else {
          obb1_faces = reference_face;
          obb2_faces = incident_face;
        }
        printf("Error: %d\n", error);
        return false; // FAIL
      }

      if(out_num > 0) {
        manifold.contact_count = out_num;
        manifold.normal = flip ? -normal : normal;
        if(flip) {
          // flip the faces
          obb1_faces = incident_face;
          obb2_faces = reference_face;
        } else {
          obb1_faces = reference_face;
          obb2_faces = incident_face;
        }

        for(daxa_i32 i = 0; i < out_num; ++i) {
          
          if(flip) {
            var tmp = out_vertices[i].f.in_reference;
            out_vertices[i].f.in_reference = out_vertices[i].f.in_incident;
            out_vertices[i].f.in_incident = tmp;
            tmp = out_vertices[i].f.out_reference;
            out_vertices[i].f.out_reference = out_vertices[i].f.out_incident;
            out_vertices[i].f.out_incident = tmp;
          }

          manifold.contacts[i].fp = out_vertices[i].f;
          manifold.contacts[i].position = out_vertices[i].v;
          // A clipped vertex cannot penetrate deeper than the SAT projection overlap (|s_max|) along
          // the reference normal -- far-side vertices (z near -e.z) otherwise report ~1 cube-length of
          // bogus depth. Clamp to s_max (the true separation) to kill the gross over-report.
          manifold.contacts[i].penetration = max(depths[i], s_max);
          manifold.contacts[i].tangent_impulse[0] = manifold.contacts[i].tangent_impulse[1] = manifold.contacts[i].normal_impulse = 0.0f;
          manifold.contacts[i].warm_start = 0u;
        }
      }
    } else {
      // Edge axis
      // Find the closest point on each edge
      normal = tx_A.rotate(normal);

      // Reverse the normal if it is not facing the same direction as the axis
      if( dot(normal, B.position - A.position) < 0.0f) {
        normal = -normal;
      }

      Transform atx = tx_A;
      Transform btx = tx_B;

      daxa_f32vec3 PA, QA;
      daxa_f32vec3 PB, QB;
      support_edge(atx, e_A, normal, PA, QA);
      support_edge(btx, e_B, -normal, PB, QB);

      daxa_f32vec3 CA, CB;
      edges_contact(PA, QA, PB, QB, CA, CB);

      manifold.contact_count = 1;
      manifold.normal = normal;

      manifold.contacts[0].position = (CA + CB) * 0.5f;
      // The edge separation s_max = s/|cross| amplifies float noise for near-parallel edges (q_tilt
      // cubes) and over-reports depth. Clamp to the true overlap along the edge normal (same bound as
      // the face path); a spurious edge axis (no real overlap) naturally clamps toward separation.
      {
        daxa_f32vec3 dd = B->position - A->position;
        daxa_f32vec3 hA = (A->maximum - A->minimum) * 0.5f;
        daxa_f32vec3 hB = (B->maximum - B->minimum) * 0.5f;
        daxa_f32 pA = hA.x * abs(dot(normal, A->rotate_vector(daxa_f32vec3(1,0,0)))) + hA.y * abs(dot(normal, A->rotate_vector(daxa_f32vec3(0,1,0)))) + hA.z * abs(dot(normal, A->rotate_vector(daxa_f32vec3(0,0,1))));
        daxa_f32 pB = hB.x * abs(dot(normal, B->rotate_vector(daxa_f32vec3(1,0,0)))) + hB.y * abs(dot(normal, B->rotate_vector(daxa_f32vec3(0,1,0)))) + hB.z * abs(dot(normal, B->rotate_vector(daxa_f32vec3(0,0,1))));
        daxa_f32 ovlp = pA + pB - abs(dot(dd, normal));
        manifold.contacts[0].penetration = max(s_max, -ovlp);
      }
      manifold.key_to_fp(axis, manifold.contacts[0].fp);
      manifold.contacts[0].tangent_impulse[0] = manifold.contacts[0].tangent_impulse[1] = manifold.contacts[0].normal_impulse = 0.0f;
      manifold.contacts[0].warm_start = 0u;
    }
    manifold.key = axis;

    // a speculative pair can clip to zero points (axis within the margin but the
    // reference-face footprint already past it): no manifold, not an empty allocation
    return manifold.contact_count > 0;
  }
};



struct BodyLinkManifold {
  static void body_link_manifold(daxa_u32 manifold_id, daxa_u32 body_index, Ptr<daxa_u32> manifold_node_count, Ptr<daxa_u32> node_overflow, RigidBody* rigid_bodies, ManifoldNode* manifold_nodes)
  {
    // Add manifold to buffer
    daxa_u32 node_index = 0;
    InterlockedAdd(*manifold_node_count, 1u, node_index);
    if (node_index >= BB_MAX_MANIFOLD_NODE_COUNT)
    {
      // node pool full: this manifold link is silently dropped (the body's manifold list is
      // truncated). Count it so saturation is OBSERVABLE (surfaced in [PERF] as node_of=).
      daxa_u32 _of; InterlockedAdd(*node_overflow, 1u, _of);
      return;
    }

    // Fill the node
    manifold_nodes[node_index].manifold_id = manifold_id;
    manifold_nodes[node_index].next = -1;

    // Insert at head using atomic operations:
    // We want: new_node->next = old_head; body->manifold_node_index = node_idx;
    daxa_u32 old_head;
    InterlockedExchange(rigid_bodies[body_index].manifold_node_index, node_index, old_head);

    // Now old_head is the previous head of the list
    if (old_head != MAX_U32) {
        manifold_nodes[node_index].next = old_head;
    }
  }

  // want_key filters the per-pair search: voxel pairs carry up to SIX manifolds (keys
  // 90-95, one per contact-face axis), and matching the wrong one would steal another
  // axis's lambdas. OBB pairs carry exactly ONE manifold, so if the exact key is gone
  // we fall back to the pair's manifold under ANY key: tilted deep stacks (the box
  // pool) flap the SAT winner between near-tied axes, and every flap used to lose the
  // whole manifold (the MISS storms that kept re-waking the pile). update_from_old
  // guards the cross-key inheritance with a normal-coherence check.
  static daxa_b32 find_old_collision(daxa_u32 a, daxa_u32 b, daxa_i32 want_key, daxa_u32 first_manifold_node_index, daxa_u32 manifold_node_count, daxa_u32 collision_count, CollisionEntry* collision_map, ManifoldNode* manifold_nodes, Manifold* old_collisions, out daxa_u32 old_index, out daxa_u32 swapped) {
    daxa_u32 current = first_manifold_node_index;
    daxa_u32 steps = 0u;
    daxa_u32 fallback = MAX_U32;
    daxa_u32 fallback_swapped = MAX_U32;
    bool exact_only = want_key >= 90 && want_key <= 95;
    swapped = 0u;
    while (current != MAX_U32 && current < manifold_node_count && steps < manifold_node_count) {
      daxa_u32 id = manifold_nodes[current].manifold_id;
      if (id >= collision_count) {
        break;
      }
      daxa::u32 index = collision_map[id].index;
      if (index >= collision_count) {
        break;
      }
      if (old_collisions[index].obb1_index == a && old_collisions[index].obb2_index == b) {
        if (old_collisions[index].key == want_key) {
          old_index = index;
          return true;
        }
        if (!exact_only) {
          fallback = index;
        }
      }
      // SWAPPED-ORDER fallback (OBB pairs): the broad phase orders pairs by CURRENT
      // sorted index (LBVH check_index), which flips when the radix sort swaps two
      // bodies across a Morton boundary - the pair's manifold then sits on the chain
      // with the roles reversed and the exact-order match above misses it, wiping the
      // whole manifold's lambdas (the deep-MISS anomaly: walk_diag found the manifolds
      // present with order swapped at every latched event). update_from_old performs
      // the role-swapped hand-off (normal flip, anchor swap, tangent sign).
      else if (!exact_only && old_collisions[index].obb1_index == b && old_collisions[index].obb2_index == a) {
        fallback_swapped = index;
      }
      daxa_i32 next = manifold_nodes[current].next;
      current = next < 0 ? MAX_U32 : daxa_u32(next);
      ++steps;
    }
    if (fallback != MAX_U32) {
      old_index = fallback;
      return true;
    }
    if (fallback_swapped != MAX_U32) {
      old_index = fallback_swapped;
      swapped = 1u;
      return true;
    }
    return false;
  }

  // DEEP-MISS diagnostic re-walk (probe only, runs on anomalous manifolds): like
  // find_old_collision but records WHY the walk ended and whether the pair's manifold
  // exists under EITHER body order. Encoding:
  //   bits 30-31 reason: 0 = chain head empty, 1 = node id out of bounds (chain
  //                      corruption), 2 = collision_map entry out of bounds, 3 = chain
  //                      end reached (pair genuinely absent from this chain)
  //   bit  29    pair manifold FOUND on this chain (any key, any body order)
  //   bits 16-28 steps walked
  //   bits 0-15  manifolds of OTHER pairs seen on the chain
  static daxa_u32 walk_diag(daxa_u32 a, daxa_u32 b, daxa_u32 head, daxa_u32 manifold_node_count, daxa_u32 collision_count, CollisionEntry* collision_map, ManifoldNode* manifold_nodes, Manifold* old_collisions)
  {
    if (head == MAX_U32) { return 0u; }
    if (head >= manifold_node_count) { return (1u << 30) | 0x8000u; } // corrupt head marker
    daxa_u32 current = head;
    daxa_u32 steps = 0u;
    daxa_u32 others = 0u;
    daxa_u32 found = 0u;
    daxa_u32 reason = 3u << 30;
    while (current != MAX_U32 && current < manifold_node_count && steps < manifold_node_count)
    {
      daxa_u32 id = manifold_nodes[current].manifold_id;
      if (id >= collision_count) { reason = 1u << 30; break; }
      daxa_u32 index = collision_map[id].index;
      if (index >= collision_count) { reason = 2u << 30; break; }
      bool is_pair = (old_collisions[index].obb1_index == a && old_collisions[index].obb2_index == b) ||
                     (old_collisions[index].obb1_index == b && old_collisions[index].obb2_index == a);
      if (is_pair) { found = 1u; } else { ++others; }
      daxa_i32 next = manifold_nodes[current].next;
      current = next < 0 ? MAX_U32 : daxa_u32(next);
      ++steps;
    }
    return reason | (found << 29) | (min(steps, 0x1FFFu) << 16) | min(others, 0xFFFFu);
  }
};


// ---- voxel-bitmask narrow phase (V1) + in-register manifold reduction (V2) ----
// Concave bodies as occupancy bitmasks: sample the SURFACE voxels of one body against the
// other's solid volume (bit tests / OBB face distance), reduce hundreds of raw samples to
// <= 8 contacts (deepest per world octant around the sampled body's center) and emit a
// standard Manifold - everything downstream (warm start, islands, PGS, AVBD, sticking
// anchors) consumes it unchanged. The voxel pools are static after scene load and are
// reached through SimConfig device addresses (the NP head is at the 128B push limit).

daxa_b32 voxel_occ_bit(daxa_u32* occ, VoxelShape* s, daxa_i32vec3 c)
{
  if (c.x < 0 || c.y < 0 || c.z < 0 ||
      c.x >= daxa_i32(s->dims.x) || c.y >= daxa_i32(s->dims.y) || c.z >= daxa_i32(s->dims.z))
  {
    return false;
  }
  daxa_u32 bit = daxa_u32(c.x) + daxa_u32(c.y) * s->dims.x + daxa_u32(c.z) * s->dims.x * s->dims.y;
  return ((occ[s->occ_offset + bit / 32u] >> (bit % 32u)) & 1u) != 0u;
}

// A zero SDF gradient has no orientation (common in one-cell-thick fragments).
// Keep the individual exposed faces: summing opposite face normals would cancel.
daxa_u32 voxel_exposed_faces(daxa_u32* occ, VoxelShape* s, daxa_i32vec3 c)
{
  daxa_u32 mask = 0u;
  if (!voxel_occ_bit(occ, s, c + daxa_i32vec3( 1, 0, 0))) mask |= 1u;
  if (!voxel_occ_bit(occ, s, c + daxa_i32vec3(-1, 0, 0))) mask |= 2u;
  if (!voxel_occ_bit(occ, s, c + daxa_i32vec3( 0, 1, 0))) mask |= 4u;
  if (!voxel_occ_bit(occ, s, c + daxa_i32vec3( 0,-1, 0))) mask |= 8u;
  if (!voxel_occ_bit(occ, s, c + daxa_i32vec3( 0, 0, 1))) mask |= 16u;
  if (!voxel_occ_bit(occ, s, c + daxa_i32vec3( 0, 0,-1))) mask |= 32u;
  return mask;
}

// occupancy gradient at cell c (points OUT of the solid); zero when fully interior
daxa_f32vec3 voxel_occ_gradient(daxa_u32* occ, VoxelShape* s, daxa_i32vec3 c)
{
  daxa_f32 gx = (voxel_occ_bit(occ, s, daxa_i32vec3(c.x - 1, c.y, c.z)) ? 1.0f : 0.0f) -
                (voxel_occ_bit(occ, s, daxa_i32vec3(c.x + 1, c.y, c.z)) ? 1.0f : 0.0f);
  daxa_f32 gy = (voxel_occ_bit(occ, s, daxa_i32vec3(c.x, c.y - 1, c.z)) ? 1.0f : 0.0f) -
                (voxel_occ_bit(occ, s, daxa_i32vec3(c.x, c.y + 1, c.z)) ? 1.0f : 0.0f);
  daxa_f32 gz = (voxel_occ_bit(occ, s, daxa_i32vec3(c.x, c.y, c.z - 1)) ? 1.0f : 0.0f) -
                (voxel_occ_bit(occ, s, daxa_i32vec3(c.x, c.y, c.z + 1)) ? 1.0f : 0.0f);
  return daxa_f32vec3(-gx, -gy, -gz); // solid-on-the-left convention -> flip to outward
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

// reduction state: the 4 DIAGONAL-EXTREME samples of the contact patch (qu3e keeps 4)
struct VoxelManifoldReducer
{
  daxa_f32vec3 pos[4];
  daxa_f32 pen[4];
  daxa_f32vec3 nrm[4];
  daxa_u32 fp_a[4];
  daxa_u32 fp_b[4];
  daxa_f32 score[4]; // extreme-direction score of the kept sample (winner selection)
  daxa_f32vec3 normal_sum;
  daxa_f32 weight_sum; // sum of |pen| weights (detects opposing-normal cancellation)
  daxa_u32 sample_count;
  daxa_u32 interior_hits; // samples landing in zero-gradient interior cells (deep overlap)
  daxa_u32 overlap_hits;  // samples whose CENTER cell is inside the other body's solid = true
                          // volume overlap (independent of exit feasibility/depth caps) — the
                          // thin-feature wedge signature: large while per-contact depths read ~mm

  [mutating] void init()
  {
    [unroll]
    for (daxa_u32 i = 0u; i < 4u; ++i) { pen[i] = 1.0e9f; fp_a[i] = 0u; score[i] = -1.0e30f; } // 1e9 = empty
    normal_sum = daxa_f32vec3(0.0f, 0.0f, 0.0f);
    weight_sum = 0.0f;
    sample_count = 0u;
    interior_hits = 0u;
    overlap_hits = 0u;
  }

  // Winner per EXTREME DIRECTION: the 4 diagonal directions of the contact-face plane
  // (the two non-axis local coordinates of the reference body). The kept voxels are the
  // OUTERMOST of the patch - its bounding-quad corners, the largest support polygon the
  // samples can offer (max lever arms against tipping; what qu3e/Box2D get from clipped
  // face corners / max-area point replacement). Four points suffice for stability
  // (user-confirmed); more just gives the penalty ramps redundant rows to fight over.
  // Score ties resolve by deeper quantized depth, then LOWEST voxel id, so winners pin
  // to fixed voxels at rest (raw deepest churned identity within depth noise).
  [mutating] void add(daxa_f32vec3 p, daxa_f32 pe, daxa_f32vec3 n, daxa_u32 fa, daxa_u32 fb,
                      daxa_f32 su, daxa_f32 sv, daxa_f32 quant)
  {
    // scores quantized to half-voxel buckets, exactly like depths used to be: raw float
    // comparisons flip winners on micro-motion (measured: settling never converged, the
    // corner pairs churned identity at 4-12 contacts/step), while bucketed scores only
    // change when the patch really moves. Within a bucket: deeper, then LOWEST voxel id.
    daxa_f32 sc4[4] = { su + sv, su - sv, -su + sv, -su - sv };
    daxa_i32 nb = daxa_i32(floor(pe / quant));
    [unroll]
    for (daxa_u32 o = 0u; o < 4u; ++o)
    {
      bool empty = pen[o] > 0.5e9f;
      daxa_i32 nb_s = daxa_i32(floor(sc4[o] / quant));
      daxa_i32 ob_s = empty ? -2147483640 : daxa_i32(floor(score[o] / quant));
      daxa_i32 ob = empty ? 0 : daxa_i32(floor(pen[o] / quant));
      if (empty || nb_s > ob_s || (nb_s == ob_s && (nb < ob || (nb == ob && fa < fp_a[o]))))
      {
        pos[o] = p; pen[o] = pe; nrm[o] = n; fp_a[o] = fa; fp_b[o] = fb; score[o] = sc4[o];
      }
    }
    daxa_f32 w = max(-pe, 0.1f * quant); // band samples (pe >~ 0) still steer the normal
    normal_sum += n * w;
    weight_sum += w;
    ++sample_count;
  }
};

// voxel body r1 (shape s1) sampled against: voxel body r2 (shape s2) when s2_is_voxel,
// otherwise r2's OBB. Fill six directional reducers in canonical sample/cell order.
// Shared SDF and cell SAT work is independent of the destination face bucket.
void voxel_sample_body(RigidBody* r1, VoxelShape* s1, RigidBody* r2, VoxelShape* s2, bool s2_is_voxel,
                           daxa_u32* occ, daxa_u32* surf, daxa_f32* sdf, inout VoxelManifoldReducer red[6])
{
  daxa_f32 vs1 = s1->voxel_size;
  // per-axis support radius of the (rotated) sample voxel in body2's frame: a flat
  // 0.5*vs1 underestimates the overlap of rotated bodies by up to sqrt(3), which showed
  // as visible standing interpenetration at rotated piece-piece contacts
  daxa_f32vec3 a0 = r2->rotate_vector_inverse(r1->rotate_vector(daxa_f32vec3(1.0f, 0.0f, 0.0f)));
  daxa_f32vec3 a1 = r2->rotate_vector_inverse(r1->rotate_vector(daxa_f32vec3(0.0f, 1.0f, 0.0f)));
  daxa_f32vec3 a2 = r2->rotate_vector_inverse(r1->rotate_vector(daxa_f32vec3(0.0f, 0.0f, 1.0f)));
  daxa_f32vec3 sup = 0.5f * vs1 * daxa_f32vec3(abs(a0.x) + abs(a1.x) + abs(a2.x),
                                               abs(a0.y) + abs(a1.y) + abs(a2.y),
                                               abs(a0.z) + abs(a1.z) + abs(a2.z));
  // contact-face tangent coordinates: the two non-axis components of the sample position
  // in body2's LOCAL frame drive the extreme-direction winner selection in the reducer
  // FULL 15-axis SAT table for sample-cube (OBB) vs B-cell (AABB), in body2's frame.
  // The per-B-axis pass design only ever REPRESENTED body2's 3 face directions; A-face
  // and edge-cross separating axes (the true MTV of every leaning-piece contact) were
  // unrepresentable, so two ~45-degree rows shared the job and could equilibrate
  // penetrated - the persistent standing interpenetration of rotated pieces. Axes and
  // projection sums depend only on the pair's relative rotation: hoisted out of the loop.
  daxa_f32 h1 = 0.5f * vs1;
  daxa_f32vec3 sat_axis[15];
  daxa_f32 sat_proj[15];
  daxa_u32 sat_n = 0u;
  if (s2_is_voxel)
  {
    daxa_f32 h2 = 0.5f * s2->voxel_size;
    daxa_f32vec3 cand[15];
    cand[0] = daxa_f32vec3(1.0f, 0.0f, 0.0f);
    cand[1] = daxa_f32vec3(0.0f, 1.0f, 0.0f);
    cand[2] = daxa_f32vec3(0.0f, 0.0f, 1.0f);
    cand[3] = a0; cand[4] = a1; cand[5] = a2;
    cand[6] = cross(a0, cand[0]); cand[7] = cross(a0, cand[1]); cand[8] = cross(a0, cand[2]);
    cand[9] = cross(a1, cand[0]); cand[10] = cross(a1, cand[1]); cand[11] = cross(a1, cand[2]);
    cand[12] = cross(a2, cand[0]); cand[13] = cross(a2, cand[1]); cand[14] = cross(a2, cand[2]);
    for (daxa_u32 k = 0u; k < 15u; ++k)
    {
      daxa_f32 len2 = dot(cand[k], cand[k]);
      if (len2 < 1e-6f) { continue; } // parallel edge pair: its face axes cover this direction
      daxa_f32vec3 L = cand[k] / sqrt(len2);
      sat_axis[sat_n] = L;
      sat_proj[sat_n] = h1 * (abs(dot(a0, L)) + abs(dot(a1, L)) + abs(dot(a2, L))) +
                        h2 * (abs(L.x) + abs(L.y) + abs(L.z));
      ++sat_n;
    }
  }
  for (daxa_u32 i = 0u; i < s1->surf_count; ++i)
  {
    daxa_u32 e = surf[s1->surf_offset + i];
    daxa_f32vec3 local = daxa_f32vec3(s1->grid_origin.x + (daxa_f32(e & 0xFFu) + 0.5f) * vs1,
                                      s1->grid_origin.y + (daxa_f32((e >> 8u) & 0xFFu) + 0.5f) * vs1,
                                      s1->grid_origin.z + (daxa_f32((e >> 16u) & 0xFFu) + 0.5f) * vs1);
    daxa_f32vec3 pw = r1->position + r1->rotate_vector(local);
    daxa_f32 pe;
    daxa_f32vec3 n_out2; // outward from body 2's solid at the sample (world)
    daxa_u32 fb;
    daxa_f32 su = 0.0f;
    daxa_f32 sv = 0.0f;
    if (s2_is_voxel)
    {
      // SDF narrow phase: TRUE signed distance + exit gradient from body2's node field.
      // Replaces the occupied/hover/exit-feasibility/dominance heuristics wholesale: those
      // all measured CELL-LOCAL depth (<= 1 cell), so a body sunk past a thin feature's
      // midplane read ~10 mm while visually overlapping ~500 mm, and its opposing-exit
      // manifolds cancelled into a stable wedge (the scene_5 persistent interpenetration).
      // Node distances are exact at grid nodes; interpolation and its gradient are
      // approximations between nodes and may be flat in solids or empty cavities.
      daxa_f32 vs2 = s2->voxel_size;
      daxa_f32vec3 l2 = r2->rotate_vector_inverse(pw - r2->position);
      // Conservative finite-cube bound before eight SDF-node reads and cell SAT.
      // Cropped fragments often overlap only a small part of each other's grids.
      daxa_f32vec3 grid_max = s2->grid_origin + daxa_f32vec3(s2->dims) * vs2;
      if (any(l2 + sup < s2->grid_origin - daxa_f32vec3(0.02f)) ||
          any(l2 - sup > grid_max + daxa_f32vec3(0.02f))) { continue; }
      // wedge-detector volume probe (unchanged semantics: sample center inside B's solid)
      daxa_f32vec3 g = (l2 - s2->grid_origin) / vs2;
      daxa_i32vec3 c = daxa_i32vec3(daxa_i32(floor(g.x)), daxa_i32(floor(g.y)), daxa_i32(floor(g.z)));
      if (voxel_occ_bit(occ, s2, c)) { for (uint face=0u;face<6u;++face) red[face].overlap_hits += 1u; }
      daxa_f32vec3 grad;
      daxa_f32 sd = voxel_sdf_sample(sdf, s2, l2, grad);
      daxa_f32 gl2 = dot(grad, grad);
      bool has_gradient = gl2 >= 1e-6f;
      if (!has_gradient && voxel_occ_bit(occ, s2, c) && voxel_exposed_faces(occ, s2, c) == 0u)
      {
        // Only a fully enclosed cell has no local surface exit. Thin fragments
        // can have a flat node field while still exposing real collision faces.
        for (uint face=0u;face<6u;++face) red[face].interior_hits += 1u;
        continue;
      }
      // A flat EMPTY cavity is not penetration. Still run cell SAT below: the sample
      // cube can touch its walls even though its center is outside the solid.
      daxa_f32vec3 gn = has_gradient ? grad / sqrt(gl2) : daxa_f32vec3(0.0f);
      // pass membership: the exit direction must have a significant component along this
      // pass's (axis, sign); diagonal exits join both their axes' manifolds (the stable
      // two-row edge wedge the old dominance band existed for)
      if (has_gradient && sd < -0.35f * vs2)
      {
        // DEEP band: only the field knows the global depth and the real nearest exit
        // (cell-local/pairwise measurements are structurally blind past one cell - the
        // wedge bug). Membership by gradient direction, as one direction per sample.
        daxa_f32 sup_n = 0.5f * vs1 * (abs(dot(gn, a0)) + abs(dot(gn, a1)) + abs(dot(gn, a2)));
        pe = sd - sup_n;
        if (pe >= 0.02f) { continue; }
        pe = max(pe, -0.5f * vs1);
        n_out2 = r2->rotate_vector(gn);
        fb = daxa_u32(c.x & 0xFF) | (daxa_u32(c.y & 0xFF) << 8u) | (daxa_u32(c.z & 0xFF) << 16u);
        for (uint face=0u;face<6u;++face) {
          uint axk=face/2u;
          float sgn_pass=(face&1u)!=0u ? 1.0f : -1.0f;
          float ga=axk==0u ? gn.x : (axk==1u ? gn.y : gn.z);
          if (sgn_pass*ga<0.45f) continue;
          su=axk==0u ? l2.y : l2.x;
          sv=axk==2u ? l2.y : l2.z;
          red[face].add(pw, pe, n_out2, e & 0xFFFFFFu, fb, su, sv, 0.5f * vs1);
        }
        continue;
      }
      // NEAR-SURFACE band (where every resting contact lives): EXACT contact by full
      // 15-axis SAT of the rotated sample cube against each occupied neighboring CELL
      // from the bitmask. The MTV gives the exact depth AND the exact direction for ANY
      // relative orientation - A-face and edge-cross contacts included, which the B-axis
      // formulas could only approximate with fighting diagonal rows. Union internal
      // faces are rejected by SIGN checks against both bodies' SDF gradients (no
      // thresholds): the MTV must point out of B's solid and against A's outward.
      if (sd > 0.867f * vs1 + 0.02f) { continue; } // support-sphere bound: cannot reach
      daxa_f32vec3 gA_l1;
      voxel_sdf_sample(sdf, s1, local, gA_l1); // A's own outward at this sample
      daxa_f32vec3 gA = r2->rotate_vector_inverse(r1->rotate_vector(gA_l1));
      daxa_f32 gA2 = dot(gA, gA);
      if (gA2 > 1e-6f) { gA = gA / sqrt(gA2); } else { gA = daxa_f32vec3(0.0f); }
      daxa_i32vec3 ca = daxa_i32vec3(daxa_i32(e & 0xFFu), daxa_i32((e >> 8u) & 0xFFu), daxa_i32((e >> 16u) & 0xFFu));
      daxa_u32 exposed_a = gA2 > 1e-6f ? 0u : voxel_exposed_faces(occ, s1, ca);
      daxa_i32vec3 clo = daxa_i32vec3(daxa_i32(floor((l2.x - sup.x - s2->grid_origin.x) / vs2)),
                                      daxa_i32(floor((l2.y - sup.y - s2->grid_origin.y) / vs2)),
                                      daxa_i32(floor((l2.z - sup.z - s2->grid_origin.z) / vs2)));
      daxa_i32vec3 chi = daxa_i32vec3(daxa_i32(floor((l2.x + sup.x - s2->grid_origin.x) / vs2)),
                                      daxa_i32(floor((l2.y + sup.y - s2->grid_origin.y) / vs2)),
                                      daxa_i32(floor((l2.z + sup.z - s2->grid_origin.z) / vs2)));
      // Out-of-grid cells are always empty; do not iterate over them.
      clo = max(clo, daxa_i32vec3(0));
      chi = min(chi, daxa_i32vec3(s2->dims) - daxa_i32vec3(1));
      for (daxa_i32 cz = clo.z; cz <= chi.z; ++cz)
      {
        for (daxa_i32 cy = clo.y; cy <= chi.y; ++cy)
        {
          for (daxa_i32 cx = clo.x; cx <= chi.x; ++cx)
          {
            daxa_i32vec3 cb = daxa_i32vec3(cx, cy, cz);
            if (!voxel_occ_bit(occ, s2, cb)) { continue; }
            daxa_f32vec3 cc = s2->grid_origin + (daxa_f32vec3(daxa_f32(cx), daxa_f32(cy), daxa_f32(cz)) + daxa_f32vec3(0.5f, 0.5f, 0.5f)) * vs2;
            daxa_u32 exposed_b = has_gradient ? 0u : voxel_exposed_faces(occ, s2, cb);
            daxa_f32vec3 d = l2 - cc;
            daxa_f32 best_ov = 1e9f;
            daxa_f32 best_raw = 1e9f;
            daxa_f32vec3 best_mtv = daxa_f32vec3(0.0f);
            bool separated = false;
            for (daxa_u32 k = 0u; k < sat_n; ++k)
            {
              daxa_f32 dl = dot(d, sat_axis[k]);
              daxa_f32 ov = sat_proj[k] - abs(dl);
              if (ov < 0.0f) { separated = true; break; }
              if (ov < best_raw) { best_raw = ov; }
              daxa_f32vec3 mtv = dl >= 0.0f ? sat_axis[k] : -sat_axis[k];
              // union filtering by sign only: a real exit points out of B's solid and
              // against A's own outward (the touching A-face must be true A surface)
              if (has_gradient ? dot(mtv, gn) <= 0.0f : !voxel_face_faces_direction(exposed_b, mtv)) { continue; }
              if (gA2 > 1e-6f)
              {
                if (dot(mtv, gA) >= 0.0f) { continue; }
              }
              else
              {
                // -MTV is A's outward direction, expressed in A's local frame.
                daxa_f32vec3 outward_a = -daxa_f32vec3(dot(mtv, a0), dot(mtv, a1), dot(mtv, a2));
                if (!voxel_face_faces_direction(exposed_a, outward_a)) { continue; }
              }
              if (ov < best_ov) { best_ov = ov; best_mtv = mtv; }
            }
            if (separated) { continue; }
            if (best_ov > 0.5e9f)
            {
              // SILENT-DROP DETECTOR: the pair truly overlaps but every axis was
              // filtered out - the one remaining route to invisible interpenetration.
              // Latched into interior_hits ("overlap with no usable direction"), which
              // feeds both the vox= PERF gauge and the code-0 center rescue.
              if (best_raw > 0.05f * vs2) { for (uint face=0u;face<6u;++face) red[face].interior_hits += 1u; }
              continue;
            }
            pe = -best_ov;
            if (pe >= 0.02f) { continue; }
            // pass membership: dominant MTV component in body2's frame buckets the
            // contact into one of the six manifolds (deterministic index-order ties)
            daxa_f32 mx = abs(best_mtv.x);
            daxa_f32 my = abs(best_mtv.y);
            daxa_f32 mz = abs(best_mtv.z);
            daxa_u32 dom = mx >= my ? (mx >= mz ? 0u : 2u) : (my >= mz ? 1u : 2u);
            daxa_f32 dsgn = (dom == 0u ? best_mtv.x : (dom == 1u ? best_mtv.y : best_mtv.z)) >= 0.0f ? 1.0f : -1.0f;
            uint face=2u*dom+(dsgn>0.0f ? 1u : 0u);
            su=dom==0u ? l2.y : l2.x;
            sv=dom==2u ? l2.y : l2.z;
            pe = max(pe, -0.5f * vs1);
            n_out2 = r2->rotate_vector(best_mtv);
            fb = daxa_u32(cx & 0xFF) | (daxa_u32(cy & 0xFF) << 8u) | (daxa_u32(cz & 0xFF) << 16u);
            red[face].add(pw, pe, n_out2, e & 0xFFFFFFu, fb, su, sv, 0.5f * vs1);
          }
        }
      }
      continue;
    }
    else
    {
      // r2 is a legacy OBB: voxel-center-with-radius vs box face distance
      daxa_f32vec3 l2 = r2->rotate_vector_inverse(pw - r2->position);
      daxa_f32vec3 half = (r2->maximum - r2->minimum) * 0.5f;
      daxa_f32vec3 center_off = (r2->maximum + r2->minimum) * 0.5f;
      l2 = l2 - center_off;
      daxa_f32vec3 q = abs(l2) - half;
      if (q.x > sup.x || q.y > sup.y || q.z > sup.z) { continue; }
      // PER-AXIS membership against the OBB faces (same dominance-band rule as the
      // voxel branch): the pass's signed face distance must be competitive with the
      // least-penetrated face, and the depth is the pass axis's own
      daxa_f32vec3 qs = q - sup;
      daxa_f32 best_q = max(qs.x, max(qs.y, qs.z));
      for (uint face=0u;face<6u;++face)
      {
        uint axk=face/2u;
        float sgn_pass=(face&1u)!=0u ? 1.0f : -1.0f;
        su=axk==0u ? l2.y : l2.x;
        sv=axk==2u ? l2.y : l2.z;
        daxa_f32 l2a = axk == 0u ? l2.x : (axk == 1u ? l2.y : l2.z);
        daxa_f32 half_a = axk == 0u ? half.x : (axk == 1u ? half.y : half.z);
        daxa_f32 sup_a = axk == 0u ? sup.x : (axk == 1u ? sup.y : sup.z);
        daxa_f32 q_pass = sgn_pass * l2a - half_a - sup_a;
        if (q_pass < best_q - 0.15f * vs1) { continue; }
        daxa_f32vec3 axis = daxa_f32vec3(axk == 0u ? 1.0f : 0.0f, axk == 1u ? 1.0f : 0.0f, axk == 2u ? 1.0f : 0.0f);
        n_out2 = r2->rotate_vector(axis * sgn_pass);
        pe = q_pass;
        fb = axk;
        // Keep the reference margin and full-voxel OBB extraction cap.
        if (pe >= 0.02f) { continue; }
        pe = max(pe, -vs1);
        red[face].add(pw, pe, n_out2, e & 0xFFFFFFu, fb, su, sv, 0.5f * vs1);
      }
    }
  }
}

// Cheap separating-axis BOOLEAN prefilter on the bodies' bounding OBBs (face axes only -
// a separating face axis PROVES disjointness, so culling here can never drop a real
// contact; edge-edge separations just fall through to sampling, which finds nothing).
// Broad-phase world AABBs are conservative for rotated bodies; this gate skips the whole
// surface-voxel loop for near-miss pairs. Small margin keeps resting manifolds alive.
daxa_b32 voxel_obb_prefilter(RigidBody* a, RigidBody* b)
{
  daxa_f32vec3 ha = (a->maximum - a->minimum) * 0.5f;
  daxa_f32vec3 hb = (b->maximum - b->minimum) * 0.5f;
  daxa_f32vec3 ca = a->position + a->rotate_vector((a->maximum + a->minimum) * 0.5f);
  daxa_f32vec3 cb = b->position + b->rotate_vector((b->maximum + b->minimum) * 0.5f);
  daxa_f32vec3 d = cb - ca;
  daxa_f32vec3 ax[3] = { a->rotate_vector(daxa_f32vec3(1, 0, 0)), a->rotate_vector(daxa_f32vec3(0, 1, 0)), a->rotate_vector(daxa_f32vec3(0, 0, 1)) };
  daxa_f32vec3 bx[3] = { b->rotate_vector(daxa_f32vec3(1, 0, 0)), b->rotate_vector(daxa_f32vec3(0, 1, 0)), b->rotate_vector(daxa_f32vec3(0, 0, 1)) };
  daxa_f32 haa[3] = { ha.x, ha.y, ha.z };
  daxa_f32 hbb[3] = { hb.x, hb.y, hb.z };
  [unroll]
  for (daxa_u32 i = 0u; i < 3u; ++i)
  {
    daxa_f32 rb = hbb[0] * abs(dot(ax[i], bx[0])) + hbb[1] * abs(dot(ax[i], bx[1])) + hbb[2] * abs(dot(ax[i], bx[2]));
    if (abs(dot(d, ax[i])) > haa[i] + rb + 0.01f) { return false; }
    daxa_f32 ra = haa[0] * abs(dot(bx[i], ax[0])) + haa[1] * abs(dot(bx[i], ax[1])) + haa[2] * abs(dot(bx[i], ax[2]));
    if (abs(dot(d, bx[i])) > hbb[i] + ra + 0.01f) { return false; }
  }
  return true;
}

// a contact point offset to a face corner is real support only where body2 actually has
// material behind it: probe one cell just inside body2 along the contact axis (voxel),
// or the face rectangle extents (OBB). Voxel-center samples had this by construction -
// their cell PASSED the occupancy test - but an offset corner can hang past a hole edge
// or face end, and its phantom depth then shoves the pair around.
daxa_b32 voxel_corner_supported(daxa_f32vec3 p, daxa_f32vec3 n_sep, RigidBody* r2, VoxelShape* s2,
                                bool s2_is_voxel, daxa_u32* occ, daxa_u32 axk)
{
  if (s2_is_voxel)
  {
    daxa_f32 vs2 = s2->voxel_size;
    daxa_f32vec3 l2 = r2->rotate_vector_inverse((p - n_sep * (0.3f * vs2)) - r2->position);
    daxa_f32vec3 g = (l2 - s2->grid_origin) / vs2;
    daxa_i32vec3 cc = daxa_i32vec3(daxa_i32(floor(g.x)), daxa_i32(floor(g.y)), daxa_i32(floor(g.z)));
    return voxel_occ_bit(occ, s2, cc);
  }
  daxa_f32vec3 half = (r2->maximum - r2->minimum) * 0.5f;
  daxa_f32vec3 co = (r2->maximum + r2->minimum) * 0.5f;
  daxa_f32vec3 l2 = r2->rotate_vector_inverse(p - r2->position) - co;
  daxa_f32 lu = axk == 0u ? l2.y : l2.x;
  daxa_f32 lv = axk == 2u ? l2.y : l2.z;
  daxa_f32 hu = axk == 0u ? half.y : half.x;
  daxa_f32 hv = axk == 2u ? half.y : half.z;
  return abs(lu) <= hu + 0.01f && abs(lv) <= hv + 0.01f;
}

// full voxel narrow phase for a candidate pair; emits the standard Manifold.
// emit one reduced cluster as a standard Manifold (normal from the cluster's own
// |pen|-weighted sample normals; key identifies the cluster for warm-start matching)
daxa_b32 voxel_emit_manifold(VoxelManifoldReducer red, bool flip, daxa_i32 key, daxa_f32vec3 n_sep, daxa_f32 vs,
                             RigidBody* r2, VoxelShape* s2, bool s2_is_voxel, daxa_u32* occ, daxa_u32 axk,
                             out Manifold manifold)
{
  manifold.contact_count = 0;
  manifold.key = key;
  manifold.error = 0;
  if (red.sample_count == 0u)
  {
    return false;
  }
  // n_sep: the EXACT contact axis in world (separation direction for the sampled body) -
  // a per-manifold constant, so the tangent basis and the persisted lambdas never see a
  // wiggling normal (the SAT cubes that slept perfectly have this same property)
  manifold.normal = flip ? n_sep : -n_sep;

  // collect surviving slots; one voxel can win several extreme directions but is kept
  // once (duplicate rows double-count its stiffness and collide warm-start identities)
  daxa_u32 kept[4];
  daxa_f32 keep_align[4];
  daxa_u32 kn = 0u;
  [unroll]
  for (daxa_u32 o = 0u; o < 4u; ++o)
  {
    if (red.pen[o] >= 0.02f) { continue; } // band contacts emit; empty slots (1e9) drop.
                                           // The band must stay WIDE: a 5mm cutoff made
                                           // marginal side-face manifolds (pen ~ +0.004)
                                           // flicker in/out, resetting their lambdas on
                                           // every reappearance (measured n4 churn bursts)
    // depths were measured along each sample's own normal but get RESOLVED along the
    // manifold normal: project them (and drop near-orthogonal stragglers, which would
    // otherwise be squeezed sideways by the full unprojected depth)
    daxa_f32 align = dot(red.nrm[o], n_sep);
    if (align <= 0.1f) { continue; }
    bool dup = false;
    for (daxa_u32 q = 0u; q < kn; ++q)
    {
      if (red.fp_a[kept[q]] == red.fp_a[o]) { dup = true; break; }
    }
    if (dup) { continue; }
    kept[kn] = o;
    keep_align[kn] = align;
    ++kn;
  }

  // FOUR support points max (user-confirmed: 4 suffice for stability; qu3e keeps 4),
  // placed at the outermost real support the patch offers:
  //   3-4 distinct extreme voxels -> their centers (the patch bounding-quad corners)
  //   2 voxels (a strip)          -> 2 face corners each (full-width rectangle; a line
  //                                  of centers has zero roll lever -> limit cycle)
  //   1 voxel                     -> its 4 face corners (the face quad)
  // Corners retreat toward the always-supported voxel center where body2 has no
  // material behind them (hole edges / face ends fed phantom depths to the tangles).
  daxa_u32 reps = kn == 1u ? 4u : (kn == 2u ? 2u : 1u);
  daxa_f32vec3 t1 = normalize(cross(n_sep, abs(n_sep.y) < 0.9f ? daxa_f32vec3(0.0f, 1.0f, 0.0f) : daxa_f32vec3(1.0f, 0.0f, 0.0f)));
  daxa_f32vec3 t2 = cross(n_sep, t1);
  daxa_f32vec3 da = (t1 + t2) * 0.318f * vs; // 0.45/sqrt(2): corner of the face footprint
  daxa_f32vec3 db = (t1 - t2) * 0.318f * vs;
  daxa_u32 out_n = 0u;
  for (daxa_u32 k = 0u; k < kn; ++k)
  {
    daxa_u32 o = kept[k];
    // diagonal choice is a property of the VOXEL (id parity), not of the kept-array
    // index: index-based parity rotated a voxel's corner pair 90 degrees whenever a
    // NEIGHBORING slot appeared/vanished, jolting the warm-started anchors
    daxa_u32 vpar = (red.fp_a[o] ^ (red.fp_a[o] >> 8u) ^ (red.fp_a[o] >> 16u)) & 1u;
    daxa_f32vec3 emitted[4];
    daxa_u32 emitted_n = 0u;
    for (daxa_u32 c = 0u; c < reps; ++c)
    {
      daxa_f32vec3 p = red.pos[o];
      if (reps > 1u)
      {
        // pair 0 (c=0,1) on the voxel's parity diagonal, pair 1 (c=2,3) on the other;
        // unsupported corners retreat toward the (always-supported) voxel center
        daxa_f32vec3 offv = (c < 2u) == (vpar == 0u) ? da : db;
        daxa_f32vec3 off = offv * ((c & 1u) == 0u ? 1.0f : -1.0f);
        p = red.pos[o] + off;
        if (!voxel_corner_supported(p, n_sep, r2, s2, s2_is_voxel, occ, axk))
        {
          p = red.pos[o] + off * 0.5f;
          if (!voxel_corner_supported(p, n_sep, r2, s2, s2_is_voxel, occ, axk)) { p = red.pos[o]; }
        }
      }
      // corners that retreated onto an already-emitted point are dropped instead of
      // doubling the spring at that spot
      bool dup_p = false;
      for (daxa_u32 q = 0u; q < emitted_n; ++q)
      {
        daxa_f32vec3 dp = p - emitted[q];
        if (dot(dp, dp) < 1.0e-12f) { dup_p = true; break; }
      }
      if (dup_p) { continue; }
      emitted[emitted_n] = p;
      ++emitted_n;
      manifold.contacts[out_n].position = p;
      manifold.contacts[out_n].penetration = red.pen[o] * keep_align[k];
      manifold.contacts[out_n].normal_impulse = 0.0f;
      manifold.contacts[out_n].tangent_impulse[0] = 0.0f;
      manifold.contacts[out_n].tangent_impulse[1] = 0.0f;
      manifold.contacts[out_n].warm_start = 0u;
      manifold.contacts[out_n].stick = 0u;
      // warm-start matching keys on the SAMPLED body's voxel id (B-cell ids churn on
      // micro-motion). Bits 0..23 = packed voxel coords; bits 24-25 = corner index, so
      // each point of one voxel keeps a distinct, stable identity
      manifold.contacts[out_n].fp.in_reference = flip ? 1u : 0u;
      manifold.contacts[out_n].fp.out_reference = red.fp_b[o];
      manifold.contacts[out_n].fp.in_incident = red.fp_a[o] | (c << 24u);
      manifold.contacts[out_n].fp.out_incident = red.fp_a[o] | (c << 24u);
      ++out_n;
    }
  }
  manifold.contact_count = daxa_i32(out_n);
  return out_n > 0u;
}

// Manifold convention: normal points obb1 -> obb2, penetration < 0 = overlap.
// Interlocked concave pairs (e.g. a cross inside a frame's hole) touch with OPPOSING
// normals; averaging those into one manifold normal cancels to a degenerate direction
// and the bodies random-walk INTO each other (the reported "collapse into one point").
// When cancellation is detected, the samples are split into two clusters around the
// deepest sample's normal and emitted as TWO manifolds (keys 99 / 98).
// Per-LOCAL-AXIS voxel narrow phase: one manifold per contact face direction of the
// NON-sampled body (code = axis*2 + sign, keys 90..95). Each manifold's normal is the
// EXACT world-rotated axis - a constant, like the SAT face normals of the cubes that
// slept perfectly. Orthogonal contact directions coexist as separate manifolds instead
// of fighting across a cluster boundary, and an edge contact emits BOTH face manifolds
// (a stable two-row wedge - previously a relaxation oscillator that flipped one face).
// qu3e-inspired: deterministic feature-derived contacts + stable per-region winners.
daxa_b32 voxel_finish_axis_manifold(SimConfig* sim_config, RigidBody* rA, RigidBody* rB,
                                        daxa_u32 code, inout VoxelManifoldReducer red, out Manifold manifold)
{
  manifold.contact_count = 0;
  manifold.key = daxa_i32(90u + code);
  manifold.error = 0;

  VoxelShape* shapes = (VoxelShape*)(sim_config->voxel_shapes_addr);
  daxa_u32* occ = (daxa_u32*)(sim_config->voxel_occupancy_addr);
  daxa_u32* surf = (daxa_u32*)(sim_config->voxel_surface_addr);
  daxa_f32* sdf = (daxa_f32*)(sim_config->voxel_sdf_addr);

  bool a_vox = rA->shape_index != 0u;
  bool b_vox = rB->shape_index != 0u;
  VoxelShape* sA = &shapes[a_vox ? rA->shape_index - 1u : 0u];
  VoxelShape* sB = &shapes[b_vox ? rB->shape_index - 1u : 0u];

  // sample from the voxel body with the smaller surface list (counts hoisted: the Slang
  // parser chokes on chained -> derefs inside && expressions, E30101)
  daxa_u32 sa_n = sA->surf_count;
  daxa_u32 sb_n = sB->surf_count;
  bool flip = !a_vox || (b_vox && sb_n < sa_n); // sampled body = obb2
  RigidBody* r1 = flip ? rB : rA;
  VoxelShape* s1 = flip ? sB : sA;
  RigidBody* r2 = flip ? rA : rB;
  VoxelShape* s2 = flip ? sA : sB;
  bool s2_is_voxel = flip ? a_vox : b_vox;

  // probe channel: true volume overlap (narrow_phase's wedge probe reads it; the emission path
  // overwrites error with the extraction-trend sentinel right before emitting)
  manifold.error = daxa_i32(red.overlap_hits);
  if (code == 0u && red.interior_hits > 0u)
  {
    // embedding latch: interior samples mean part of this body sits INSIDE the other body's
    // solid — depth that dbg_pen structurally cannot see (scene_5 "invisible interpenetration")
    InterlockedMax(sim_config->dbg_vox_interior, red.interior_hits);
  }
  // Deep-interpenetration rescue (code 0 owns it — interior cells are counted in EVERY pass).
  // Fires for PARTIAL embedding too (interior_hits >= 2 even with rim samples present), not just
  // the fully-swallowed case: a body sunk >1 cell into another's solid gets NO contacts from its
  // interior samples, and the rim contacts measure CELL-LOCAL depth (~1 cell), so the solver held
  // the body like a cork — a STABLE embedded rest state (user-confirmed in scene_5) that dbg_pen
  // reported as ~10 mm. The center-to-center rescue provides the missing extraction force and is
  // self-limiting: once interior_hits drops to 0, code 0 reverts to its normal axis manifold.
  // Natural settles never produce interior hits (7-run probe: dbg_vox_interior == 0 throughout),
  // so this is a no-op outside genuine embedding.
  if (code == 0u && (red.interior_hits >= 2u || (red.sample_count == 0u && red.interior_hits > 0u)))
  {
    daxa_f32vec3 d = r1->position - r2->position;
    daxa_f32vec3 n_res = dot(d, d) > 1e-8f ? normalize(d) : daxa_f32vec3(0.0f, 1.0f, 0.0f);
    manifold.normal = flip ? n_res : -n_res;
    manifold.contacts[0].position = (r1->position + r2->position) * 0.5f;
    manifold.contacts[0].penetration = -0.5f * s1->voxel_size;
    manifold.contacts[0].normal_impulse = 0.0f;
    manifold.contacts[0].tangent_impulse[0] = 0.0f;
    manifold.contacts[0].tangent_impulse[1] = 0.0f;
    manifold.contacts[0].warm_start = 0u;
    manifold.contacts[0].stick = 0u;
    manifold.contacts[0].fp.in_reference = 0xFFFFFFu;
    manifold.contacts[0].fp.out_reference = 0xFFFFFFu;
    manifold.contacts[0].fp.in_incident = 0xFFFFFFu;
    manifold.contacts[0].fp.out_incident = 0xFFFFFFu;
    manifold.contact_count = 1;
    return true;
  }
  if (red.sample_count == 0u)
  {
    return false;
  }

  // the manifold normal is the exact contact axis in world space (outward of body2 =
  // separation direction for the sampled body r1). The bucket's |pen|-weighted TRUE MTV
  // average replaces the raw B axis when samples exist: an A-face or edge-cross contact
  // then resolves along its REAL direction instead of two fighting diagonal axis rows.
  // At rest the winning MTV axis identity is constant and bodies do not rotate, so the
  // normal is as constant as the old axis was (the property the SAT cubes sleep on).
  daxa_u32 ax = code / 2u;
  daxa_f32 sgn = (code & 1u) != 0u ? 1.0f : -1.0f;
  daxa_f32vec3 n_local = daxa_f32vec3(ax == 0u ? sgn : 0.0f, ax == 1u ? sgn : 0.0f, ax == 2u ? sgn : 0.0f);
  daxa_f32vec3 n_world = r2->rotate_vector(n_local);
  daxa_f32 ns2 = dot(red.normal_sum, red.normal_sum);
  if (ns2 > 1e-8f) { n_world = red.normal_sum / sqrt(ns2); }
  return voxel_emit_manifold(red, flip, daxa_i32(90u + code), n_world, s1->voxel_size,
                             r2, s2, s2_is_voxel, occ, ax, manifold);
}

// Evaluate each surface sample / neighboring-cell SAT once. Each reducer sees
// exactly the same ordered subsequence as its former independent face pass.
void voxel_pair_collision_detection(SimConfig* sc, RigidBody* a, RigidBody* b,
                                    out bool have[6], out Manifold manifolds[6])
{
  VoxelShape* shapes=(VoxelShape*)sc->voxel_shapes_addr;
  VoxelShape* sa=&shapes[a->shape_index!=0u ? a->shape_index-1u : 0u];
  VoxelShape* sb=&shapes[b->shape_index!=0u ? b->shape_index-1u : 0u];
  uint a_shape=a->shape_index, b_shape=b->shape_index;
  uint a_surface_count=sa->surf_count, b_surface_count=sb->surf_count;
  bool flip=a_shape==0u || (b_shape!=0u && b_surface_count<a_surface_count);
  VoxelManifoldReducer reducers[6];
  for (uint face=0u;face<6u;++face) reducers[face].init();
  voxel_sample_body(flip ? b : a,flip ? sb : sa,flip ? a : b,flip ? sa : sb,
      (flip ? a->shape_index : b->shape_index)!=0u,
      (uint*)sc->voxel_occupancy_addr,(uint*)sc->voxel_surface_addr,
      (float*)sc->voxel_sdf_addr,reducers);
  for (uint face=0u;face<6u;++face)
    have[face]=voxel_finish_axis_manifold(sc,a,b,face,reducers[face],manifolds[face]);
}

func narrow_phase(BroadPhaseCollision broad_phase_collision, RigidBodyEntry* rigid_boy_map, RigidBody* rigid_bodies, SimConfig* sim_config, SimConfig previous_sim_config, ManifoldNode* manifold_nodes, CollisionEntry* collision_map, Manifold* collisions, RigidBodyEntry* previous_rigid_body_map, RigidBody* previous_rigid_bodies, ManifoldNode* previous_manifold_nodes, CollisionEntry* collision_map_prev, Manifold* old_collisions, BodyLink* body_links, daxa_u32 bp_index) -> void
{
  daxa::u32 rba_index = broad_phase_collision.body_a_index;
  daxa::u32 rbb_index = broad_phase_collision.body_b_index;

  RigidBody* r = &rigid_bodies[rba_index];
  RigidBody* r2 = &rigid_bodies[rbb_index];

  // static-static pairs (floor vs the scene's static blocks) produce inert manifolds the
  // solver never acts on, but the warm-start matcher chewed on them every frame (the
  // phantom persistent fresh=4 with an OBB SAT key in the probes): skip them outright
  if (!r->has_flag(RigidBodyFlag::DYNAMIC) && !r2->has_flag(RigidBodyFlag::DYNAMIC))
  {
    return;
  }

  IslandBuilder island_builder = IslandBuilder(sim_config.active_rigid_body_count, body_links);

  Manifold manifold;

  var obb1_faces = 0u;
  var obb2_faces = 0u;

  // voxel pairs emit up to SIX manifolds (one per contact-face direction in the other
  // body's local frame, keys 90..95); OBB pairs keep the single-manifold SAT path
  bool voxel_pair = r->shape_index != 0u || r2->shape_index != 0u;
  bool first_emit = true;

  // deep-MISS MONITOR: once a pair has been latched (dbg_dm_ids, persistent), report per
  // frame how far its pipeline got: 1 = this NP thread processed the pair (it WAS in the
  // broad list), 2 = detection produced a manifold, 4 = warm-start matched. An anomaly
  // frame reading 0 = broad-phase miss; 1 = SAT false at deep overlap.
  daxa_u32 mon_ids = sim_config->dbg_dm_ids;
  bool monitored = mon_ids != 0u &&
                   (((((r->id & 0xFFFFu) << 16) | (r2->id & 0xFFFFu)) == mon_ids) ||
                    ((((r2->id & 0xFFFFu) << 16) | (r->id & 0xFFFFu)) == mon_ids));
  if (monitored)
  {
    daxa_u32 _m;
    InterlockedOr(sim_config->dbg_dm_mon, 1u, _m);
  }

  // THIN-FEATURE PASS-THROUGH WEDGE resolution (the scene_5 stable-interpenetration report):
  // when a thin feature (2-cell slab/arm) penetrates PAST another thin feature's midplane, its
  // overlap samples split between the two opposing exits of the SAME axis — the +axis and -axis
  // manifolds both emit with significant depth and their forces CANCEL: a stable interpenetration
  // that dbg_pen reads as ~10 mm (cell-local depths) and dbg_vox_interior cannot see (2-cell
  // features have no interior cells). Resolution: probe the six passes first; when an axis pair
  // has BOTH sides deeper than the wedge threshold, SUPPRESS the deeper side so the shallower one
  // (least displacement = the entry side before the midplane) extracts the piece. Legitimate
  // hole-threading keeps both walls: resting clearance contacts stay far under the threshold.
  // One sample traversal fills all six reducers. Cache the resulting manifolds
  // while the wedge rule selects exits, then emit in the original face order.
  bool axis_have[6];
  Manifold vox_cache[6];
  for (daxa_u32 pi = 0u; pi < 6u; ++pi) { axis_have[pi] = true; }
  if (voxel_pair)
  {
    // Bounds are unchanged across the six face passes. Reject the pair once,
    // before sampling, while retaining the same tests and contact emission order.
    if (!voxel_obb_prefilter(r, r2)) { return; }

    // volume + separation criterion (v2 — the v1 same-axis-pair depth rule was blind to the
    // user's TILTED wedge, whose per-contact cell-local depths read only ~10 mm): the probe's
    // overlap_hits counts samples truly INSIDE the other body's solid. A resting or threading
    // contact keeps sample centers OUTSIDE the solid (hover band), so overlap stays ~0; a
    // pass-through wedge has a large overlap volume regardless of its shallow contact depths.
    // When embedded, SUPPRESS every manifold whose normal opposes the pair's separation
    // direction (emitted-normal convention: separation === normalize(rB - rA), both flip cases)
    // so the surviving contacts push the bodies APART instead of cancelling.
    voxel_pair_collision_detection(sim_config,r,r2,axis_have,vox_cache);
    daxa_u32 overlap = 0u;
    for (daxa_u32 pi = 0u; pi < 6u; ++pi)
    {
      overlap = max(overlap, daxa_u32(max(vox_cache[pi].error, 0)));
    }
    if (overlap >= 2u) // edge/sliver wedges (a frame edge sunk into an arm) overlap only a few
                       // voxel centers despite being visually deep — 2 centers ~ 0.25 wu^3 is
                       // already far above anything a resting/threading contact produces (0)
    {
      daxa_f32vec3 d12 = r2->position - r->position;
      if (dot(d12, d12) > 1e-8f)
      {
        daxa_f32vec3 sep = normalize(d12);
        for (daxa_u32 pi = 0u; pi < 6u; ++pi)
        {
          if (axis_have[pi] && dot(vox_cache[pi].normal, sep) < -0.2f)
          {
            axis_have[pi] = false;
            daxa_u32 _w; InterlockedAdd(sim_config->dbg_vox_wedge, 1u, _w);
          }
        }
      }
    }
  }

  for (daxa_u32 emit_i = 0u; emit_i < 6u; ++emit_i)
  {
  var have = false;
  if (voxel_pair)
  {
    if (!axis_have[emit_i]) { continue; }
    manifold = vox_cache[emit_i];
    have = true;
  }
  else
  {
    if (emit_i > 0u) { break; }
    have = OBBvsOBBCollision.collision_detection(r, r2, manifold, obb1_faces, obb2_faces);
    // EXTRACTION CAP - same lesson the voxel path learned ("full-voxel corrections
    // pop"): rained cubes punch 0.2-0.9 deep into the pile, and AVBD's post-stabilize
    // corrects the stored C0 almost fully in one positional step. Uncapped, that is a
    // teleport that re-buries the neighbors and avalanches the crust (the box pool
    // never converged: violent corrections, re-waking waves). Capped, the burial
    // extracts smoothly over a few frames; resting pens (mm) never touch the cap.
    if (have)
    {
      for (daxa_i32 ci = 0; ci < manifold.contact_count; ++ci)
      {
        manifold.contacts[ci].penetration = max(manifold.contacts[ci].penetration, -0.25f);
      }
    }
    // NOTE: a post-clip CONTACT-SET REDUCTION (voxel-style <=4 stable corners, quantized
    // scores) was tried here in THREE variants (2026-06-12, patch at
    // scratch/sat_clip_reduction.patch): sliver->2-extremes is a rotational hinge (deep
    // edge-on pockets rock forever at the extraction cap); always-4-diagonals spuriously
    // shrinks legit 4-sets when one vertex wins two diagonals; and the cc>4-only gate
    // still wake-storm cycles. One lucky run hit full sleep at 98 (best ever), the other
    // four runs stalled or regressed - the raw clip output + the fp/proximity matching
    // passes remain the most stable known configuration. The recurring law held again:
    // with persisted-lambda, do NOT reshape contact identity to chase convergence.
  }
  if (!have) { continue; }
  if (monitored)
  {
    daxa_u32 _m;
    InterlockedOr(sim_config->dbg_dm_mon, 2u, _m);
  }
  manifold.obb1_index = rba_index;
  manifold.obb2_index = rbb_index;
  manifold.anchor_a = r->position;
  manifold.anchor_b = r2->position;
  // extraction-trend sentinel: "unknown" until update_from_old measures a frame delta;
  // the sleep depth veto treats unknown as still-moving (conservative)
  manifold.error = 0x7FFFFFFF;
#if defined(BB_DEBUG)
  manifold.frame_count = sim_config.frame_count;
#endif // BB_DEBUG

  {
    daxa_u32 index = 0;
    InterlockedAdd(sim_config->g_c_info.collision_count, 1u, index);
    if (index >= BB_MAX_COLLISION_COUNT)
    {
      // buffer full: this collision is silently dropped. Count it so saturation is OBSERVABLE
      // (surfaced in [PERF] as contact_of=) instead of a body losing support with zero signal.
      daxa_u32 _of; InterlockedAdd(sim_config->dbg_contact_overflow, 1u, _of);
      return;
    }
    manifold.compute_basis();

    // Add collision to collision map
    collision_map[index].index = index;

    BodyLinkManifold::body_link_manifold(index, rba_index, &sim_config->manifold_node_count, &sim_config->dbg_node_overflow, rigid_bodies, manifold_nodes);

    BodyLinkManifold::body_link_manifold(index, rbb_index, &sim_config->manifold_node_count, &sim_config->dbg_node_overflow, rigid_bodies, manifold_nodes);

    daxa::u32 rba_prev_index = previous_rigid_body_map[r->id].index;
    bool rba_prev_exists = rba_prev_index < previous_sim_config.rigid_body_count;
    daxa::u32 rbb_prev_index = previous_rigid_body_map[r2->id].index;
    bool rbb_prev_exists = rbb_prev_index < previous_sim_config.rigid_body_count;

    daxa_b32 warm_starting = sim_config->has_flag(SimFlag::WARM_STARTING);
    bool matched_old = false;
    daxa_i32 old_cc_dbg = -1; // churn-anatomy probe: previous contact count of the matched manifold
    if(warm_starting && rba_prev_exists && rbb_prev_exists) {
      daxa_u32 old_index = 0;
      daxa_u32 old_swapped = 0u;
      RigidBody* prev_rba = &previous_rigid_bodies[rba_prev_index];
      daxa_u32 previous_manifold_node_count = min(previous_sim_config.manifold_node_count, BB_MAX_MANIFOLD_NODE_COUNT);
      daxa_u32 previous_collision_count = min(previous_sim_config.g_c_info.collision_count, BB_MAX_COLLISION_COUNT);

#if defined(BB_SIM_DEBUG)
      if(BodyLinkManifold::find_old_collision(rba_prev_index, rbb_prev_index, manifold.key, prev_rba->manifold_node_index, previous_manifold_node_count, previous_collision_count, collision_map_prev, previous_manifold_nodes, old_collisions, old_index, old_swapped))
      {
#else
      if(BodyLinkManifold::find_old_collision(rba_prev_index, rbb_prev_index, manifold.key, prev_rba->manifold_node_index, previous_manifold_node_count, previous_collision_count, collision_map_prev, previous_manifold_nodes, old_collisions, old_index, old_swapped)) {
#endif
        matched_old = true;
        old_cc_dbg = old_collisions[old_index].contact_count;
        manifold.update_from_old(&old_collisions[old_index], old_swapped);
      }
    }
    if (monitored && matched_old)
    {
      daxa_u32 _m;
      InterlockedOr(sim_config->dbg_dm_mon, 4u, _m);
    }

    // DEEP-MISS probe: a pair >= 10cm interpenetrated MUST have existed last frame -
    // failing to find its manifold is the open existence-flicker anomaly. The FIRST
    // event EVER latches the pair persistently (dm_ids survives frames); from then on
    // the monitor below reports, per frame, where that pair's pipeline stops.
    if (warm_starting && rba_prev_exists && rbb_prev_exists && !matched_old)
    {
      daxa_f32 deepest = 0.0f;
      for (daxa_i32 ci = 0; ci < manifold.contact_count; ++ci)
      {
        deepest = min(deepest, manifold.contacts[ci].penetration);
      }
      if (deepest < -0.1f)
      {
        daxa_u32 prev_events = 0u;
        InterlockedAdd(sim_config->dbg_dm_count, 1u, prev_events);
        if (prev_events == 0u && sim_config->dbg_dm_ids == 0u)
        {
          daxa_u32 pmnc = min(previous_sim_config.manifold_node_count, BB_MAX_MANIFOLD_NODE_COUNT);
          daxa_u32 pcc = min(previous_sim_config.g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
          sim_config->dbg_dm_ids = ((r->id & 0xFFFFu) << 16) | (r2->id & 0xFFFFu);
          sim_config->dbg_dm_walk_a = BodyLinkManifold::walk_diag(
              rba_prev_index, rbb_prev_index, previous_rigid_bodies[rba_prev_index].manifold_node_index,
              pmnc, pcc, collision_map_prev, previous_manifold_nodes, old_collisions);
          sim_config->dbg_dm_walk_b = BodyLinkManifold::walk_diag(
              rba_prev_index, rbb_prev_index, previous_rigid_bodies[rbb_prev_index].manifold_node_index,
              pmnc, pcc, collision_map_prev, previous_manifold_nodes, old_collisions);
        }
      }
    }

    // identity-churn probe: contacts that failed BOTH the fp match and proximity
    // inheritance re-enter the solver with lambda=0 (the "dancing blue points" of the
    // TAB debug view, which tints by warm_start). At rest this must read ZERO.
    {
      daxa_u32 fresh = 0u;
      for (daxa_i32 ci = 0; ci < manifold.contact_count; ++ci)
      {
        if (manifold.contacts[ci].warm_start == 0u) { ++fresh; }
        daxa_f32 cpen = manifold.contacts[ci].penetration;
        if (cpen < 0.0f)
        {
          daxa_u32 prev_pen;
          InterlockedMax(sim_config->dbg_pen, daxa_u32(min(-cpen * 1000.0f, 1.0e9f)), prev_pen);
          // count HOW MANY contacts are deep (vs dbg_pen = single saturated max): tells a whole
          // collapsed layer apart from one wedged outlier among the 432 cubes.
          daxa_u32 prev_d;
          if (-cpen > 0.1f) InterlockedAdd(sim_config->dbg_deep100, 1u, prev_d);
          if (-cpen > 0.2f) InterlockedAdd(sim_config->dbg_deep200, 1u, prev_d);
        }
      }
      if (fresh > 0u)
      {
        InterlockedAdd(sim_config->dbg_fresh, fresh);
        // MISS anatomy histogram: when the whole manifold failed to match, re-walk the
        // previous chain and record WHY (matcher bug vs existence flicker vs corruption).
        // The ftag culprit latch showed rest churn is dominated by whole-manifold MISSes;
        // this splits them by cause so the fix targets the real one.
        if (!matched_old && warm_starting && rba_prev_exists && rbb_prev_exists)
        {
          daxa_u32 pmnc2 = min(previous_sim_config.manifold_node_count, BB_MAX_MANIFOLD_NODE_COUNT);
          daxa_u32 pcc2 = min(previous_sim_config.g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
          daxa_u32 wd = BodyLinkManifold::walk_diag(
              rba_prev_index, rbb_prev_index, previous_rigid_bodies[rba_prev_index].manifold_node_index,
              pmnc2, pcc2, collision_map_prev, previous_manifold_nodes, old_collisions);
          daxa_u32 _mh;
          if ((wd & (1u << 29)) != 0u) { InterlockedAdd(sim_config->dbg_miss_present, 1u, _mh); }
          else
          {
            daxa_u32 reason = wd >> 30;
            if (reason == 3u) { InterlockedAdd(sim_config->dbg_miss_absent, 1u, _mh); }
            else if (reason == 0u) { InterlockedAdd(sim_config->dbg_miss_emptyhead, 1u, _mh); }
            else { InterlockedAdd(sim_config->dbg_miss_corrupt, 1u, _mh); }
          }
        }
        // culprit latch: pair + key + count of one churning manifold (Max = stable pick).
        // Bit 31 distinguishes "no old manifold FOUND for this pair+key" (key flap /
        // first touch) from "found, but these contacts failed both matching passes".
        daxa_u32 tag = (min(rba_index, 0x1FFu) << 22u) | (min(rbb_index, 0x3FFu) << 12u) |
                       ((daxa_u32(manifold.key) & 0xFFu) << 4u) | min(fresh, 0xFu) |
                       (matched_old ? 0u : 0x80000000u);
        InterlockedMax(sim_config->dbg_fresh_tag, tag);
        // churn ANATOMY latch (highest-fresh manifold wins): new contact count vs the
        // previous frame's. fresh == new_cc - old_cc -> the contact SET membership flaps
        // (clip emits a different set; extras can never match). fresh > that surplus ->
        // identity/distance failures on contacts that DO have an old counterpart.
        daxa_u32 anat = (min(fresh, 0xFu) << 28u) |
                        (min(daxa_u32(max(manifold.contact_count, 0)), 0xFu) << 24u) |
                        ((old_cc_dbg >= 0 ? 1u : 0u) << 23u) |
                        (min(daxa_u32(max(old_cc_dbg, 0)), 0xFu) << 19u) |
                        ((min(rba_index, 0x1FFu) << 10u) | min(rbb_index, 0x3FFu)) & 0x7FFFFu;
        InterlockedMax(sim_config->dbg_pad1, anat);
      }
    }

    // Add collision to buffer
    write_manifold_fields_from_value(&collisions[index], manifold);

#if !defined(BB_SIM_DEBUG)
    // ISLAND BUILDER
    // Get active rigid body indices
    daxa_u32 in_first = r->has_flag(RigidBodyFlag::DYNAMIC) ? r->active_index : MAX_U32;
    daxa_u32 in_second = r2->has_flag(RigidBodyFlag::DYNAMIC) ? r2->active_index : MAX_U32;

    // Link the bodies (once per PAIR: the union-find CAS loop is not safe to re-enter
    // from the same thread for an already-merged pair - it hung the GPU on manifold 2)
    if (first_emit)
    {
      island_builder.link_bodies(in_first, in_second);
      first_emit = false;
    }
    // ISLAND BUILDER
#endif // BB_SIM_DEBUG


#if defined(BB_DEBUG)
    if(obb1_faces != 0) {
      InterlockedOr(r->face_collided, obb1_faces);
      r->flags |= RigidBodyFlag::COLLIDING;
    }
    if(obb2_faces != 0) {
      InterlockedOr(r2->face_collided, obb2_faces);
      r2->flags |= RigidBodyFlag::COLLIDING;
    }
#endif // BB_DEBUG
  }
  } // per-manifold emit loop
}
