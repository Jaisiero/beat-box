#include "extensions.slang"
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
    daxa_f32vec3 DA = QA - PA;
    daxa_f32vec3 DB = QB - PB;
    daxa_f32vec3 r = PA - PB;
    daxa_f32 a = dot(DA, DA);
    daxa_f32 e = dot(DB, DB);
    daxa_f32 f = dot(DB, r);
    daxa_f32 c = dot(DA, r);
    daxa_f32 b = dot(DA, DB);
    daxa_f32 denom = a * e - b * b;

    // If denom is zero, try finding the closest point on the segment to the other segment
    if (denom == 0.0f) {
        // Find the closest point on segment A to segment B
        daxa_f32 t = (dot(r, DA) / a);
        t = clamp(t, 0.0f, 1.0f);
        CA = PA + DA * t;

        // Find the closest point on segment B to segment A
        daxa_f32vec3 s = CA - PB;
        daxa_f32 u = (dot(s, DB) / e);
        u = clamp(u, 0.0f, 1.0f);
        CB = PB + DB * u;
        return;
    }

    daxa_f32 TA = (b * f - c * e) / denom;
    daxa_f32 TB = (b * TA + f) / e;

    // Clamp the values of TA and TB
    TA = clamp(TA, 0.0f, 1.0f);
    TB = clamp(TB, 0.0f, 1.0f);

    CA = PA + DA * TA;
    CB = PB + DB * TB;
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

  static daxa_b32 collision_detection(const RigidBody* A, const RigidBody* B, out Manifold manifold, out daxa_u32 obb1_faces, out daxa_u32 obb2_faces) {
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

    daxa_f32mat3x3 C = mul(tx_A.get_inverse_rotation_matrix(), tx_B.get_rotation_matrix());
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
          daxa_f32 rA = e_A[i1] * abs_C[i2][j] + e_A[i2] * abs_C[i1][j];
          daxa_f32 rB = e_B[j1] * abs_C[i][j2] + e_B[j2] * abs_C[i][j1];

          // Calculate s
          daxa_f32 s = abs(t[i1] * C[i2][j] - t[i2] * C[i1][j]) - (rA + rB);

          // Axis vector
          daxa_f32vec3 axis = daxa_f32vec3(0.0f);
          axis[i1] = -C[i1][j];
          axis[i2] = C[i2][j];

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
          // obb2_faces = incident_face;
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
          manifold.contacts[i].penetration = depths[i];
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
      manifold.contacts[0].penetration = s_max;
      manifold.key_to_fp(axis, manifold.contacts[0].fp);
      manifold.contacts[0].tangent_impulse[0] = manifold.contacts[0].tangent_impulse[1] = manifold.contacts[0].normal_impulse = 0.0f;
      manifold.contacts[0].warm_start = 0u;
    }
    manifold.key = axis;


    return true;
  }
};



struct BodyLinkManifold {
  static void body_link_manifold(daxa_u32 manifold_id, daxa_u32 body_index, Ptr<daxa_u32> manifold_node_count, RigidBody* rigid_bodies, ManifoldNode* manifold_nodes)
  {
    // Add manifold to buffer
    daxa_u32 node_index = 0;
    InterlockedAdd(*manifold_node_count, 1u, node_index);

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

  static daxa_b32 find_old_collision(daxa_u32 a, daxa_u32 b, daxa_u32 first_manifold_node_index, CollisionEntry* collision_map, ManifoldNode* manifold_nodes, Ptr<Manifold> old_collisions, out Manifold collision_found, out daxa_u32 old_index) {
    daxa_u32 current = first_manifold_node_index;
    while (current != MAX_U32) {
      daxa_u32 id = manifold_nodes[current].manifold_id;
      daxa::u32 index = collision_map[id].index;
      if (old_collisions[index].obb1_index == a && old_collisions[index].obb2_index == b) {
        old_index = index;
        collision_found = old_collisions[index];
        return true;
      }
      current = manifold_nodes[current].next;
    }
    return false;
  }
};


func narrow_phase(BroadPhaseCollision broad_phase_collision, RigidBodyEntry* rigid_boy_map, RigidBody* rigid_bodies, SimConfig* sim_config, SimConfig previous_sim_config, ManifoldNode* manifold_nodes, CollisionEntry* collision_map, Manifold* collisions, RigidBodyEntry* previous_rigid_body_map, RigidBody* previous_rigid_bodies, ManifoldNode* previous_manifold_nodes, CollisionEntry* collision_map_prev, Manifold* old_collisions, BodyLink* body_links) -> void
{
  daxa::u32 rba_index = broad_phase_collision.body_a_index;
  daxa::u32 rbb_index = broad_phase_collision.body_b_index;

  RigidBody* r = (RigidBody*)(&rigid_bodies[rba_index]);
  RigidBody* r2 = (RigidBody*)(&rigid_bodies[rbb_index]);

  IslandBuilder island_builder = IslandBuilder(sim_config.active_rigid_body_count, body_links);

  Manifold manifold;

  var obb1_faces = 0u;
  var obb2_faces = 0u;

  let collision = OBBvsOBBCollision.collision_detection(r, r2, manifold, obb1_faces, obb2_faces);
  
  manifold.obb1_index = rba_index;
  manifold.obb2_index = rbb_index;
  manifold.anchor_a = r->position;
  manifold.anchor_b = r2->position;
#if defined(BB_DEBUG)
  manifold.frame_count = sim_config.frame_count;
#endif // BB_DEBUG

  // 
  if (collision)
  {
    daxa_u32 index = 0;
    InterlockedAdd(sim_config->g_c_info.collision_count, 1u, index);
    manifold.compute_basis();

    // Add collision to collision map
    collision_map[index].index = index;

    BodyLinkManifold::body_link_manifold(index, rba_index, &sim_config->manifold_node_count, rigid_bodies, manifold_nodes);

    BodyLinkManifold::body_link_manifold(index, rbb_index, &sim_config->manifold_node_count, rigid_bodies, manifold_nodes);

    daxa::u32 rba_prev_index = previous_rigid_body_map[r->id].index;
    bool rba_prev_exists = rba_prev_index != MAX_U32;
    daxa::u32 rbb_prev_index = previous_rigid_body_map[r2->id].index;
    bool rbb_prev_exists = rbb_prev_index != MAX_U32;

    daxa_b32 warm_starting = sim_config->has_flag(SimFlag::WARM_STARTING);
    if(warm_starting && rba_prev_exists && rbb_prev_exists) {
      Manifold old_manifold;
      daxa_u32 old_index = 0;
      
#if defined(BB_SIM_DEBUG)
      if(find_old_collision(rba_prev_index, rbb_prev_index, previous_sim_config.g_c_info.collision_count, old_collisions, old_manifold, old_index))
      {
#else
      if(BodyLinkManifold::find_old_collision(rba_prev_index, rbb_prev_index, previous_rigid_bodies[rba_prev_index].manifold_node_index, collision_map_prev, previous_manifold_nodes, old_collisions, old_manifold, old_index)) {
#endif
        manifold.update(old_manifold);
      } 
    }

    // Add collision to buffer
    collisions[index] = manifold;

#if !defined(BB_SIM_DEBUG)
    // ISLAND BUILDER
    // Get active rigid body indices
    daxa_u32 in_first = r->has_flag(RigidBodyFlag::DYNAMIC) ? r->active_index : MAX_U32;
    daxa_u32 in_second = r2->has_flag(RigidBodyFlag::DYNAMIC) ? r2->active_index : MAX_U32;

    // Link the bodies
    island_builder.link_bodies(in_first, in_second);
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
}