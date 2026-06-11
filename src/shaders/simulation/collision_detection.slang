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

    // Closest points between the edge SEGMENTS, not the infinite lines (Ericson 5.1.9).
    // The parameters must be clamped to [0,1]: for nearly-parallel edges (settled neighbor
    // cubes jiggle below the SAT `parallel` guard) denom ~ sin^2(theta) -> the unclamped
    // line-line solution puts the contact point hundreds of units off-body. A warm-started
    // impulse applied at that lever arm injects enormous angular velocity and the rest of
    // the contact sweep converts it into a velocity explosion (bodies ejected/tunneling).
    daxa_f32 TA = (b * f - c * e) / denom;
    TA = clamp(TA, 0.0f, 1.0f);
    daxa_f32 TB = (b * TA + f) / e;

    // Re-project after clamping TB so the pair is the true segment-segment closest pair.
    if (TB < 0.0f) {
        TB = 0.0f;
        TA = clamp(-c / a, 0.0f, 1.0f);
    } else if (TB > 1.0f) {
        TB = 1.0f;
        TA = clamp((b - c) / a, 0.0f, 1.0f);
    }

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
          daxa_f32 rA = e_A[i1] * abs_C[j][i2] + e_A[i2] * abs_C[j][i1];
          daxa_f32 rB = e_B[j1] * abs_C[j2][i] + e_B[j2] * abs_C[j1][i];

          // Calculate s
          daxa_f32 s = abs(t[i2] * C[j][i1] - t[i1] * C[j][i2]) - (rA + rB);

          // Axis vector
          daxa_f32vec3 axis = daxa_f32vec3(0.0f);
          axis[i1] = -C[j][i2];
          axis[i2] = C[j][i1];

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
    if (node_index >= BB_MAX_MANIFOLD_NODE_COUNT)
    {
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

  // want_key filters the per-pair search: voxel pairs carry up to TWO manifolds (keys
  // 99/98, opposing-normal clusters), and matching the wrong one fails the key check in
  // update_from_old, silently disabling warm-starting for the pair.
  static daxa_b32 find_old_collision(daxa_u32 a, daxa_u32 b, daxa_i32 want_key, daxa_u32 first_manifold_node_index, daxa_u32 manifold_node_count, daxa_u32 collision_count, CollisionEntry* collision_map, ManifoldNode* manifold_nodes, Manifold* old_collisions, out daxa_u32 old_index) {
    daxa_u32 current = first_manifold_node_index;
    daxa_u32 steps = 0u;
    while (current != MAX_U32 && current < manifold_node_count && steps < manifold_node_count) {
      daxa_u32 id = manifold_nodes[current].manifold_id;
      if (id >= collision_count) {
        return false;
      }
      daxa::u32 index = collision_map[id].index;
      if (index >= collision_count) {
        return false;
      }
      if (old_collisions[index].obb1_index == a && old_collisions[index].obb2_index == b &&
          old_collisions[index].key == want_key) {
        old_index = index;
        return true;
      }
      daxa_i32 next = manifold_nodes[current].next;
      current = next < 0 ? MAX_U32 : daxa_u32(next);
      ++steps;
    }
    return false;
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

// reduction state: deepest sample per world octant around body A's position
struct VoxelManifoldReducer
{
  daxa_f32vec3 pos[8];
  daxa_f32 pen[8];
  daxa_f32vec3 nrm[8];
  daxa_u32 fp_a[8];
  daxa_u32 fp_b[8];
  daxa_f32vec3 normal_sum;
  daxa_f32 weight_sum; // sum of |pen| weights (detects opposing-normal cancellation)
  daxa_u32 sample_count;
  daxa_u32 interior_hits; // samples landing in zero-gradient interior cells (deep overlap)

  [mutating] void init()
  {
    [unroll]
    for (daxa_u32 i = 0u; i < 8u; ++i) { pen[i] = 1.0e9f; fp_a[i] = 0u; } // 1e9 = empty
    normal_sum = daxa_f32vec3(0.0f, 0.0f, 0.0f);
    weight_sum = 0.0f;
    sample_count = 0u;
    interior_hits = 0u;
  }

  // Winner per octant: depth quantized to half a voxel, ties broken by LOWEST voxel id.
  // Raw "deepest wins" churns at rest (many samples sit within depth noise of each other,
  // so the winning voxel - and with it the warm-start feature id - swapped every step and
  // blocked sleeping). Quantized depth + id tie-break pins contacts to fixed voxels.
  [mutating] void add(daxa_f32vec3 p, daxa_f32 pe, daxa_f32vec3 n, daxa_u32 fa, daxa_u32 fb,
                      daxa_f32vec3 center, daxa_f32 quant)
  {
    daxa_f32vec3 d = p - center;
    daxa_u32 oct = (d.x > 0.0f ? 1u : 0u) | (d.y > 0.0f ? 2u : 0u) | (d.z > 0.0f ? 4u : 0u);
    bool empty = pen[oct] > 0.5e9f;
    daxa_i32 nb = daxa_i32(floor(pe / quant));
    daxa_i32 ob = empty ? 0 : daxa_i32(floor(pen[oct] / quant));
    if (empty || nb < ob || (nb == ob && fa < fp_a[oct]))
    {
      pos[oct] = p; pen[oct] = pe; nrm[oct] = n; fp_a[oct] = fa; fp_b[oct] = fb;
    }
    daxa_f32 w = max(-pe, 0.1f * quant); // band samples (pe >~ 0) still steer the normal
    normal_sum += n * w;
    weight_sum += w;
    ++sample_count;
  }
};

// voxel body r1 (shape s1) sampled against: voxel body r2 (shape s2) when s2_is_voxel,
// otherwise r2's OBB. Returns contact count; fills the reducer.
// mode 0 = accept all samples; 1 = only those with dot(n, ref_n) >= 0; 2 = only dot < 0
// (modes 1/2 split opposing-normal contact sets - e.g. both walls of a hole - into two
// manifolds, because averaging opposing normals cancels to a degenerate direction)
daxa_u32 voxel_sample_body(RigidBody* r1, VoxelShape* s1, RigidBody* r2, VoxelShape* s2, bool s2_is_voxel,
                           daxa_u32* occ, daxa_u32* surf, inout VoxelManifoldReducer red,
                           daxa_f32vec3 ref_n, daxa_u32 mode)
{
  daxa_f32 vs1 = s1->voxel_size;
  daxa_u32 found = 0u;
  // per-axis support radius of the (rotated) sample voxel in body2's frame: a flat
  // 0.5*vs1 underestimates the overlap of rotated bodies by up to sqrt(3), which showed
  // as visible standing interpenetration at rotated piece-piece contacts
  daxa_f32vec3 a0 = r2->rotate_vector_inverse(r1->rotate_vector(daxa_f32vec3(1.0f, 0.0f, 0.0f)));
  daxa_f32vec3 a1 = r2->rotate_vector_inverse(r1->rotate_vector(daxa_f32vec3(0.0f, 1.0f, 0.0f)));
  daxa_f32vec3 a2 = r2->rotate_vector_inverse(r1->rotate_vector(daxa_f32vec3(0.0f, 0.0f, 1.0f)));
  daxa_f32vec3 sup = 0.5f * vs1 * daxa_f32vec3(abs(a0.x) + abs(a1.x) + abs(a2.x),
                                               abs(a0.y) + abs(a1.y) + abs(a2.y),
                                               abs(a0.z) + abs(a1.z) + abs(a2.z));
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
    if (s2_is_voxel)
    {
      daxa_f32 vs2 = s2->voxel_size;
      daxa_f32vec3 l2 = r2->rotate_vector_inverse(pw - r2->position);
      daxa_f32vec3 g = (l2 - s2->grid_origin) / vs2;
      daxa_i32vec3 c = daxa_i32vec3(daxa_i32(floor(g.x)), daxa_i32(floor(g.y)), daxa_i32(floor(g.z)));
      if (!voxel_occ_bit(occ, s2, c)) { continue; }
      // CELL-FACE contact (axis-aligned mini-SAT in B's grid): among the axes whose
      // neighbor cell is EMPTY (outward-feasible directions), take the minimum overlap;
      // its sign gives a crisp face normal and an exact depth. The previous occupancy
      // GRADIENT gave diagonal normals at face edges/corners ((1,1,1)/sqrt(3) etc.) -
      // and the per-octant winners ARE those edge/corner cells - so every face contact
      // was tilted and depth-underestimated: mushy contacts that slowly sank under
      // gravity (bodies half-buried in the floor / clumped into each other, then slept).
      daxa_f32vec3 cc_l = s2->grid_origin + (daxa_f32vec3(daxa_f32(c.x), daxa_f32(c.y), daxa_f32(c.z)) + daxa_f32vec3(0.5f, 0.5f, 0.5f)) * vs2;
      daxa_f32vec3 dl = l2 - cc_l;

      daxa_f32 best = 1e9f;
      daxa_i32 best_axis = -1;
      daxa_f32 best_sign = 1.0f;
      [unroll]
      for (daxa_u32 ax = 0u; ax < 3u; ++ax)
      {
        daxa_f32 di = ax == 0u ? dl.x : (ax == 1u ? dl.y : dl.z);
        daxa_f32 sgn = di >= 0.0f ? 1.0f : -1.0f;
        daxa_i32vec3 nb = c;
        if (ax == 0u) { nb.x += daxa_i32(sgn); }
        else if (ax == 1u) { nb.y += daxa_i32(sgn); }
        else { nb.z += daxa_i32(sgn); }
        if (voxel_occ_bit(occ, s2, nb)) { continue; } // that way is more solid: not an exit
        daxa_f32 sup_ax = ax == 0u ? sup.x : (ax == 1u ? sup.y : sup.z);
        daxa_f32 ov = (0.5f * vs2 + sup_ax) - abs(di);
        // deterministic tie band: TILTED contacts have two near-equal minimal axes and a
        // raw argmin flips the normal 90 deg between steps (the "dancing" leaning pieces);
        // a later axis must beat the current pick by a clear margin to displace it
        if (ov < best - 0.05f * vs2) { best = ov; best_axis = daxa_i32(ax); best_sign = sgn; }
      }
      if (best_axis < 0)
      {
        // surrounded on all axes: deep interior, no reliable direction
        red.interior_hits += 1u;
        continue;
      }
      daxa_f32vec3 n_local = daxa_f32vec3(best_axis == 0 ? best_sign : 0.0f,
                                          best_axis == 1 ? best_sign : 0.0f,
                                          best_axis == 2 ? best_sign : 0.0f);
      n_out2 = r2->rotate_vector(n_local);
      pe = -best;
      fb = daxa_u32(c.x & 0xFF) | (daxa_u32(c.y & 0xFF) << 8u) | (daxa_u32(c.z & 0xFF) << 16u);
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
      // exit along the least-penetrated face (per-axis rotated support, as above)
      daxa_f32vec3 qs = q - sup;
      // same deterministic tie band as the voxel-voxel mini-SAT (edge-resting pieces
      // have two near-equal exit faces; raw argmax flips the normal between steps)
      daxa_f32 tie = 0.05f * vs1;
      daxa_u32 k = (qs.x > qs.y + tie && qs.x > qs.z + tie) ? 0u : (qs.y > qs.z + tie ? 1u : 2u);
      daxa_f32 lk = k == 0u ? l2.x : (k == 1u ? l2.y : l2.z);
      daxa_f32 qk = k == 0u ? qs.x : (k == 1u ? qs.y : qs.z);
      daxa_f32vec3 axis = daxa_f32vec3(k == 0u ? 1.0f : 0.0f, k == 1u ? 1.0f : 0.0f, k == 2u ? 1.0f : 0.0f);
      n_out2 = r2->rotate_vector(axis * (lk >= 0.0f ? 1.0f : -1.0f));
      pe = qk;
      fb = k;
    }
    if (pe >= 0.02f) { continue; } // margin band: resting contacts (pen ~ -0.0005) must
                                   // not flicker across the cutoff and lose their lambdas
    if (mode == 1u && dot(n_out2, ref_n) < 0.0f) { continue; }
    if (mode == 2u && dot(n_out2, ref_n) >= 0.0f) { continue; }
    // depth caps: half a voxel between voxel bodies (pen feeds AVBD's positional sweep -
    // full-voxel corrections pop), but a FULL voxel against statics/OBBs: fast spawns
    // tunnel into the floor and a half-voxel extraction rate loses to gravity, leaving
    // bodies half-buried (where the sleep test then froze them)
    pe = max(pe, s2_is_voxel ? -0.5f * vs1 : -vs1);
    red.add(pw, pe, n_out2, e & 0xFFFFFFu, fb, r1->position, 0.5f * vs1);
    ++found;
  }
  return found;
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

// full voxel narrow phase for a candidate pair; emits the standard Manifold.
// emit one reduced cluster as a standard Manifold (normal from the cluster's own
// |pen|-weighted sample normals; key identifies the cluster for warm-start matching)
daxa_b32 voxel_emit_manifold(VoxelManifoldReducer red, bool flip, daxa_i32 key, out Manifold manifold)
{
  manifold.contact_count = 0;
  manifold.key = key;
  manifold.error = 0;
  if (red.sample_count == 0u || dot(red.normal_sum, red.normal_sum) < 1e-10f)
  {
    return false;
  }
  daxa_f32vec3 n_sep = normalize(red.normal_sum); // separation direction for the SAMPLED body
  manifold.normal = flip ? n_sep : -n_sep;
  daxa_u32 out_n = 0u;
  [unroll]
  for (daxa_u32 o = 0u; o < 8u; ++o)
  {
    if (red.pen[o] >= 0.02f) { continue; } // band contacts emit; empty slots (1e9) drop
    // depths were measured along each sample's own normal but get RESOLVED along the
    // cluster normal: project them (and drop near-orthogonal stragglers, which would
    // otherwise be squeezed sideways by the full unprojected depth)
    daxa_f32 align = dot(red.nrm[o], n_sep);
    if (align <= 0.1f) { continue; }
    manifold.contacts[out_n].position = red.pos[o];
    manifold.contacts[out_n].penetration = red.pen[o] * align;
    manifold.contacts[out_n].normal_impulse = 0.0f;
    manifold.contacts[out_n].tangent_impulse[0] = 0.0f;
    manifold.contacts[out_n].tangent_impulse[1] = 0.0f;
    manifold.contacts[out_n].warm_start = 0u;
    manifold.contacts[out_n].stick = 0u;
    // warm-start matching keys on the SAMPLED body's voxel id only (B-cell ids churn on
    // micro-motion); the octant winner is depth-sticky, so the A id is stable
    manifold.contacts[out_n].fp.in_reference = flip ? 1u : 0u;
    manifold.contacts[out_n].fp.out_reference = red.fp_b[o];
    manifold.contacts[out_n].fp.in_incident = red.fp_a[o];
    manifold.contacts[out_n].fp.out_incident = red.fp_a[o];
    ++out_n;
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
daxa_b32 voxel_collision_detection(SimConfig* sim_config, RigidBody* rA, RigidBody* rB,
                                   out Manifold manifold, out Manifold manifold2, out daxa_b32 has2)
{
  manifold.contact_count = 0;
  manifold.key = 99;
  manifold.error = 0;
  manifold2.contact_count = 0;
  manifold2.key = 98;
  manifold2.error = 0;
  has2 = false;

  if (!voxel_obb_prefilter(rA, rB))
  {
    return false;
  }

  VoxelShape* shapes = (VoxelShape*)(sim_config->voxel_shapes_addr);
  daxa_u32* occ = (daxa_u32*)(sim_config->voxel_occupancy_addr);
  daxa_u32* surf = (daxa_u32*)(sim_config->voxel_surface_addr);

  bool a_vox = rA->shape_index != 0u;
  bool b_vox = rB->shape_index != 0u;
  VoxelShape* sA = &shapes[a_vox ? rA->shape_index - 1u : 0u];
  VoxelShape* sB = &shapes[b_vox ? rB->shape_index - 1u : 0u];

  // sample from the voxel body with the smaller surface list (both directions would be
  // more complete; one-sided is the prototype trade-off). Counts hoisted to locals: the
  // Slang parser chokes on chained -> derefs inside && expressions (E30101).
  daxa_u32 sa_n = sA->surf_count;
  daxa_u32 sb_n = sB->surf_count;
  bool flip = !a_vox || (b_vox && sb_n < sa_n); // sampled body = obb2
  RigidBody* r1 = flip ? rB : rA;
  VoxelShape* s1 = flip ? sB : sA;
  RigidBody* r2 = flip ? rA : rB;
  VoxelShape* s2 = flip ? sA : sB;
  bool s2_is_voxel = flip ? a_vox : b_vox;

  VoxelManifoldReducer red;
  red.init();
  voxel_sample_body(r1, s1, r2, s2, s2_is_voxel, occ, surf, red, daxa_f32vec3(0.0f, 1.0f, 0.0f), 0u);
  if (red.sample_count == 0u)
  {
    if (red.interior_hits > 0u)
    {
      // deep-interpenetration rescue: every sample fell in interior cells (no surface
      // signal anywhere) - emit a single center-to-center contact so the bodies separate
      // along a sane direction instead of sinking further
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
    return false;
  }

  // ALWAYS split by the stable center-to-center reference - no coherence threshold.
  // The old "one manifold when coherent" test sat near its threshold for resting pieces
  // (mostly-up normals + some lateral) and FLAPPED the pair between 1- and 2-manifold
  // structures every few frames, rebuilding lambdas each time (the visible standing
  // jitter; manifold count oscillated 8-11 at rest). With an unconditional split the
  // opposing cluster is simply empty for coherent contact sets (has2 = false), and the
  // structure is deterministic frame to frame. The deepest sample's
  // normal flip-flopped between steps, swapping which wall was "cluster 99" - and since
  // warm-start persists lambda BY KEY, the persisted lambda then acted along the flipped
  // normal and ATTRACTED the bodies (the observed collapse-into-one-point). The
  // center-to-center direction never flips for a resting pair.
  // cluster reference = the STABLE deepest sample's normal (quantized-depth bucket +
  // lowest-voxel-id winner - the same persistent identity the reducer uses, so it does
  // not flip-flop). The previous center-to-center reference was stable but geometrically
  // wrong as a discriminant: for side-by-side pieces the contact normals are
  // PERPENDICULAR to the center line, so every sample sat exactly ON the cluster
  // boundary and micro-rotations flipped it between manifolds 99/98, resetting its
  // lambdas each time - the accumulator-visible piece-piece jitter (floor contacts,
  // whose normals align with the center line, stayed sharp).
  daxa_f32vec3 ref_n = daxa_f32vec3(0.0f, 1.0f, 0.0f);
  daxa_f32 ref_quant = 0.5f * s1->voxel_size;
  daxa_f32 best_qb = 1.0e9f;
  daxa_u32 best_id = 0xFFFFFFFFu;
  [unroll]
  for (daxa_u32 o = 0u; o < 8u; ++o)
  {
    if (red.pen[o] > 0.5e9f) { continue; } // empty slot
    daxa_f32 qb = floor(red.pen[o] / ref_quant);
    if (qb < best_qb || (qb == best_qb && red.fp_a[o] < best_id))
    {
      best_qb = qb;
      best_id = red.fp_a[o];
      ref_n = red.nrm[o];
    }
  }
  VoxelManifoldReducer red1;
  red1.init();
  voxel_sample_body(r1, s1, r2, s2, s2_is_voxel, occ, surf, red1, ref_n, 1u);
  VoxelManifoldReducer red2;
  red2.init();
  voxel_sample_body(r1, s1, r2, s2, s2_is_voxel, occ, surf, red2, ref_n, 2u);
  daxa_b32 ok1 = voxel_emit_manifold(red1, flip, 99, manifold);
  has2 = voxel_emit_manifold(red2, flip, 98, manifold2);
  if (!ok1 && has2)
  {
    manifold = manifold2;
    has2 = false;
    ok1 = true;
  }
  return ok1;
}

func narrow_phase(BroadPhaseCollision broad_phase_collision, RigidBodyEntry* rigid_boy_map, RigidBody* rigid_bodies, SimConfig* sim_config, SimConfig previous_sim_config, ManifoldNode* manifold_nodes, CollisionEntry* collision_map, Manifold* collisions, RigidBodyEntry* previous_rigid_body_map, RigidBody* previous_rigid_bodies, ManifoldNode* previous_manifold_nodes, CollisionEntry* collision_map_prev, Manifold* old_collisions, BodyLink* body_links) -> void
{
  daxa::u32 rba_index = broad_phase_collision.body_a_index;
  daxa::u32 rbb_index = broad_phase_collision.body_b_index;

  RigidBody* r = &rigid_bodies[rba_index];
  RigidBody* r2 = &rigid_bodies[rbb_index];

  IslandBuilder island_builder = IslandBuilder(sim_config.active_rigid_body_count, body_links);

  Manifold manifold;
  Manifold manifold2;
  daxa_b32 has2 = false;

  var obb1_faces = 0u;
  var obb2_faces = 0u;

  // voxel bodies use the occupancy-bitmask narrow phase; OBB pairs keep the SAT path.
  // Voxel pairs may emit TWO manifolds (opposing-normal clusters of interlocked shapes).
  var collision = false;
  if (r->shape_index != 0u || r2->shape_index != 0u)
  {
    collision = voxel_collision_detection(sim_config, r, r2, manifold, manifold2, has2);
  }
  else
  {
    collision = OBBvsOBBCollision.collision_detection(r, r2, manifold, obb1_faces, obb2_faces);
  }
  
  daxa_u32 emit_count = collision ? (has2 ? 2u : 1u) : 0u;
  for (daxa_u32 emit_i = 0u; emit_i < emit_count; ++emit_i)
  {
  if (emit_i == 1u)
  {
    manifold = manifold2;
  }
  manifold.obb1_index = rba_index;
  manifold.obb2_index = rbb_index;
  manifold.anchor_a = r->position;
  manifold.anchor_b = r2->position;
#if defined(BB_DEBUG)
  manifold.frame_count = sim_config.frame_count;
#endif // BB_DEBUG

  {
    daxa_u32 index = 0;
    InterlockedAdd(sim_config->g_c_info.collision_count, 1u, index);
    if (index >= BB_MAX_COLLISION_COUNT)
    {
      return;
    }
    manifold.compute_basis();

    // Add collision to collision map
    collision_map[index].index = index;

    BodyLinkManifold::body_link_manifold(index, rba_index, &sim_config->manifold_node_count, rigid_bodies, manifold_nodes);

    BodyLinkManifold::body_link_manifold(index, rbb_index, &sim_config->manifold_node_count, rigid_bodies, manifold_nodes);

    daxa::u32 rba_prev_index = previous_rigid_body_map[r->id].index;
    bool rba_prev_exists = rba_prev_index < previous_sim_config.rigid_body_count;
    daxa::u32 rbb_prev_index = previous_rigid_body_map[r2->id].index;
    bool rbb_prev_exists = rbb_prev_index < previous_sim_config.rigid_body_count;

    daxa_b32 warm_starting = sim_config->has_flag(SimFlag::WARM_STARTING);
    if(warm_starting && rba_prev_exists && rbb_prev_exists) {
      daxa_u32 old_index = 0;
      RigidBody* prev_rba = &previous_rigid_bodies[rba_prev_index];
      daxa_u32 previous_manifold_node_count = min(previous_sim_config.manifold_node_count, BB_MAX_MANIFOLD_NODE_COUNT);
      daxa_u32 previous_collision_count = min(previous_sim_config.g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
      
#if defined(BB_SIM_DEBUG)
      if(BodyLinkManifold::find_old_collision(rba_prev_index, rbb_prev_index, manifold.key, prev_rba->manifold_node_index, previous_manifold_node_count, previous_collision_count, collision_map_prev, previous_manifold_nodes, old_collisions, old_index))
      {
#else
      if(BodyLinkManifold::find_old_collision(rba_prev_index, rbb_prev_index, manifold.key, prev_rba->manifold_node_index, previous_manifold_node_count, previous_collision_count, collision_map_prev, previous_manifold_nodes, old_collisions, old_index)) {
#endif
        manifold.update_from_old(&old_collisions[old_index]);
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
    if (emit_i == 0u)
    {
      island_builder.link_bodies(in_first, in_second);
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
