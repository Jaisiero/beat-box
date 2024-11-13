#include "extensions.slang"

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

    // TODO: check if this is correct
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
    return v > 0.0f;
  }

  static daxa_b32 behind(daxa_f32 v) {
    return v < 0.0f;
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

      daxa_f32 da = e - sign * a.v[axis];
      daxa_f32 db = e - sign * b.v[axis];

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
        cv.f.out_incident = 0;
        out[out_count++] = cv;
      } 
      else if(behind(da) && in_front(db)) 
      {
        cv.f = a.f;
        cv.v = a.v + (b.v - a.v) * (da / (da - db));
        cv.f.in_reference = clip_edge;
        cv.f.in_incident = 0;
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
    }

    out_count = ortographic_clip(1.0f, e.x, 0, clip_edges[0], in, in_count, out);

    if(out_count == 0) {
      // DEBUGGING
      error = 10;
      // // ClipVertex a = in[in_count - 1];
      // for(daxa_i32 i = 0; i < MAX_INCIDENT_VERTEX_COUNT; ++i) {
      //   ClipVertex b = in[i];
      //   daxa_f32 da = e.x - 1.0 * b.v[0];
      //   daxa_f32 db = e.y - 1.0 * b.v[1];
      //   out_vertices[i] = in[i];
      //   depths[i] = da;
      //   depths[i+MAX_INCIDENT_VERTEX_COUNT] = db;
      // }
      return 0;
    }

    in_count = ortographic_clip(1.0f, e.y, 1, clip_edges[1], out, out_count, in);

    if(in_count == 0) {
      // DEBUGGING
      error = 11; 
      // // ClipVertex a = in[in_count - 1];
      // for(daxa_i32 i = 0; i < MAX_INCIDENT_VERTEX_COUNT; ++i) {
      //   ClipVertex b = in[i];
      //   daxa_f32 da = e.x - 1.0 * b.v[0];
      //   daxa_f32 db = e.y - 1.0 * b.v[1];
      //   out_vertices[i] = in[i];
      //   depths[i] = da;
      //   depths[i+MAX_INCIDENT_VERTEX_COUNT] = db;
      // }
      return 0;
    }

    out_count = ortographic_clip(-1.0f, e.x, 0, clip_edges[2], in, in_count, out);

    if(out_count == 0) {
      // DEBUGGING
      error = 12;
      return 0;
    }

    in_count = ortographic_clip(-1.0f, e.y, 1, clip_edges[3], out, out_count, in);

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


  static void support_edge(const Transform tx, const daxa_f32vec3 e, daxa_f32vec3 n, out daxa_f32vec3 CA, out daxa_f32vec3 CB) {
    n = tx.rotate_transpose(n);
    daxa_f32vec3 abs_n = abs(n);
    daxa_f32vec3 a, b;

	  // x > y
    if(abs_n.x > abs_n.y) {
		  // x > y > z
      if(abs_n.y > abs_n.z) {
        a = daxa_f32vec3(e.x, e.y, e.z);
        b = daxa_f32vec3(e.x, e.y, -e.z);
		  // x > z > y || z > x > y
      } else {
        a = daxa_f32vec3(e.x, e.y, e.z);
        b = daxa_f32vec3(e.x, -e.y, e.z);
      }
	  // y > x
    } else  {
		  // y > x > z
      if(abs_n.x > abs_n.z) {
        a = daxa_f32vec3(e.x, e.y, e.z);
        b = daxa_f32vec3(e.x, e.y, -e.z);
      } else {
        a = daxa_f32vec3(e.x, e.y, e.z);
        b = daxa_f32vec3(-e.x, e.y, e.z);
      }
		// z > y > x || y > z > x
    }

    daxa_f32 sign_x = sign(n.x);
    daxa_f32 sign_y = sign(n.y);
    daxa_f32 sign_z = sign(n.z);

    a.x *= sign_x;
    a.y *= sign_y;
    a.z *= sign_z;

    b.x *= sign_x;
    b.y *= sign_y;
    b.z *= sign_z;

    CA = tx.transform(a);
    CB = tx.transform(b);
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

    // If denom is zero, then the line segments are parallel
    if (denom == 0.0f) {
      CA = PA;
      CB = PB;
      return;
    }

    daxa_f32 TA = (b * f - c * e) / denom;
    daxa_f32 TB = (b * TA + f) / e;

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

  static daxa_b32 collision_detection(const Ptr<RigidBody> A, const Ptr<RigidBody> B, inout Manifold manifold, out daxa_u32 obb1_faces, out daxa_u32 obb2_faces) {
    
    obb1_faces = 0;
    obb2_faces = 0;

    // Transform the OBBs to world space
    Transform tx_A = Transform(A->get_rotation_matrix(), A->position);
    Transform tx_B = Transform(B->get_rotation_matrix(), B->position);

    // Get the half extents of the OBBs
    daxa_f32vec3 e_A = (A->maximum - A->minimum) * 0.5f;
    daxa_f32vec3 e_B = (B->maximum - B->minimum) * 0.5f;


    Transform tx_C;
    Transform tx_abs_C;

    daxa_f32mat3x3 C = mul(tx_A.get_inverse_rotation_matrix(), tx_B.get_rotation_matrix());
    daxa_f32mat3x3 abs_C = abs(C);

    daxa_b32 parallel = false;
    {
      // r2 to r1
      for (daxa_u32 i = 0; i < 3; i++) {
        for (daxa_u32 j = 0; j < 3; j++) {
          if (abs_C[i][j] > 0.999f) {
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
            return false; // Separating axis found
        }
    }

    // Iterate through OBB2's axes
    for(int i = 0; i < 3; ++i) {
        // Compute the projection using dot product
        // dot(t, C[i]) = t.x * C[0][i] + t.y * C[1][i] + t.z * C[2][i]
        float projection = dot(t, tx_C.get_axis(i));
        // Compute the separation distance
        float s = abs(projection) - (e_B[i] + dot(tx_abs_C.get_axis(i), e_A));

        if(track_face_axis(b_axis, 
                           OBB2_Axes[i].index, 
                           s, 
                           b_max, 
                           OBB2_Axes[i].normal, 
                           n_b)) {
            return false; // Separating axis found
        }
    }

    daxa_f32 e_max = -FLT_MAX;
    daxa_i32 e_axis = -1;
    daxa_f32vec3 n_e = daxa_f32vec3(0.0f);

    // Edge cross product axes
    if(!parallel) {
      
      // Edge axis checks
      daxa_f32 rA;
      daxa_f32 rB;

      // Cross (a.x, b.x)
      rA = e_A.y * abs_C[0][2] + e_A.z * abs_C[0][1];
      rB = e_B.y * abs_C[2][0] + e_B.z * abs_C[1][0];

      daxa_f32 s = abs(t.z * C[0][1] - t.y * C[0][2]) - (rA + rB);
      if(track_edge_axis(e_axis, 
                         A_X_B_X_EDGE, 
                         s, 
                         e_max, 
                         daxa_f32vec3(0, -C[0][2], C[0][1]), 
                         n_e)) {
          return false; // Separating axis found
      }

      // Cross (a.x, b.y)
      rA = e_A.y * abs_C[1][2] + e_A.z * abs_C[1][1];
      rB = e_B.x * abs_C[2][0] + e_B.z * abs_C[0][0];

      s = abs(t.z * C[1][1] - t.y * C[1][2]) - (rA + rB);
      if(track_edge_axis(e_axis, 
                         A_X_B_Y_EDGE, 
                         s, 
                         e_max, 
                         daxa_f32vec3(0, -C[1][2], C[1][1]), 
                         n_e)) {
          return false; // Separating axis found
      }

      // Cross (a.x, b.z)
      rA = e_A.y * abs_C[2][2] + e_A.z * abs_C[2][1];
      rB = e_B.x * abs_C[1][0] + e_B.y * abs_C[0][0];

      s = abs(t.z * C[2][1] - t.y * C[2][2]) - (rA + rB);
      if(track_edge_axis(e_axis, 
                         A_X_B_Z_EDGE, 
                         s, 
                         e_max, 
                         daxa_f32vec3(0, -C[2][2], C[2][1]), 
                         n_e)) {
          return false; // Separating axis found
      }

      // Cross (a.y, b.x)
      rA = e_A.x * abs_C[0][2] + e_A.z * abs_C[0][0];
      rB = e_B.y * abs_C[2][1] + e_B.z * abs_C[1][1];

      s = abs(t.x * C[0][2] - t.z * C[0][0]) - (rA + rB);
      if(track_edge_axis(e_axis, 
                         A_Y_B_X_EDGE, 
                         s, 
                         e_max, 
                         daxa_f32vec3(C[0][2], 0, -C[0][0]), 
                         n_e)) {
          return false; // Separating axis found
      }

      // Cross (a.y, b.y)
      rA = e_A.x * abs_C[1][2] + e_A.z * abs_C[1][0];
      rB = e_B.x * abs_C[2][1] + e_B.z * abs_C[0][1];

      s = abs(t.x * C[1][2] - t.z * C[1][0]) - (rA + rB);
      if(track_edge_axis(e_axis, 
                         A_Y_B_Y_EDGE, 
                         s, 
                         e_max, 
                         daxa_f32vec3(C[1][2], 0, -C[1][0]), 
                         n_e)) {
          return false; // Separating axis found
      }

      // Cross (a.y, b.z)
      rA = e_A.x * abs_C[2][2] + e_A.z * abs_C[2][0];
      rB = e_B.x * abs_C[1][1] + e_B.y * abs_C[0][1];

      s = abs(t.x * C[2][2] - t.z * C[2][0]) - (rA + rB);
      if(track_edge_axis(e_axis, 
                         A_Y_B_Z_EDGE, 
                         s, 
                         e_max, 
                         daxa_f32vec3(C[2][2], 0, -C[2][0]), 
                         n_e)) {
          return false; // Separating axis found
      }

      // Cross (a.z, b.x)
      rA = e_A.x * abs_C[0][1] + e_A.y * abs_C[0][0];
      rB = e_B.y * abs_C[2][2] + e_B.z * abs_C[1][2];

      s = abs(t.y * C[0][0] - t.x * C[0][1]) - (rA + rB);
      if(track_edge_axis(e_axis, 
                         A_Z_B_X_EDGE, 
                         s, 
                         e_max, 
                         daxa_f32vec3(-C[0][1], C[0][0], 0), 
                         n_e)) {
          return false; // Separating axis found
      }

      // Cross (a.z, b.y)
      rA = e_A.x * abs_C[1][1] + e_A.y * abs_C[1][0];
      rB = e_B.x * abs_C[2][2] + e_B.z * abs_C[0][2];

      s = abs(t.y * C[1][0] - t.x * C[1][1]) - (rA + rB);
      if(track_edge_axis(e_axis, 
                         A_Z_B_Y_EDGE, 
                         s, 
                         e_max, 
                         daxa_f32vec3(-C[1][1], C[1][0], 0), 
                         n_e)) {
          return false; // Separating axis found
      }

      // Cross (a.z, b.z)
      rA = e_A.x * abs_C[2][1] + e_A.y * abs_C[2][0];
      rB = e_B.x * abs_C[1][2] + e_B.y * abs_C[0][2];

      s = abs(t.y * C[2][0] - t.x * C[2][1]) - (rA + rB);
      if(track_edge_axis(e_axis, 
                         A_Z_B_Z_EDGE, 
                         s, 
                         e_max, 
                         daxa_f32vec3(-C[2][1], C[2][0], 0), 
                         n_e)) {
          return false; // Separating axis found
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
        return true; // FAIL
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



func broad_phase(const daxa_u32 rigid_body_index, Ptr<RigidBody> r,
         Ptr<SimConfig> sim_config, SimConfig previous_sim_config, Ptr<Manifold> collisions,
         Ptr<Manifold> old_collisions) -> void
{
#if defined(BB_DEBUG)
  r->clear_flag(RigidBodyFlag::COLLIDING);
#endif // BB_DEBUG
  for (daxa_u32 i = rigid_body_index + 1; i < sim_config->rigid_body_count; i++)
  {
    // Simplex simplex;
    Ptr<RigidBody> r2 = Ptr<RigidBody>(&BP.task_head.rigid_bodies[i]);

    Manifold manifold;
    manifold.obb1_index = rigid_body_index;
    manifold.obb2_index = i;
    manifold.contact_count = 0;
    manifold.key = -1;

    var obb1_faces = 0u;
    var obb2_faces = 0u;

    let collision = OBBvsOBBCollision.collision_detection(r, r2, manifold, obb1_faces, obb2_faces);

    // 
    if (collision)
    {
      daxa_u32 index = 0;
      InterlockedAdd(sim_config->g_c_info.collision_count, 1u, index);
      manifold.compute_basis();

      daxa_b32 warm_starting = sim_config->has_flag(SimFlag::WARM_STARTING);
      if(warm_starting) {
        Manifold old_manifold;
        if(find_old_collision(rigid_body_index, i, previous_sim_config.g_c_info.collision_count, old_collisions, old_manifold))
        {
          // printf("Found old collision %d %d\n", rigid_body_index, i);
          manifold.update(old_manifold);
        }
      }

      collisions[index] = manifold;
#if defined(BB_DEBUG)
      var old_obb1_faces = 0u;
      var old_obb2_faces = 0u;
      if(obb1_faces != 0) {
        InterlockedOr(r->face_collided, obb1_faces, old_obb1_faces);
        r->flags |= RigidBodyFlag::COLLIDING;
      }
      if(obb2_faces != 0) {
        InterlockedOr(r2->face_collided, obb2_faces, old_obb2_faces);
        r2->flags |= RigidBodyFlag::COLLIDING;
      }
#endif // BB_DEBUG
    }
  }
}