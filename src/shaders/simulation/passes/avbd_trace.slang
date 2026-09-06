#include "avbd_common.slang"

void avbd_trace_contact(daxa_u32vec3 compute_i)
{
  SimConfig* sim_config = (SimConfig*)(AVBD_PC.task_head.sim_config);
  if (sim_config->solver_type != SimSolverType::AVBD)
  {
    return;
  }
  daxa_u32 m_idx = compute_i.x;
  daxa_u32 coll = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (m_idx >= coll)
  {
    return;
  }
  Manifold* m = &((Manifold*)(AVBD_PC.task_head.collisions))[m_idx];
  daxa_u32 b1 = m->obb1_index;
  daxa_u32 b2 = m->obb2_index;
  if (b1 >= sim_config->rigid_body_count || b2 >= sim_config->rigid_body_count || b1 == b2)
  {
    return;
  }
  RigidBody* rigid_bodies = (RigidBody*)(AVBD_PC.task_head.rigid_bodies);
  RigidBody* r1 = &rigid_bodies[b1];
  RigidBody* r2 = &rigid_bodies[b2];
  bool dyn1 = avbd_is_dynamic(r1) && (r1->flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE;
  bool dyn2 = avbd_is_dynamic(r2) && (r2->flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE;
  if (!dyn1 && !dyn2)
  {
    return; // only awake pairs; the oscillator is awake by definition
  }
  daxa_f32vec3 n = m->normal;
  // peak angular rate over contacting awake bodies (the user's symptom is ROTATIONAL jitter):
  // captured globally, not just at the deepest contact, since the rocking body may be shallow
  daxa_u32 w1 = dyn1 ? daxa_u32(length(r1->omega) * 1000.0f) : 0u;
  daxa_u32 w2 = dyn2 ? daxa_u32(length(r2->omega) * 1000.0f) : 0u;
  daxa_u32 prev_w;
  InterlockedMax(sim_config->dbg_pk_omega, max(w1, w2), prev_w);
  daxa_u32 cc = min((daxa_u32)max(m->contact_count, 0), MAX_CONTACT_POINT_COUNT);
  for (daxa_u32 j = 0u; j < cc; ++j)
  {
    daxa_f32 pen = m->contacts[j].penetration; // <0 when penetrating (SAT convention)
    daxa_u32 pen_mm = daxa_u32(clamp(-pen * 1000.0f, 0.0f, 1.0e9f));
    daxa_u32 prev;
    InterlockedMax(sim_config->dbg_pk_pen, pen_mm, prev);
    // The trace entry runs this reduction on one invocation, so the maximum and
    // its payload always describe the same contact.
    if (pen_mm > prev) // this contact is (currently) the deepest; latch its full state
    {
      daxa_f32vec3 p = m->contacts[j].position;
      daxa_f32vec3 ra = p - r1->position;
      daxa_f32vec3 rb = p - r2->position;
      daxa_f32vec3 vrel = (r2->velocity + cross(r2->omega, rb)) - (r1->velocity + cross(r1->omega, ra));
      sim_config->dbg_pk_vn = dot(n, vrel);
      sim_config->dbg_pk_lambda = m->contacts[j].normal_impulse;
      sim_config->dbg_pk_k = m->contacts[j].mass_coefficient;
      sim_config->dbg_pk_body = (b1 << 16) | (b2 & 0xFFFFu);
      sim_config->dbg_pk_stick = (cc << 1) | (m->contacts[j].stick & 1u);
    }
  }
}

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_pocket_trace(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  if (compute_i.x != 0u) return;
  SimConfig* sc = (SimConfig*)(AVBD_PC.task_head.sim_config);
  daxa_u32 count = min(sc->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  for (daxa_u32 i = 0u; i < count; ++i) avbd_trace_contact(daxa_u32vec3(i, 0u, 0u));
}
