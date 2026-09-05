#include "avbd_constraints.slang"

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_warmstart(daxa_u32vec3 compute_i : SV_DispatchThreadID)
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
  AvbdBodyState* states = (AvbdBodyState*)(AVBD_PC.task_head.avbd_state);
  RigidBody* r1 = &rigid_bodies[b1];
  RigidBody* r2 = &rigid_bodies[b2];
  bool dyn1 = avbd_is_dynamic(r1) && (r1->flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE;
  bool dyn2 = avbd_is_dynamic(r2) && (r2->flags & RigidBodyFlag::SLEEPING) == RigidBodyFlag::NONE;
  // step-start poses for the anchor local frames (prepare already jumped the bodies to the
  // warmstarted pose, so dynamics read pos/rot_start from avbd_state; statics never move)
  daxa_f32vec3 c1 = dyn1 ? states[b1].pos_start : r1->position;
  daxa_f32vec3 c2 = dyn2 ? states[b2].pos_start : r2->position;
  Quaternion q1c = dyn1 ? Quaternion(-states[b1].rot_start.v, states[b1].rot_start.w) : Quaternion(-r1->rotation.v, r1->rotation.w);
  Quaternion q2c = dyn2 ? Quaternion(-states[b2].rot_start.v, states[b2].rot_start.w) : Quaternion(-r2->rotation.v, r2->rotation.w);

  daxa_u32 cc = min((daxa_u32)max(m->contact_count, 0), MAX_CONTACT_POINT_COUNT);
  for (daxa_u32 j = 0u; j < cc; ++j)
  {
    bool matched = m->contacts[j].warm_start != 0u && sim_config->has_flag(SimFlag::WARM_STARTING);
    if (!matched)
    {
      // fresh contact: zero force estimates, minimum penalties
      m->contacts[j].normal_impulse = 0.0f;
      m->contacts[j].mass_coefficient = BB_AVBD_PENALTY_MIN;
      m->contacts[j].tangent_impulse[0] = 0.0f;
      m->contacts[j].tangent_impulse[1] = 0.0f;
      m->contacts[j].tangent_mass[0] = BB_AVBD_PENALTY_MIN;
      m->contacts[j].tangent_mass[1] = BB_AVBD_PENALTY_MIN;
      m->contacts[j].stick = 0u;
    }
    else if (!dyn1 && !dyn2)
    {
      // SLEEPING STASIS (user-found chain-reaction wakes: "a small perturbation turns
      // whole red regions blue in a propagating wave"): the gamma decay below used to
      // run on SLEEPING pairs too, so a contact asleep N frames had its stiffness
      // collapse k *= 0.99^N toward 1 while its lambda stayed loaded. Waking the region
      // resumed the solver on force-without-stiffness contacts: the area SANK under its
      // own weight while the ramp regrew k from 1, the motion churned the clips (fresh,
      // blue) and woke the neighbors - whose k had also collapsed - a propagating
      // softness wave. A sleeping contact is in STASIS: keep k and lambda bit-exact.
      m->contacts[j].normal_impulse = min(m->contacts[j].normal_impulse, 0.0f);
    }
    else
    {
      // post-stabilization mode (reference solver.cpp): keep the FULL lambda from last step
      // (it never saw C0, so it is the steady contact force, not an impact spike) and only
      // decay the penalty so it can shrink when contacts calm down.
      daxa_f32 k = m->contacts[j].mass_coefficient;
      m->contacts[j].mass_coefficient = clamp(k * BB_AVBD_GAMMA, BB_AVBD_PENALTY_MIN, BB_AVBD_PENALTY_MAX);
      // lambda (normal_impulse) kept as is; force convention lambda <= 0
      m->contacts[j].normal_impulse = min(m->contacts[j].normal_impulse, 0.0f);
      // friction rows: same treatment (lambda kept, penalty decayed); tangent lambda is
      // free-signed inside its box bounds
      [unroll]
      for (daxa_u32 t = 0u; t < 2u; ++t)
      {
        daxa_f32 kt = m->contacts[j].tangent_mass[t];
        m->contacts[j].tangent_mass[t] = clamp(kt * BB_AVBD_GAMMA, BB_AVBD_PENALTY_MIN, BB_AVBD_PENALTY_MAX);
      }
    }
    // anchor maintenance (reference initialize()): sticking contacts arrived here with their
    // old anchors (update_from_old reused them); everything else re-anchors at the fresh
    // clipped point, expressed in each body's step-start local frame
    bool sticking = matched && m->contacts[j].stick != 0u;
    if (!sticking)
    {
      daxa_f32vec3 p = m->contacts[j].position;
      m->contacts[j].anchor_a = avbd_q_rotate(q1c, p - c1);
      m->contacts[j].anchor_b = avbd_q_rotate(q2c, p - c2);
    }
    if (sticking)
    {
      daxa_u32 prevd;
      InterlockedAdd(sim_config->avbd_stick_count, 1u, prevd);
    }
  }
}
