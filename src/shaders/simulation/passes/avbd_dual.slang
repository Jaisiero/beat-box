#include "avbd_constraints.slang"

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_dual(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(AVBD_PC.task_head.sim_config);
  if (sim_config->solver_type != SimSolverType::AVBD)
  {
    return;
  }
  // convergence tick: exactly one thread advances the iteration counter and clears the
  // slot the NEXT primal sweep will InterlockedMax into (the dual runs between sweeps,
  // so no residual writer races this clear)
  if (compute_i.x == 0u)
  {
    daxa_u32 t = sim_config->avbd_iter_tick + 1u;
    sim_config->avbd_iter_tick = t;
    sim_config->avbd_step_res[t & 1u] = 0u;
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
  if (!dyn1 && !dyn2)
  {
    return; // fully at rest / static pair
  }

  daxa_u32 cc = min((daxa_u32)max(m->contact_count, 0), MAX_CONTACT_POINT_COUNT);
  daxa_f32 mu = sqrt(max(r1->friction, 0.0f) * max(r2->friction, 0.0f));
  for (daxa_u32 j = 0u; j < cc; ++j)
  {
    daxa_f32vec3 rr1, rr2;
    // dual updates only run during the main sweeps -> stab_alpha = 1 (delta-only constraint)
    daxa_f32 C = avbd_contact_C(m, j, r1, r2, &states[b1], &states[b2], dyn1, dyn2, 1.0f, rr1, rr2);
    daxa_f32 lam = m->contacts[j].normal_impulse;
    daxa_f32 k = m->contacts[j].mass_coefficient;
    // lambda update (reference Eq 11): ASSIGN the clamped force estimate
    daxa_f32 new_lam = min(k * C + lam, 0.0f);
    m->contacts[j].normal_impulse = new_lam;
    // penalty ramp (Eq 16): LINEAR growth, only while the force is not at its clamp bound
    if (new_lam < 0.0f)
    {
      m->contacts[j].mass_coefficient = min(k + BB_AVBD_BETA * abs(C), BB_AVBD_PENALTY_MAX);
    }
    // friction rows: CONE projection (reference manifold.cpp updateDual). Compute both
    // tangent force estimates, then clamp their JOINT 2D magnitude to mu*|lambda_n| (a
    // disc), instead of clamping each tangent to a box independently. Within the cone =
    // static friction (penalty ramps); on the cone = kinetic (force projected to the rim).
    daxa_f32 bound = mu * abs(lam);
    daxa_f32vec3 td0 = m->tangent_vectors[0];
    daxa_f32vec3 td1 = m->tangent_vectors[1];
    daxa_f32vec3 ar1_0, ar2_0, ar1_1, ar2_1;
    daxa_f32 C0t0, C0t1;
    daxa_f32 Ct0 = avbd_dir_C(td0, m, j, r1, r2, &states[b1], &states[b2], dyn1, dyn2, 1.0f, ar1_0, ar2_0, C0t0);
    daxa_f32 Ct1 = avbd_dir_C(td1, m, j, r1, r2, &states[b1], &states[b2], dyn1, dyn2, 1.0f, ar1_1, ar2_1, C0t1);
    daxa_f32 ft0 = m->contacts[j].tangent_mass[0] * Ct0 + m->contacts[j].tangent_impulse[0];
    daxa_f32 ft1 = m->contacts[j].tangent_mass[1] * Ct1 + m->contacts[j].tangent_impulse[1];
    daxa_f32 fmag = sqrt(ft0 * ft0 + ft1 * ft1);
    bool within_cone = fmag <= bound;
    if (!within_cone && fmag > 1.0e-9f) { daxa_f32 sc = bound / fmag; ft0 *= sc; ft1 *= sc; }
    m->contacts[j].tangent_impulse[0] = ft0;
    m->contacts[j].tangent_impulse[1] = ft1;
    if (within_cone) // static regime: grow both tangent penalties (reference Eq 16)
    {
      m->contacts[j].tangent_mass[0] = min(m->contacts[j].tangent_mass[0] + BB_AVBD_BETA * abs(Ct0), BB_AVBD_PENALTY_MAX);
      m->contacts[j].tangent_mass[1] = min(m->contacts[j].tangent_mass[1] + BB_AVBD_BETA * abs(Ct1), BB_AVBD_PENALTY_MAX);
    }
    daxa_f32 drift2 = C0t0 * C0t0 + C0t1 * C0t1;
    // stick determination (reference: within the cone AND drift below the slop)
    m->contacts[j].stick = (within_cone && drift2 < BB_AVBD_STICK_THRESH * BB_AVBD_STICK_THRESH) ? 1u : 0u;
  }
}
