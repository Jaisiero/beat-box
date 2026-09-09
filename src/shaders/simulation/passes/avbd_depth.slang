#include "avbd_common.slang"

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_depth_reset(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(AVBD_PC.task_head.sim_config);
  if (sim_config->solver_type != SimSolverType::AVBD)
  {
    return;
  }
  if (compute_i.x == 0u)
  {
    sim_config->avbd_max_support_depth = 0u; // A2: reset before the per-body reduction (entry_avbd_max_depth)
  }
  if (compute_i.x < BB_AVBD_SHOCK_LAYERS)
    sim_config->avbd_layer_color_mask[compute_i.x] = 0u;
  daxa_u32 i = compute_i.x;
  if (i >= sim_config->rigid_body_count)
  {
    return;
  }
  RigidBody* rigid_bodies = (RigidBody*)(AVBD_PC.task_head.rigid_bodies);
  AvbdBodyState* states = (AvbdBodyState*)(AVBD_PC.task_head.avbd_state);
  RigidBody* r = &rigid_bodies[i];
  bool grounded = !avbd_is_dynamic(r) || (r->flags & RigidBodyFlag::SLEEPING) != RigidBodyFlag::NONE;
  states[i].support_depth = grounded ? 0u : MAX_U32;
}

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_depth_relax(daxa_u32vec3 compute_i : SV_DispatchThreadID)
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
  if (m->contact_count <= 0)
  {
    return;
  }
  AvbdBodyState* states = (AvbdBodyState*)(AVBD_PC.task_head.avbd_state);
  daxa_u32 d1, d2;
  InterlockedAdd(states[b1].support_depth, 0u, d1);
  InterlockedAdd(states[b2].support_depth, 0u, d2);
  daxa_u32 prev;
  if (d2 != MAX_U32 && d2 + 1u < d1)
  {
    InterlockedMin(states[b1].support_depth, d2 + 1u, prev);
  }
  if (d1 != MAX_U32 && d1 + 1u < d2)
  {
    InterlockedMin(states[b2].support_depth, d1 + 1u, prev);
  }
}

[shader("compute")]
[numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_avbd_max_depth(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(AVBD_PC.task_head.sim_config);
  if (sim_config->solver_type != SimSolverType::AVBD)
  {
    return;
  }
  daxa_u32 i = compute_i.x;
  if (i >= sim_config->rigid_body_count)
  {
    return;
  }
  RigidBody* r = &((RigidBody*)(AVBD_PC.task_head.rigid_bodies))[i];
  if (!avbd_is_dynamic(r) || (r->flags & RigidBodyFlag::SLEEPING) != RigidBodyFlag::NONE)
  {
    return;
  }
  AvbdBodyState* s = &((AvbdBodyState*)(AVBD_PC.task_head.avbd_state))[i];
  daxa_u32 layer = min(s->support_depth, BB_AVBD_SHOCK_LAYERS - 1u); // same clamp the cascade filter uses
  daxa_u32 prev;
  InterlockedMax(sim_config->avbd_max_support_depth, layer, prev);
  daxa_u32 color = ((daxa_u32*)AVBD_PC.task_head.body_color)[i];
  if (color < BB_AVBD_MAX_BODY_COLORS)
    InterlockedOr(sim_config->avbd_layer_color_mask[layer], 1u << color, prev);
}
