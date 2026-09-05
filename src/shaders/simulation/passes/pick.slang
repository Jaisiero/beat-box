#include "rigid_body_math.slang"

[[vk::push_constant]] PickSpringPushConstants PSP;

[shader("compute")]
    [numthreads(1, 1, 1)] void entry_pick_spring(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config = (SimConfig*)(PSP.task_head.sim_config);
  RigidBody* rigid_bodies = (RigidBody*)(PSP.task_head.rigid_bodies);
  PickState* pick = (PickState*)(PSP.task_head.pick_state);

  daxa_u32 flags = pick->flags;

  // --- grab: pick button pressed this frame -> ray-cast the live scene ---
  if ((flags & BB_PICK_REQUEST) != 0u)
  {
    pick->flags = flags & ~BB_PICK_REQUEST; // consume once, including catch-up steps
    pick->grab_count += 1u; // diagnostic: counts EXECUTED raycasts (should be ~1 per click)
    daxa_f32 best_t = 1e30f;
    daxa_u32 best_i = MAX_U32;
    bool best_dynamic = false;
    daxa_f32vec3 best_anchor = daxa_f32vec3(0.0f);
    // ALL bodies compete for the nearest hit — statics too. Statics are OCCLUDERS: without them,
    // a click on the floor or a pool wall sailed straight through and grabbed some unrelated cube
    // BEHIND it ("the drag force affects other boxes"). Only a nearest hit that is dynamic grabs.
    for (daxa_u32 i = 0u; i < sim_config->rigid_body_count; ++i)
    {
      RigidBody* r = &rigid_bodies[i];
      Ray ray;
      ray.origin = r->world_to_object(pick->ray_origin);
      ray.direction = r->rotate_vector_inverse(pick->ray_dir); // rotation preserves |d| -> same t scale
      Aabb aabb = {r->minimum, r->maximum};
      daxa_f32 t = hitAabb(aabb, ray);
      if (t > 0.0f && t < best_t)
      {
        best_t = t;
        best_i = i;
        best_dynamic = (r->flags & RigidBodyFlag::DYNAMIC) != RigidBodyFlag::NONE;
        best_anchor = ray.origin + ray.direction * t; // hit point in the body's LOCAL frame
      }
    }
    // store the PERSISTENT id, not the row: the morton reorder re-sorts the rows every step,
    // so a row index would silently drift onto a different body ("the drag loses its box")
    pick->picked_id = (best_i != MAX_U32 && best_dynamic) ? rigid_bodies[best_i].id : MAX_U32;
    pick->grab_t = best_t;
    pick->anchor_local = best_anchor;
  }

  // --- drag: left button held -> velocity spring toward the ray point at the grab distance ---
  if ((flags & BB_PICK_DRAGGING) != 0u && pick->picked_id != MAX_U32)
  {
    // Resolve the persistent id to THIS step's row by SEARCHING (1 thread x N bodies: trivial).
    // Deliberately NOT via rigid_body_map: this pass runs FIRST in the step, BEFORE the reorder,
    // and the map is one layout out of phase at that point. Because the sort is by MORTON code
    // (spatial locality), a stale map[id] resolved to a row now occupied by a spatially NEARBY
    // box — the spring sprayed impulses across the grabbed box's neighbourhood, which read as
    // "boxes near the anchor get attracted like gravity" (user-reported, trace-confirmed:
    // one picked_id, many affected bodies).
    daxa_u32 row = MAX_U32;
    for (daxa_u32 i = 0u; i < sim_config->rigid_body_count; ++i)
    {
      if (rigid_bodies[i].id == pick->picked_id) { row = i; break; }
    }
    if (row == MAX_U32) { return; }
    RigidBody* r = &rigid_bodies[row];
    if ((r->flags & RigidBodyFlag::DYNAMIC) != RigidBodyFlag::NONE)
    {
      // WAKE on drag: a sleeping body skips advect/solve/integrate, so the spring's velocity
      // write would be inert (and no contact partner moves to veto it awake). The pick IS the
      // external force the sleep system cannot see - clear the flag and the quiet timer.
      r->flags = RigidBodyFlag(daxa_u32(r->flags) & ~daxa_u32(RigidBodyFlag::SLEEPING));
      r->sleep_timer = 0u;
      daxa_f32 PULL_RATE = 10.0f;      // 1/s: desired closing speed per meter of separation
      daxa_f32 MAX_PULL_SPEED = 12.0f; // m/s: match the terminal clamp. Note: a fast cursor
                                       // sweep carries the box through the pile at this speed,
                                       // shoving what it TOUCHES (real collisions); the spring
                                       // itself only ever writes the grabbed body
      daxa_f32 GAIN = 0.5f;            // velocity under-relaxation (smooth convergence at 60 Hz)
      daxa_f32vec3 target = pick->ray_origin + pick->ray_dir * pick->grab_t;
      daxa_f32vec3 anchor_w = r->object_to_world(pick->anchor_local);
      daxa_f32vec3 arm = anchor_w - r->position;
      daxa_f32vec3 desired_v = (target - anchor_w) * PULL_RATE;
      daxa_f32 speed = length(desired_v);
      if (speed > MAX_PULL_SPEED) { desired_v *= MAX_PULL_SPEED / speed; }

      // Sequential point-constraint rows include rotational effective mass. Using
      // body mass alone over-corrects an off-center grab and makes the spring spin.
      // Recompute point velocity after each row; release preserves both velocities.
      daxa_f32mat3x3 inv_i = r->get_transformed_inertia();
      [unroll]
      for (daxa_u32 axis = 0u; axis < 3u; ++axis)
      {
        daxa_f32vec3 n = daxa_f32vec3(0.0f);
        n[axis] = 1.0f;
        daxa_f32vec3 angular_j = cross(arm, n);
        daxa_f32 k = r->inv_mass + dot(angular_j, mul(inv_i, angular_j));
        daxa_f32vec3 point_v = r->velocity + cross(r->omega, arm);
        daxa_f32 impulse = GAIN * dot(desired_v - point_v, n) / max(k, 1.0e-8f);
        r->velocity += n * (impulse * r->inv_mass);
        r->omega += mul(inv_i, angular_j) * impulse;
      }
    }
  }
  else if ((flags & BB_PICK_DRAGGING) == 0u)
  {
    pick->picked_id = MAX_U32; // released -> drop the grab
  }
}
