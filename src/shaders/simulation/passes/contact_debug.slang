#include "rigid_body_math.slang"

const static daxa_u32 LINE_POINT_COUNT = 2;
const static daxa::f32 LINE_SIZE = 0.5f;

[[vk::push_constant]] CreatePointsPushConstants CP;

[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void
    entry_create_contact_points(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(CP.task_head.sim_config);

  daxa_u32 collision_count = min(sim_config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  if (compute_i.x >= collision_count)
  {
    return;
  }

  Manifold* manifolds = (Manifold*)(CP.task_head.collisions);
  Manifold* m = &manifolds[compute_i.x];

  daxa_u32 contact_count = m->contact_count > 0 ? (daxa_u32)m->contact_count : 0u;
  if(contact_count == 0u)
  {
    return;
  }

  // count ONLY when the vertices are actually written (DEBUG_INFO on): the GUI draws
  // collision_point_count vertices from these buffers, so a count taken while the flag was
  // off points the draw at stale/garbage data (the TAB-after-full-sleep ghost contacts)
  if (!sim_config->has_flag(SimFlag::DEBUG_INFO))
  {
    return;
  }
  daxa_u32 index = 0;
  InterlockedAdd(sim_config->g_c_info.collision_point_count, contact_count, index);
  if(index >= BB_MAX_DEBUG_CONTACT_POINT_COUNT)
  {
    return;
  }
  contact_count = min(contact_count, BB_MAX_DEBUG_CONTACT_POINT_COUNT - index);

  {
    var vertex_buffer = (GUIVertex*)(CP.task_head.vertex_buffer);
    var line_vertex_buffer = (GUIVertexLine*)(CP.task_head.line_vertex_buffer);

    // contact tint: by graph color (debug) or by warm-start
    bool by_graph = sim_config->has_flag(SimFlag::DEBUG_GRAPH_COLORS);
    daxa::f32vec3 gcol = daxa::f32vec3(1.0f, 1.0f, 1.0f); // uncolored/overflow => white
    if (by_graph)
    {
      // AVBD colors BODIES (body_color); the PGS/TGS family colors CONTACTS (manifold_color). The
      // debug view used to always read manifold_color -> all-red for AVBD (whose manifold_color is
      // never set). Pick the buffer matching the active solver so the tint is meaningful for both.
      daxa_u32 gc; bool colored;
      if (sim_config->solver_type == SimSolverType::AVBD)
      {
        daxa_u32* body_color = (daxa_u32*)(CP.task_head.body_color);
        gc = body_color[m->obb1_index];          // tint by this contact's body-A color
        colored = gc < BB_AVBD_MAX_BODY_COLORS;
      }
      else
      {
        daxa_u32* manifold_color = (daxa_u32*)(CP.task_head.manifold_color);
        gc = manifold_color[compute_i.x];
        colored = gc < BB_MAX_COLORS;
      }
      if (colored)
      {
        // golden-ratio hue scramble: adjacent graph colors land far apart on the hue wheel,
        // so the colors actually used stay clearly distinguishable
        daxa::f32 h = frac((daxa::f32)gc * 0.6180339887f) * 6.0f;
        gcol = clamp(daxa::f32vec3(abs(h - 3.0f) - 1.0f, 2.0f - abs(h - 2.0f), 2.0f - abs(h - 4.0f)), 0.0f, 1.0f);
      }
    }

    for (daxa_u32 i = 0; i < contact_count; i++)
    {
      daxa_u32 current_index = index + i;
      vertex_buffer[current_index].position = m->contacts[i].position;
      if (by_graph)
      {
        vertex_buffer[current_index].color = gcol;
      }
      else
      {
        daxa::f32 blue = (daxa::f32)(255 - m->contacts[i].warm_start) / 255.0f;
        daxa::f32 red = 1.0f - blue;
        vertex_buffer[current_index].color = daxa::f32vec3(red, 0.0f, blue);
      }

      line_vertex_buffer[current_index * LINE_POINT_COUNT].position = m->contacts[i].position;
      line_vertex_buffer[current_index * LINE_POINT_COUNT + 1].position = m->contacts[i].position + m->normal * LINE_SIZE;
    }
  }
}
