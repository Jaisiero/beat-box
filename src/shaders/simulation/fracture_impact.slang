#include "shared.inl"
[[vk::push_constant]] FractureImpactPushConstants P;

[shader("compute")]
[numthreads(64, 1, 1)]
void entry_fracture_impact_reset(uint3 tid : SV_DispatchThreadID)
{
  SimConfig* config = (SimConfig*)P.config_addr;
  FractureEventBuffer* events = (FractureEventBuffer*)config->fracture_events_addr;
  FractureImpactScratch* scratch = (FractureImpactScratch*)events->scratch_addr;
  if (tid.x < BB_MAX_COLLISION_COUNT) scratch->candidates[tid.x].impulse = 0.0f;
  if (tid.x < BB_MAX_RIGID_BODY_COUNT) scratch->selected[tid.x].impulse = 0.0f;
}

// A total ordering makes equal impulses independent of contact-list insertion order.
bool precedes(FractureEvent a, FractureEvent b)
{
  if (a.impulse != b.impulse) return a.impulse > b.impulse;
  for (uint k = 0u; k < 3u; ++k)
    if (a.position[k] != b.position[k]) return a.position[k] < b.position[k];
  for (uint k = 0u; k < 3u; ++k)
    if (a.normal[k] != b.normal[k]) return a.normal[k] < b.normal[k];
  return false;
}

[shader("compute")]
[numthreads(64, 1, 1)]
void entry_fracture_impact_select(uint3 tid : SV_DispatchThreadID)
{
  SimConfig* config = (SimConfig*)P.config_addr;
  uint i = tid.x;
  if (i >= min(config->rigid_body_count, BB_MAX_RIGID_BODY_COUNT)) return;
  RigidBody body = ((RigidBody*)P.bodies_addr)[i];
  // Two retained components need at least three voxels each. Smaller bodies
  // cannot split after sliver merging, so they must not trigger a host batch.
  if (!body.has_flag(RigidBodyFlag::DYNAMIC) || body.has_flag(RigidBodyFlag::SLEEPING)
      || body.shape_index == 0u || body.primitive_count < 6u || !(body.fracture_impulse > 0.0f)
      || body.id >= BB_MAX_RIGID_BODY_COUNT) return;
  FractureEventBuffer* events = (FractureEventBuffer*)config->fracture_events_addr;
  FractureImpactScratch* scratch = (FractureImpactScratch*)events->scratch_addr;
  ManifoldNode* nodes = (ManifoldNode*)P.nodes_addr;
  CollisionEntry* map = (CollisionEntry*)P.map_addr;
  Manifold* manifolds = (Manifold*)P.manifolds_addr;
  uint node_count = min(config->manifold_node_count, BB_MAX_MANIFOLD_NODE_COUNT);
  uint count = min(config->g_c_info.collision_count, BB_MAX_COLLISION_COUNT);
  FractureEvent best = (FractureEvent)0;
  uint node = body.manifold_node_index;
  for (uint visited = 0u; node < node_count && visited < node_count; ++visited)
  {
    uint slot = nodes[node].manifold_id;
    uint packed = slot < count ? map[slot].index : MAX_U32;
    if (packed < count && (manifolds[packed].obb1_index == i || manifolds[packed].obb2_index == i))
    {
      // A zero impulse marks an unwritten candidate; do not load stale payloads.
      if (scratch->candidates[packed].impulse > body.fracture_impulse)
      {
        FractureEvent candidate = scratch->candidates[packed];
        if (precedes(candidate, best)) best = candidate;
      }
    }
    node = nodes[node].next < 0 ? MAX_U32 : uint(nodes[node].next);
  }
  best.body_id = body.id;
  scratch->selected[body.id] = best;
}

[shader("compute")]
[numthreads(1, 1, 1)]
void entry_fracture_impact_publish(uint3 tid : SV_DispatchThreadID)
{
  SimConfig* config = (SimConfig*)P.config_addr;
  FractureEventBuffer* events = (FractureEventBuffer*)config->fracture_events_addr;
  FractureImpactScratch* scratch = (FractureImpactScratch*)events->scratch_addr;
  bool consumed = events->consumed_serial == events->serial;
  bool changed = false;
  uint count = 0u;
  for (uint id = 0u; id < BB_MAX_RIGID_BODY_COUNT; ++id)
  {
    FractureEvent best = (FractureEvent)0;
    if (!consumed && scratch->pending[id].impulse > 0.0f) best = scratch->pending[id];
    if (scratch->selected[id].impulse > 0.0f)
    {
      FractureEvent candidate = scratch->selected[id];
      if (precedes(candidate,best)) { best = candidate; changed = true; }
    }
    scratch->pending[id] = best;
    if (best.impulse > 0.0f) events->events[count++] = best;
  }
  events->count = count;
  if (changed) ++events->serial;
}
