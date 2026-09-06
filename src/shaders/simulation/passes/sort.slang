#include "rigid_body_math.slang"
#include "LBVH.slang"

[[vk::push_constant]] RigidBodyGenerateMortonCodePushConstants GMC;
[[vk::push_constant]] RigidBodyRadixSortHistogramPushConstants RBRSH;
[[vk::push_constant]] RigidBodySingleRadixSortPushConstants SRS;
[[vk::push_constant]] IslandDispatcherPushConstants ID;

#if defined(BB_RADIX_SORT_HISTOGRAM)
groupshared Atomic<daxa_u32> histogram[RADIX_SORT_BINS];
#endif
static const daxa_u32 SWS_THREADS = 128;
static const daxa_u32 SWS_ITEMS = 8; // 128 * 8 = 1024 = BB_MAX_RIGID_BODY_COUNT
groupshared daxa_u32 sws_key[2][1024];
groupshared daxa_u32 sws_val[2][1024];
groupshared daxa_u32 sws_cnt[16 * SWS_THREADS]; // [digit * THREADS + thread]
groupshared daxa_u32 sws_base[16];
[shader("compute")]
    [numthreads(RIGID_BODY_SIM_COMPUTE_X, 1, 1)] void entry_generate_morton_codes(daxa_u32vec3 compute_i : SV_DispatchThreadID)
{
  SimConfig* sim_config =
      (SimConfig*)(GMC.task_head.sim_config);

  daxa_u32 index = compute_i.x;
  if (index >= sim_config->rigid_body_count)
  {
    return;
  }

  RigidBody* rigid_bodies =
      (RigidBody*)(GMC.task_head.rigid_bodies);

  MortonCode* morton_codes =
      (MortonCode*)(GMC.task_head.morton_codes);

  var r = &rigid_bodies[index];

  // Reset the manifold-chain head at the START of every step (first per-body pass). narrow_phase
  // rebuilds the chain; clearing it here makes the head fresh-by-construction so a step resumed
  // after a pause can't prepend new nodes onto a stale chain tail left in whatever buffer parity
  // the resume lands on (the residual resume kick the coherent pause flush alone doesn't remove).
  // Idempotent in normal stepping (the end-of-step update pass already leaves it MAX_U32). The
  // warm-start reads the PREVIOUS body's head, so clearing the current head does not affect it.
  r->manifold_node_index = MAX_U32;

  // Normalize the world position into the unit cube [0,1] expected by morton3D.
  // Without this, raw world coordinates (floor at y=-50, cubes spread over tens of
  // units) saturate to degenerate values -> duplicate Morton codes -> a malformed
  // Karras LBVH -> broad phase finds no overlaps -> no collisions at all.
  const daxa_f32vec3 WORLD_MIN = daxa_f32vec3(-64.0f, -64.0f, -64.0f);
  const daxa_f32vec3 WORLD_MAX = daxa_f32vec3(64.0f, 64.0f, 64.0f);
  daxa_f32vec3 normalized_position = (r->position - WORLD_MIN) / (WORLD_MAX - WORLD_MIN);

  // UNIQUE SORT KEYS (determinism): deeply overlapping bodies quantize to the SAME
  // Morton cell, and the radix tie order for equal keys is a thread race - the sorted
  // index assignment (and everything its order feeds) then differs run to run. Keep the
  // top 21 Morton bits (7/axis = 128^3 cells over the +-64 world ~= one cube per cell,
  // plenty of BVH granularity; overlap tests stay exact) and break ties with the
  // PERSISTENT body id in the low 10 bits: keys are unique, so the sort order is a pure
  // function of body poses regardless of radix stability.
  let morton_code = MortonCodeGenerator.morton3D(normalized_position);
  let unique_key = ((morton_code >> 9u) << 10u) | (r->id & 0x3FFu);

#if defined(BB_DEBUG)
  r->clear_flag(RigidBodyFlag::COLLIDING);
  r->face_collided = 0u;
#endif // BB_DEBUG

  morton_codes[index] = MortonCode(unique_key, index);
}

#if defined(BB_RADIX_SORT_HISTOGRAM)
[numthreads(RADIX_SORT_WORKGROUP_SIZE, 1, 1)] void entry_radix_sort_histogram(daxa_u32vec3 compute_i : SV_DispatchThreadID, daxa_u32vec3 GroupThreadID : SV_GroupThreadID, daxa_u32vec3 GroupID : SV_GroupID)
{
  let lID = GroupThreadID.x;
  let wID = GroupID.x;

  var global_histograms =
      (daxa_u32*)(RBRSH.task_head.global_histograms);

  var morton_codes =
      (MortonCode*)(RBRSH.task_head.morton_codes);

  let sim_config =
      (SimConfig*)(RBRSH.task_head.sim_config);

  let shift = sim_config->radix_shift;

  // Initialize histogram
  if (lID < RADIX_SORT_BINS)
  {
    histogram[lID] = 0U;
  }
  GroupMemoryBarrierWithGroupSync();

  // Compute histogram
  for (daxa_u32 index = 0; index < NUM_BLOCKS_PER_WORKGROUP; index++)
  {
    let element_id = wID * NUM_BLOCKS_PER_WORKGROUP * RADIX_SORT_WORKGROUP_SIZE + index * RADIX_SORT_WORKGROUP_SIZE + lID;
    if (element_id < sim_config->rigid_body_count)
    {
      let val = morton_codes[element_id].morton_code;
      let bin = (val >> shift) & (RADIX_SORT_BINS - 1);
      histogram[bin].add(1U);
    }
  }
  GroupMemoryBarrierWithGroupSync();

  // Write histogram to global memory
  if (lID < RADIX_SORT_BINS)
  {
    let value = histogram[lID].load();
    global_histograms[RADIX_SORT_BINS * wID + lID] = value;
  }
}
#endif

[shader("compute")]
    [numthreads(RADIX_SORT_WORKGROUP_SIZE, 1, 1)] void entry_single_radix_sort(
        daxa_u32vec3 dispatch_thread_id : SV_DispatchThreadID, daxa_u32vec3 group_thread_id : SV_GroupThreadID, daxa_u32vec3 group_id : SV_GroupID, daxa_u32 groud_index : SV_GroupIndex // Flattened thread index in the group
    )
{
  daxa_u32 gID = dispatch_thread_id.x;
  daxa_u32 lID = group_thread_id.x;
  daxa_u32 wID = group_id.x;

  // Lane index in the current wave (subgroup)
  daxa_u32 lsID = WaveGetLaneIndex();
  // Wave ID within the thread group:
  // We'll assume sID = (lID / SUBGROUP_SIZE), since SUBGROUP_SIZE divides WORKGROUP_SIZE evenly.
  daxa_u32 sID = lID / SUBGROUP_SIZE;

  DispatchBuffer* dispatch_buffer =
      (DispatchBuffer*)(SRS.task_head.dispatch_buffer);

  daxa_u32* global_histograms =
      (daxa_u32*)(SRS.task_head.global_histograms);

  MortonCode* morton_codes_in =
      (MortonCode*)(SRS.task_head.morton_codes_in);

  MortonCode* morton_codes_out =
      (MortonCode*)(SRS.task_head.morton_codes_out);

  SimConfig* sim_config =
      (SimConfig*)(SRS.task_head.sim_config);

  daxa_u32 num_of_workgroups = dispatch_buffer->radix_sort_rigid_body_dispatch.x;

  daxa_u32 shift = sim_config->radix_shift;

  daxa_u32 num_of_elements = sim_config->rigid_body_count;

#if defined(BB_RADIX_SORT)
  radix_sort(lID, wID, lsID, sID, num_of_elements, num_of_workgroups, shift,
             morton_codes_in, morton_codes_out, global_histograms);
#endif // BB_RADIX_SORT
}

[shader("compute")]
[numthreads(SWS_THREADS, 1, 1)] void entry_single_workgroup_sort(daxa_u32vec3 gtid : SV_GroupThreadID)
{
  SimConfig* sim_config = (SimConfig*)(SRS.task_head.sim_config);
  MortonCode* codes = (MortonCode*)(SRS.task_head.morton_codes_in);
  daxa_u32 t = gtid.x;
  daxa_u32 n = min(sim_config->rigid_body_count, 1024u);
  // load; the padded tail sorts to the end (MAX keys) and is never read downstream
  for (daxa_u32 i = 0u; i < SWS_ITEMS; ++i)
  {
    daxa_u32 j = t * SWS_ITEMS + i;
    sws_key[0][j] = j < n ? codes[j].morton_code : MAX_U32;
    sws_val[0][j] = j < n ? codes[j].index : MAX_U32;
  }
  GroupMemoryBarrierWithGroupSync();
  daxa_u32 src = 0u;
  for (daxa_u32 pass = 0u; pass < 8u; ++pass)
  {
    daxa_u32 shift = pass * 4u;
    // per-thread digit counts over this thread's contiguous block
    daxa_u32 cnt[16];
    for (daxa_u32 d = 0u; d < 16u; ++d) { cnt[d] = 0u; }
    for (daxa_u32 i = 0u; i < SWS_ITEMS; ++i)
    {
      daxa_u32 d = (sws_key[src][t * SWS_ITEMS + i] >> shift) & 15u;
      ++cnt[d];
    }
    for (daxa_u32 d = 0u; d < 16u; ++d) { sws_cnt[d * SWS_THREADS + t] = cnt[d]; }
    GroupMemoryBarrierWithGroupSync();
    // exclusive scan across threads per digit (16 threads, one digit column each)
    if (t < 16u)
    {
      daxa_u32 sum = 0u;
      for (daxa_u32 q = 0u; q < SWS_THREADS; ++q)
      {
        daxa_u32 c = sws_cnt[t * SWS_THREADS + q];
        sws_cnt[t * SWS_THREADS + q] = sum;
        sum += c;
      }
      sws_base[t] = sum;
    }
    GroupMemoryBarrierWithGroupSync();
    if (t == 0u)
    {
      // exclusive scan of the 16 digit totals
      daxa_u32 sum = 0u;
      for (daxa_u32 d = 0u; d < 16u; ++d)
      {
        daxa_u32 c = sws_base[d];
        sws_base[d] = sum;
        sum += c;
      }
    }
    GroupMemoryBarrierWithGroupSync();
    // stable scatter: in-thread sequential ranks on top of the scanned offsets
    daxa_u32 local_rank[16];
    for (daxa_u32 d = 0u; d < 16u; ++d) { local_rank[d] = 0u; }
    for (daxa_u32 i = 0u; i < SWS_ITEMS; ++i)
    {
      daxa_u32 j = t * SWS_ITEMS + i;
      daxa_u32 k = sws_key[src][j];
      daxa_u32 d = (k >> shift) & 15u;
      daxa_u32 dest = sws_base[d] + sws_cnt[d * SWS_THREADS + t] + local_rank[d];
      ++local_rank[d];
      sws_key[1u - src][dest] = k;
      sws_val[1u - src][dest] = sws_val[src][j];
    }
    GroupMemoryBarrierWithGroupSync();
    src = 1u - src;
  }
  for (daxa_u32 i = 0u; i < SWS_ITEMS; ++i)
  {
    daxa_u32 j = t * SWS_ITEMS + i;
    if (j < n) { codes[j] = MortonCode(sws_key[src][j], sws_val[src][j]); }
  }
}
