#define _CRT_SECURE_NO_WARNINGS // std::getenv (BB_POOL_VERIFY / BB_RESPAWN_TIMING) on MSVC
#include "acceleration_structure_manager.hpp"
#include "renderer_manager.hpp"
#include "gui_manager.hpp"
#include <cstdlib> // std::getenv (BB_RESPAWN_TIMING incremental-AS diagnostic)

BB_NAMESPACE_BEGIN

AccelerationStructureManager::AccelerationStructureManager(daxa::Device &device, std::shared_ptr<TaskManager> task_manager) : device(device), task_manager(task_manager)
{
  if (device.is_valid())
  {
    auto const &properties = device.properties();
    acceleration_structure_scratch_offset_alignment = properties.acceleration_structure_properties.has_value()
      ? properties.acceleration_structure_properties.value().min_acceleration_structure_scratch_offset_alignment
      : ACCELERATION_STRUCTURE_BUILD_OFFSET_ALIGMENT;

    update_pipeline = task_manager->create_compute(UpdateAccelerationStructures{}.info);
  }
}

AccelerationStructureManager::~AccelerationStructureManager()
{
  destroy();
}

bool AccelerationStructureManager::create(std::shared_ptr<RendererManager> renderer, std::shared_ptr<RigidBodyManager> rigid_body, std::shared_ptr<GUIManager> gui)
{
  if (device.is_valid() && !initialized)
  {
    renderer_manager = renderer.get();
    rigid_body_manager = rigid_body.get();
    gui_manager = gui.get();

    // Create buffer for RigidBodies
    rigid_body_scratch_buffer = device.create_buffer({
        .size = MAX_RIGID_BODY_COUNT * sizeof(RigidBody),
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
        .name = "rigid_body_scratch_buffer",
    });

    // Create buffer for RigidBodies
    for (auto f = 0; f < DOUBLE_BUFFERING; ++f)
      rigid_body_buffer[f] = device.create_buffer({
          .size = MAX_RIGID_BODY_COUNT * sizeof(RigidBody),
          .name = "rigid_body_buffer_ " + std::to_string(f),
      });

    // Create scratch buffer for primitives
    primitive_scratch_buffer = device.create_buffer({
        .size = MAX_PRIMITIVE_COUNT * sizeof(Aabb),
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
        .name = "primitive_scratch_buffer",
    });

    // Create buffer for primitives
    primitive_buffer = device.create_buffer({
        .size = MAX_PRIMITIVE_COUNT * sizeof(Aabb),
        .name = "primitive_buffer",
    });

    // Create BLAS buffer
    proc_blas_buffer = device.create_buffer({
        .size = BLAS_POOL_BUDGET,
        .name = "proc_blas_buffer",
    });

    // Create BLAS scratch buffer
    proc_blas_scratch_buffer = device.create_buffer({
        .size = BLAS_POOL_BUDGET,
        .name = "proc_blas_scratch_buffer",
    });

    // Create TLAS buffer
    proc_tlas_buffer = device.create_buffer({
        .size = AVERAGE_AS_SIZE,
        .name = "proc_tlas_buffer",
    });

    // Create TLAS scratch buffer
    proc_tlas_scratch_buffer = device.create_buffer({
        .size = AVERAGE_AS_SIZE,
        .name = "proc_tlas_scratch_buffer",
    });

    // Create BLAS instances buffer. +1 slot: the LBVH/BVH path appends one aggregate BLAS
    // instance at index [current_rigid_body_count] AFTER the per-body instances, so a full-
    // capacity scene (MAX_RIGID_BODY_COUNT == MAX_ACCELERATION_STRUCTURE_COUNT bodies) would
    // otherwise write one past the end (and the TLAS build would read total_instances = MAX+1).
    blas_instances_buffer = device.create_buffer({
        .size = sizeof(daxa_BlasInstanceData) * (MAX_ACCELERATION_STRUCTURE_COUNT + 1),
        .memory_flags = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
        .name = "blas_instances_buffer",
    });

    blas_instances_data = device.buffer_host_address_as<daxa_BlasInstanceData>(blas_instances_buffer).value();

    // Create TLAS
    for (auto f = 0; f < DOUBLE_BUFFERING; ++f)
      tlas[f] = device.create_tlas({
          .size = AVERAGE_AS_SIZE,
          .name = "tlas_" + std::to_string(f),
      });

    // Set the buffers for the tasks
    if (task_aabb_buffer.id() != primitive_buffer) { task_aabb_buffer.set_buffer(primitive_buffer); }
    if (task_blas_instance_data.id() != blas_instances_buffer) { task_blas_instance_data.set_buffer(blas_instances_buffer); }

    // Create a temporary placeholder BLAS to satisfy task graph compilation requirements
    placeholder_blas = device.create_blas_from_buffer(
        {{
             .size = 256,
             .name = "placeholder_blas",
         },
         proc_blas_buffer,
         0});

    task_blas.set_blas(placeholder_blas);
    task_tlas.set_tlas(tlas[0]);
    task_dispatch_buffer.set_buffer(proc_blas_scratch_buffer); // Bind a placeholder buffer

    record_accel_struct_tasks(AS_build_TG);
    AS_build_TG.submit();
    AS_build_TG.complete();

    record_update_TLAS_tasks(TLAS_update_TG, TLAS_build_TG, update_pipeline);
    TLAS_update_TG.submit();
    TLAS_update_TG.complete();
    // the TLAS build is the LAST compute submit of a sim publication: it signals the sim
    // timeline (value set via advance_sim_timeline right before each execute)
    TLAS_build_TG.submit({.additional_signal_timeline_semaphores = &task_manager->gpu->sim_signal_span});
    TLAS_build_TG.complete();

    record_update_AS_buffers_tasks(AS_update_buffers_TG);
    AS_update_buffers_TG.submit();
    AS_update_buffers_TG.complete();

    initialized = true;
  }

  return initialized;
}

void AccelerationStructureManager::destroy()
{
  if (initialized)
  {
    device.destroy_buffer(rigid_body_scratch_buffer);
    for (auto f = 0; f < DOUBLE_BUFFERING; ++f)
      device.destroy_buffer(rigid_body_buffer[f]);
    device.destroy_buffer(primitive_scratch_buffer);
    device.destroy_buffer(primitive_buffer);
    // INCREMENTAL-AS: body_blas_ is the live set (incremental may hold handles not in proc_blas);
    // it is the single destruction owner (see build_accel_structs). Empty on a never-built manager.
    for (auto &blas : body_blas_)
    {
      if (!blas.is_empty()) { device.destroy_blas(blas); blas = {}; }
    }
    if (!placeholder_blas.is_empty())
    {
      device.destroy_blas(placeholder_blas);
      placeholder_blas = {};
    }
    device.destroy_buffer(proc_blas_buffer);
    device.destroy_buffer(proc_blas_scratch_buffer);
    device.destroy_buffer(proc_tlas_buffer);
    device.destroy_buffer(proc_tlas_scratch_buffer);
    device.destroy_buffer(blas_instances_buffer);
    for (auto f = 0; f < DOUBLE_BUFFERING; ++f)
      device.destroy_tlas(tlas[f]);
    // Destroy the per-frame LBVH BLAS too (only update() freed the previous handle before
    // overwriting; without this, any LBVH BLAS live at teardown leaks its AS allocation).
    for (auto f = 0; f < DOUBLE_BUFFERING; ++f)
      if (!lbvh_blas[f].is_empty())
        device.destroy_blas(lbvh_blas[f]);
    initialized = false;
  }
}

// free_accel_structs() was DELETED here (review v3): it was unused (zero call sites), documented
// as out of sync with the live reload path, and a double-free landmine if ever wired up (destroyed
// proc_blas without clearing the vector; destroy() would then free the same handles again).
// reset_for_reload() (header) is the live, correct reload path.

daxa::TlasId AccelerationStructureManager::get_tlas()
{
  if(!initialized) {
    return {};
  }
  return tlas[renderer_manager->get_sim_frame_index()];
}

daxa::BufferId AccelerationStructureManager::get_previous_rigid_body_buffer()
{
  if(!initialized) {
    return {};
  }
  return rigid_body_buffer[renderer_manager->get_sim_previous_frame_index()];
}

daxa::BufferId AccelerationStructureManager::get_rigid_body_buffer()
{
  if(!initialized) {
    return {};
  }
  return rigid_body_buffer[renderer_manager->get_sim_frame_index()];
}

daxa::BufferId AccelerationStructureManager::get_next_rigid_body_buffer()
{
  if(!initialized) {
    return {};
  }

  return rigid_body_buffer[renderer_manager->get_sim_next_frame_index()];
}

void AccelerationStructureManager::build_AS()
{
  if(!initialized) {
    return;
  }
  AS_build_TG.execute();

  // CRITICAL: populate BOTH double-buffered rigid body buffers with the scene data.
  // AS_build_TG only writes to whichever buffer task_rigid_bodies happens to be bound
  // to at execute time (which depends on the frame index dance in load_scene, ending
  // up as buffer[1]). But the render and the first simulation step read buffer[0].
  // The authoritative scene data lives in rigid_body_scratch_buffer (memcpy'd from the
  // CPU array in build_accel_structs), so copy it into both buffers to guarantee a
  // consistent starting state regardless of the current frame index.
  auto const rb_data_size = static_cast<daxa::usize>(current_rigid_body_count) * sizeof(RigidBody);
  if (rb_data_size > 0)
  {
    auto rec = device.create_command_recorder({});
    rec.pipeline_barrier({.src_access = daxa::AccessConsts::READ_WRITE,
                          .dst_access = daxa::AccessConsts::TRANSFER_READ_WRITE});
    for (auto f = 0; f < DOUBLE_BUFFERING; ++f)
    {
      rec.copy_buffer_to_buffer({
          .src_buffer = rigid_body_scratch_buffer,
          .dst_buffer = rigid_body_buffer[f],
          .size = rb_data_size,
      });
    }
    rec.pipeline_barrier({.src_access = daxa::AccessConsts::TRANSFER_WRITE,
                          .dst_access = daxa::AccessConsts::READ});
    auto cmds = rec.complete_current_commands();
    device.submit_commands({.command_lists = std::array{cmds}});
    device.wait_idle();
  }
}

bool AccelerationStructureManager::build_accel_structs(std::vector<RigidBody> &rigid_bodies, std::vector<Aabb> const &primitives,
                                                       std::function<void(daxa::BufferId)> const &post_primitive_upload)
{
  if(!initialized) {
    std::cerr << "ERROR: AccelerationStructureManager is not initialized inside build_accel_structs!" << std::endl;
    return false;
  }
  // Get the number of rigid bodies and primitives
  auto rigid_body_count = static_cast<u32>(rigid_bodies.size());
  auto primitive_count = static_cast<u32>(primitives.size());

  // Check if the number of rigid bodies and primitives is within the limits
  if (current_rigid_body_count + rigid_body_count > MAX_RIGID_BODY_COUNT || current_primitive_count + primitive_count > MAX_PRIMITIVE_COUNT)
  {
    std::cerr << "ERROR: Exceeded max rigid bodies (" << (current_rigid_body_count + rigid_body_count) << "/" << MAX_RIGID_BODY_COUNT 
              << ") or primitives (" << (current_primitive_count + primitive_count) << "/" << MAX_PRIMITIVE_COUNT << ")!" << std::endl;
    return false;
  }

  // UNIT-QUATERNION INVARIANT. Hand-authored scene rotations are frequently non-unit (e.g.
  // (0,0,0.5,1) -> |q|^2=1.25, (0,1,1,0.8) -> |q|^2=2.64). RigidBody::to_matrix() (used to build the
  // TLAS instance transform) does NOT normalize, while the path tracer's world_to_object() uses the
  // quaternion sandwich q*.v.q which scales by |q|^2 -- the two diverge for non-unit q, so a cube's
  // ray-traced traversal AABB and its intersection OBB mismatch and the corners render clipped
  // ("dented") at rest, until the first sim step (which normalizes the quaternion) hides it.
  // This is the single canonical chokepoint that establishes the invariant (it runs on every AS build:
  // load, scene switch, runtime spawn) and feeds BOTH the render AS and the sim rigid_body buffers.
  // Downstream the hot conversions (to_matrix / rotate_vector) rely on it instead of paying a per-call
  // normalize. (NOT normalized per-call in those: they are on the per-pair / per-ray hot path.)
  for (size_t i = 0; i < rigid_bodies.size(); ++i)
  {
    auto &rb = rigid_bodies[i];
    daxa_f32 m2 = rb.rotation.v.x * rb.rotation.v.x + rb.rotation.v.y * rb.rotation.v.y +
                  rb.rotation.v.z * rb.rotation.v.z + rb.rotation.w * rb.rotation.w;
#if !defined(NDEBUG)
    // Surface dirty source data to the scene author -- a non-unit authored rotation is the only way the
    // invariant gets violated, so catch it at the door (we still auto-normalize below).
    if (m2 > 1.0e-12f && (m2 < 0.999f || m2 > 1.001f))
      std::cerr << "[WARN] rigid body " << i << " has a non-unit rotation (|q|^2=" << m2
                << "); auto-normalized. Fix the scene authoring to keep |q| == 1." << std::endl;
#endif
    rb.rotation = (m2 > 1.0e-12f) ? rb.rotation.normalize() : Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
  }

  // Copy primitives to the buffer
  std::memcpy(device.buffer_host_address_as<Aabb>(primitive_scratch_buffer).value(), primitives.data(), primitive_count * sizeof(Aabb));
  // GPU-first hook: voxel bodies' AABB ranges are built ON the GPU straight into this
  // scratch (overwriting the oracle-only CPU ranges) before the BLAS build reads it
  if (post_primitive_upload) { post_primitive_upload(primitive_scratch_buffer); }

  // BUILDING BLAS
  auto clear_build_AS = [&]()
  {
    blas_build_infos.clear();
    blas_build_infos.reserve(rigid_body_count);
    blas_build_sizes.clear();
    blas_build_sizes.reserve(rigid_body_count);
    blas_geometries.clear();
    blas_geometries.resize(rigid_body_count);
  };

  clear_build_AS();

  // Reload cleanup: build_accel_structs() is re-entered on every scene reset/switch, with the
  // upload counters rewound by reset_for_reload() so new BLAS reuse the same proc_blas_buffer
  // offsets. The per-body loop below push_back()s fresh BLAS handles, so the PREVIOUS scene's
  // handles must be destroyed and the vector cleared first; otherwise proc_blas grows unbounded,
  // the stale handles alias the reused buffer memory, task_blas binds a stale proc_blas.front()
  // (the first scene's body-0 BLAS), and shutdown double-destroys them. No-op on the initial
  // startup build (proc_blas empty). Safe here: callers run at a synchronized frame boundary.
  //
  // INCREMENTAL-AS ownership: body_blas_ is the single source of truth for BLAS destruction
  // (it holds the live set after any incremental update, which may DIFFER from proc_blas -- the
  // incremental path destroys/creates handles without touching proc_blas). Destroy via body_blas_
  // here (and in destroy()), not proc_blas, so a full rebuild after fractures frees the real live
  // handles and never double-frees a stale one. proc_blas is then just this build's transient list.
  for (auto &blas : body_blas_)
  {
    if (!blas.is_empty()) { device.destroy_blas(blas); blas = {}; }
  }
  proc_blas.clear();
  // a full build re-lays every BLAS from offset 0 (reset_for_reload rewound proc_blas_buffer_offset);
  // the region pool must start empty so its allocations match that fresh dense bump.
  blas_region_pool_.reset();

  /// Alignments:
  auto get_aligned = [&](u64 operand, u64 granularity) -> u64
  {
    return ((operand + (granularity - 1)) & ~(granularity - 1));
  };

  // TODO: one geometry per rigid body
  // TODO: one instance per BLAS

  previous_primitive_count = current_primitive_count;
  previous_rigid_body_count = current_rigid_body_count;

  // Build BLAS for each rigid body
  for (u32 i = 0; i < rigid_body_count; ++i)
  {

    // Get the rigid body
    auto &rigid_body = rigid_bodies[i];

    blas_geometries.at(i).push_back({
        .data = device.device_address(primitive_buffer).value() + current_primitive_count * sizeof(Aabb),
        .stride = sizeof(Aabb),
        .count = rigid_body.primitive_count,
        .flags = daxa::GeometryFlagBits::OPAQUE,
    });

    // Set the primitive offset
    rigid_body.primitive_offset = current_primitive_count;
    current_primitive_count += rigid_body.primitive_count;

    // Increment the scratch offset
    primitive_scratch_offset += rigid_body.primitive_count * sizeof(Aabb);

    // Create BLAS build info
    blas_build_infos.push_back({
        // review v2 #3: per-body BLAS are built ONCE on scene load then traced every frame for the
        // whole session, so optimize for trace, not build (Daxa's own default is FAST_TRACE). The
        // per-frame LBVH/TLAS rebuilds below keep FAST_BUILD (they pay the build cost every frame).
        .flags = daxa::AccelerationStructureBuildFlagBits::PREFER_FAST_TRACE,
        .dst_blas = {},
        .geometries = daxa::Span<const daxa::BlasAabbGeometryInfo>(blas_geometries.at(i).data(), blas_geometries.at(i).size()),
        .scratch_data = {},
    });
    // Get the build sizes
    blas_build_sizes.push_back(device.blas_build_sizes(blas_build_infos.back()));

    auto scratch_offset = get_aligned(blas_build_sizes.back().build_scratch_size, acceleration_structure_scratch_offset_alignment);

    if (proc_blas_scratch_offset + scratch_offset > BLAS_POOL_BUDGET)
    {
      std::cerr << "ERROR: Exceeded BLAS scratch offset limit! Current: " << (proc_blas_scratch_offset + scratch_offset) 
                << ", Limit: " << (BLAS_POOL_BUDGET) << std::endl;
      clear_build_AS();
      return false;
    }

    // Set the scratch offset
    blas_build_infos.back().scratch_data = device.device_address(proc_blas_scratch_buffer).value() + proc_blas_scratch_offset;

    // Increment the scratch offset
    proc_blas_scratch_offset += scratch_offset;

    // Get the BLAS instance offset
    auto blas_instance_offset = get_aligned(blas_build_sizes.back().acceleration_structure_size, ACCELERATION_STRUCTURE_BUILD_OFFSET_ALIGMENT);

    // Check if the buffer offset is within the limits
    if (proc_blas_buffer_offset + blas_instance_offset > BLAS_POOL_BUDGET)
    {
      std::cerr << "ERROR: Exceeded BLAS buffer offset limit! Current: " << (proc_blas_buffer_offset + blas_instance_offset) 
                << ", Limit: " << (BLAS_POOL_BUDGET) << std::endl;
      clear_build_AS();
      return false;
    }

    // Create BLAS buffer from buffer
    proc_blas.push_back(device.create_blas_from_buffer(
        {{
             .size = blas_build_sizes.back().acceleration_structure_size,
             .name = "blas" + std::to_string(i),

         },
         proc_blas_buffer,
         proc_blas_buffer_offset}));
    // Add the BLAS buffer to the BLAS build info
    blas_build_infos.back().dst_blas = proc_blas.back();

    // Increment the buffer offset
    proc_blas_buffer_offset += blas_instance_offset;

    blas_instances_data[i] = {
        .transform = rigid_body.get_instance_transform(),
        .instance_custom_index = i,
        .mask = 0xFF,
        .instance_shader_binding_table_record_offset = 0,
        .flags = {},
        .blas_device_address = device.device_address(proc_blas.back()).value(),
    };
  }

  // Copy rigid bodies to the buffer
  std::memcpy(device.buffer_host_address_as<RigidBody>(rigid_body_scratch_buffer).value(), rigid_bodies.data(), rigid_body_count * sizeof(RigidBody));

  // Increment the rigid body scratch offset
  rigid_body_scratch_offset += rigid_body_count * sizeof(RigidBody);

  // Increment the rigid body and primitive count
  current_rigid_body_count += rigid_body_count;

  // Set Task BLAS
  if (!proc_blas.empty())
  {
    task_blas.set_blas(proc_blas.front());
  }

  tlas_info[0] = {
      .data = device.device_address(blas_instances_buffer).value(),
      .count = rigid_body_count,
      .is_data_array_of_pointers = false,
      .flags = {},
  };

  // BUILDING TLAS
  tlas_build_info = {
      .flags = daxa::AccelerationStructureBuildFlagBits::PREFER_FAST_BUILD,
      .dst_tlas = {},
      .instances = tlas_info,
      .scratch_data = device.device_address(proc_tlas_scratch_buffer).value(),
  };

  // Get the build sizes and verify they fit the fixed AVERAGE_AS_SIZE TLAS/scratch buffers (B4)
  tlas_build_sizes = device.tlas_build_sizes(tlas_build_info);
  if (!tlas_within_budget("build_accel_structs"))
  {
    return false;
  }

  // Set the scratch offset
  tlas_build_info.scratch_data = device.device_address(proc_tlas_scratch_buffer).value();

  // TODO: Create TLAS buffer from buffer
  // // Create TLAS buffer from buffer
  // tlas = device.create_tlas_from_buffer(
  //     {
  //       {
  //         .size = tlas_build_sizes.acceleration_structure_size,
  //         .name = "tlas",
  //       }
  //     }
  // );

  // Set the TLAS buffer
  tlas_build_info.dst_tlas = tlas[renderer_manager->get_sim_frame_index()];

  // Set Task TLAS
  task_tlas.set_tlas(tlas[renderer_manager->get_sim_frame_index()]);

  // INCREMENTAL-AS: capture this full build as the baseline the next fracture diffs against.
  seed_incremental_state(rigid_bodies, primitives);

  return true;
}

// Record the just-completed full build as the incremental baseline: body i's BLAS is proc_blas[i],
// its buffer region is the bump slice [start, start+aligned_size), and its prim-span content hash
// is the diff key. proc_blas_buffer_offset is the dense bump end, so seed the region pool's
// high_water to it with no holes (fully packed). After this, update_accel_structs_incremental can
// rebuild only the bodies whose hash changes.
void AccelerationStructureManager::seed_incremental_state(std::vector<RigidBody> const &rigid_bodies,
                                                          std::vector<Aabb> const &primitives)
{
  u32 const n = static_cast<u32>(rigid_bodies.size());
  if (body_blas_.size() < n) { body_blas_.resize(n); }
  if (body_blas_region_.size() < n) { body_blas_region_.resize(n); }
  if (body_built_hash_.size() < n) { body_built_hash_.resize(n); }

  auto get_aligned = [](u64 operand, u64 granularity) -> u64
  { return ((operand + (granularity - 1)) & ~(granularity - 1)); };

  u64 off = 0;
  for (u32 i = 0; i < n; ++i)
  {
    u64 const sz = get_aligned(blas_build_sizes.at(i).acceleration_structure_size, ACCELERATION_STRUCTURE_BUILD_OFFSET_ALIGMENT);
    body_blas_[i] = proc_blas[i];
    body_blas_region_[i] = {off, sz};
    body_built_hash_[i] = hash_prim_span(primitives, rigid_bodies[i].primitive_offset, rigid_bodies[i].primitive_count);
    off += sz;
  }
  // any slot beyond the current scene must not carry a stale handle/region into the next diff
  for (u32 i = n; i < body_blas_.size(); ++i) { body_blas_[i] = {}; body_blas_region_[i] = {0, 0}; body_built_hash_[i] = 0; }

  // seed the region pool as fully packed up to the bump end (matches proc_blas_buffer_offset)
  blas_region_pool_.reset();
  blas_region_pool_.high_water = static_cast<daxa_u32>(off);
  blas_region_pool_.live_bytes = static_cast<daxa_u32>(off);
  incremental_ready_ = true;
}

// INCREMENTAL AS. Lays out prims densely and re-uploads them in full (cheap) exactly like the full
// build, then (re)builds ONLY the BLAS whose prim-span content hash changed since the last build,
// keeping every unchanged body's baked BLAS in place. Reuses AS_build_TG (its BLAS task builds
// whatever is in blas_build_infos, so filling it with just the dirty subset builds just those) and
// the per-frame TLAS instance/transform refresh. The dominant respawn cost (build all N BLAS)
// collapses to building the handful of changed bodies. See the header member note for why this is
// correct (BLAS bakes geometry; the shader re-reads aabbs[primitive_offset+i] from the re-uploaded
// identical content). Falls back to a full build until one has seeded the baseline.
bool AccelerationStructureManager::update_accel_structs_incremental(std::vector<RigidBody> &rigid_bodies,
                                                                    std::vector<Aabb> const &primitives,
                                                                    std::function<void(daxa::BufferId)> const &post_primitive_upload, std::span<daxa_u32 const> changed_bodies)
{
  if (!initialized)
  {
    std::cerr << "ERROR: AccelerationStructureManager is not initialized inside update_accel_structs_incremental!" << std::endl;
    return false;
  }
  // No baseline yet (fresh load / post-reset) -> do a full build, which seeds it.
  if (!incremental_ready_)
  {
    return build_accel_structs(rigid_bodies, primitives, post_primitive_upload);
  }

  u32 const rigid_body_count = static_cast<u32>(rigid_bodies.size());
  u32 const primitive_count = static_cast<u32>(primitives.size());
  if (rigid_body_count > MAX_RIGID_BODY_COUNT || primitive_count > MAX_PRIMITIVE_COUNT)
  {
    std::cerr << "ERROR: incremental AS exceeded max rigid bodies (" << rigid_body_count << "/" << MAX_RIGID_BODY_COUNT
              << ") or primitives (" << primitive_count << "/" << MAX_PRIMITIVE_COUNT << ")!" << std::endl;
    return false;
  }
  if (body_blas_.size() < rigid_body_count) { body_blas_.resize(rigid_body_count); }
  if (body_blas_region_.size() < rigid_body_count) { body_blas_region_.resize(rigid_body_count); }
  if (body_built_hash_.size() < rigid_body_count) { body_built_hash_.resize(rigid_body_count); }

  auto get_aligned = [](u64 operand, u64 granularity) -> u64
  { return ((operand + (granularity - 1)) & ~(granularity - 1)); };

  // 1. UNIT-QUATERNION INVARIANT (identical to the full build; the TLAS instance transform and the
  //    ray tracer's quaternion sandwich both rely on |q| == 1).
  for (size_t i = 0; i < rigid_bodies.size(); ++i)
  {
    auto &rb = rigid_bodies[i];
    daxa_f32 m2 = rb.rotation.v.x * rb.rotation.v.x + rb.rotation.v.y * rb.rotation.v.y +
                  rb.rotation.v.z * rb.rotation.v.z + rb.rotation.w * rb.rotation.w;
    rb.rotation = (m2 > 1.0e-12f) ? rb.rotation.normalize() : Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
  }

  // 2. DENSE PRIM LAYOUT + FULL RE-UPLOAD (exactly the full build's model): copy every prim to the
  //    scratch, run the GPU voxel-prims hook, and assign each body a sequential primitive_offset.
  //    AS_build_TG's copy task then blits [0, primitive_scratch_offset) into primitive_buffer at
  //    previous_primitive_count(0). Cheap; keeps unchanged bodies' content consistent at their
  //    (possibly shifted) offsets so their baked BLAS still intersects correctly.
  std::memcpy(device.buffer_host_address_as<Aabb>(primitive_scratch_buffer).value(), primitives.data(), primitive_count * sizeof(Aabb));
  if (post_primitive_upload) { post_primitive_upload(primitive_scratch_buffer); }

  previous_primitive_count = 0;
  previous_rigid_body_count = 0;
  u32 acc = 0;
  for (u32 i = 0; i < rigid_body_count; ++i)
  {
    rigid_bodies[i].primitive_offset = acc;
    acc += rigid_bodies[i].primitive_count;
  }
  current_primitive_count = acc;
  current_rigid_body_count = rigid_body_count;
  primitive_scratch_offset = primitive_count * sizeof(Aabb);
  rigid_body_scratch_offset = static_cast<u64>(rigid_body_count) * sizeof(RigidBody);

  // 3. BLAS: (re)build only the dirty bodies; keep the rest. blas_geometries is resized upfront and
  //    indexed by BODY id so the Spans stashed in blas_build_infos never dangle (no vector realloc).
  blas_build_infos.clear();
  blas_build_infos.reserve(rigid_body_count);
  blas_build_sizes.clear();
  blas_build_sizes.reserve(rigid_body_count);
  blas_geometries.clear();
  blas_geometries.resize(rigid_body_count);
  proc_blas_scratch_offset = 0;

  // Host voxel AABBs are placeholders, so their hash cannot detect changed occupancy.
  std::vector<bool> geometry_changed(rigid_body_count, false);
  for (daxa_u32 id : changed_bodies) if (id < rigid_body_count) geometry_changed[id] = true;
  u32 dirty_count = 0;
  for (u32 i = 0; i < rigid_body_count; ++i)
  {
    auto &rigid_body = rigid_bodies[i];
    u64 const hash = hash_prim_span(primitives, rigid_body.primitive_offset, rigid_body.primitive_count);
    bool const dirty = geometry_changed[i] || body_blas_[i].is_empty() || hash != body_built_hash_[i];

    if (dirty)
    {
      // free the stale BLAS + its buffer region (coalesces back into the pool for reuse)
      if (!body_blas_[i].is_empty())
      {
        device.destroy_blas(body_blas_[i]);
        blas_region_pool_.free(static_cast<daxa_u32>(body_blas_region_[i].first), static_cast<daxa_u32>(body_blas_region_[i].second));
        body_blas_[i] = {};
      }

      blas_geometries.at(i).push_back({
          .data = device.device_address(primitive_buffer).value() + static_cast<u64>(rigid_body.primitive_offset) * sizeof(Aabb),
          .stride = sizeof(Aabb),
          .count = rigid_body.primitive_count,
          .flags = daxa::GeometryFlagBits::OPAQUE,
      });
      blas_build_infos.push_back({
          .flags = daxa::AccelerationStructureBuildFlagBits::PREFER_FAST_TRACE,
          .dst_blas = {},
          .geometries = daxa::Span<const daxa::BlasAabbGeometryInfo>(blas_geometries.at(i).data(), blas_geometries.at(i).size()),
          .scratch_data = {},
      });
      blas_build_sizes.push_back(device.blas_build_sizes(blas_build_infos.back()));

      // scratch region for this build (reset to 0 above; each dirty build bumps a fresh slice)
      auto scratch_offset = get_aligned(blas_build_sizes.back().build_scratch_size, acceleration_structure_scratch_offset_alignment);
      if (proc_blas_scratch_offset + scratch_offset > BLAS_POOL_BUDGET)
      {
        std::cerr << "ERROR: incremental AS exceeded BLAS scratch budget! Current: " << (proc_blas_scratch_offset + scratch_offset)
                  << ", Limit: " << BLAS_POOL_BUDGET << std::endl;
        return false;
      }
      blas_build_infos.back().scratch_data = device.device_address(proc_blas_scratch_buffer).value() + proc_blas_scratch_offset;
      proc_blas_scratch_offset += scratch_offset;

      // persistent buffer region from the free-list (256-aligned size keeps every offset aligned)
      auto const as_size = blas_build_sizes.back().acceleration_structure_size;
      auto const aligned = get_aligned(as_size, ACCELERATION_STRUCTURE_BUILD_OFFSET_ALIGMENT);
      bool ok = false;
      daxa_u32 const region_off = blas_region_pool_.alloc(static_cast<daxa_u32>(aligned), BLAS_POOL_BUDGET, ok);
      if (!ok)
      {
        std::cerr << "ERROR: incremental AS exhausted the BLAS buffer pool (need " << aligned
                  << ", largest free " << blas_region_pool_.largest_free_block() << ", limit " << BLAS_POOL_BUDGET << ")!" << std::endl;
        return false;
      }
      body_blas_region_[i] = {region_off, aligned};
      body_blas_[i] = device.create_blas_from_buffer(
          {{.size = as_size, .name = "blas" + std::to_string(i)}, proc_blas_buffer, region_off});
      blas_build_infos.back().dst_blas = body_blas_[i];
      body_built_hash_[i] = hash;
      ++dirty_count;
    }

    // instance for EVERY body (dirty or not): transform is refreshed per-frame by the TLAS update
    // pass, but seed it here; the BLAS address is this body's current (kept or rebuilt) handle.
    blas_instances_data[i] = {
        .transform = rigid_body.get_instance_transform(),
        .instance_custom_index = i,
        .mask = 0xFF,
        .instance_shader_binding_table_record_offset = 0,
        .flags = {},
        .blas_device_address = device.device_address(body_blas_[i]).value(),
    };
  }

  // copy rigid bodies to the scratch (AS_build_TG blits them into the rigid-body buffer)
  std::memcpy(device.buffer_host_address_as<RigidBody>(rigid_body_scratch_buffer).value(), rigid_bodies.data(), rigid_body_count * sizeof(RigidBody));

  // bind the BLAS task to a live handle (any non-empty body BLAS; placeholder only if empty scene)
  {
    daxa::BlasId bound = placeholder_blas;
    for (u32 i = 0; i < rigid_body_count; ++i) { if (!body_blas_[i].is_empty()) { bound = body_blas_[i]; break; } }
    task_blas.set_blas(bound);
  }

  // TLAS over all instances (same as the full build)
  tlas_info[0] = {
      .data = device.device_address(blas_instances_buffer).value(),
      .count = rigid_body_count,
      .is_data_array_of_pointers = false,
      .flags = {},
  };
  tlas_build_info = {
      .flags = daxa::AccelerationStructureBuildFlagBits::PREFER_FAST_BUILD,
      .dst_tlas = {},
      .instances = tlas_info,
      .scratch_data = device.device_address(proc_tlas_scratch_buffer).value(),
  };
  tlas_build_sizes = device.tlas_build_sizes(tlas_build_info);
  if (!tlas_within_budget("update_accel_structs_incremental")) { return false; }
  tlas_build_info.scratch_data = device.device_address(proc_tlas_scratch_buffer).value();
  tlas_build_info.dst_tlas = tlas[renderer_manager->get_sim_frame_index()];
  task_tlas.set_tlas(tlas[renderer_manager->get_sim_frame_index()]);

  // BLAS-region invariant (gated by BB_POOL_VERIFY, same switch the fracture pools use): the pool's
  // free ranges must stay sorted/coalesced and account exactly (Sigma free + live == high_water). A
  // violation means a region overlap -> a rebuilt BLAS could stomp a live one -> device-lost, so
  // surface it loudly here rather than as a GPU crash later. Also assert no live region exceeds the
  // buffer budget.
  if (std::getenv("BB_POOL_VERIFY"))
  {
    std::string const err = blas_region_pool_.verify_free();
    if (!err.empty())
    {
      std::cerr << "ERROR: [BB_POOL_VERIFY] blas_region_pool_ invariant broken after incremental AS: " << err << std::endl;
      std::abort();
    }
    if (blas_region_pool_.high_water > BLAS_POOL_BUDGET)
    {
      std::cerr << "ERROR: [BB_POOL_VERIFY] blas_region_pool_ high_water " << blas_region_pool_.high_water
                << " exceeds budget " << BLAS_POOL_BUDGET << std::endl;
      std::abort();
    }
  }

  if (std::getenv("BB_RESPAWN_TIMING")) { std::cout << "[INCR-AS] dirty=" << dirty_count << "/" << rigid_body_count
                                                    << " blas_pool_live=" << blas_region_pool_.live_bytes
                                                    << " hw=" << blas_region_pool_.high_water << std::endl; }
  return true;
}

void AccelerationStructureManager::update_TLAS()
{
  if(!initialized) {
    return;
  }
  update_buffers();
  if (!update()) // B4: was discarded — a failed AS rebuild (size guard / offset overflow) must not proceed
  {
    std::cerr << "ERROR: update_TLAS aborted: the TLAS/BLAS rebuild failed (see the error above)." << std::endl;
    return;
  }
  TLAS_update_TG.execute();
  device.wait_idle();
  // timeline signal values must be strictly increasing: bump right before the signaling submit
  task_manager->gpu->advance_sim_timeline();
  TLAS_build_TG.execute();
}

bool AccelerationStructureManager::update()
{
  if (!initialized)
  {
    return false;
  }

  daxa_u32 frame_index = renderer_manager->get_sim_frame_index();

  // BUILDING BLAS
  auto clear_build_AS = [&](u32 count)
  {
    blas_build_infos.clear();
    blas_build_infos.reserve(count);
    blas_build_sizes.clear();
    blas_build_sizes.reserve(count);
    blas_geometries.clear();
    blas_geometries.resize(count);
  };

  // TODO: one BLAS for the LBVH
  clear_build_AS(1);

  proc_blas_scratch_offset = 0;

  /// Alignments:
  auto get_aligned = [&](u64 operand, u64 granularity) -> u64
  {
    return ((operand + (granularity - 1)) & ~(granularity - 1));
  };

  daxa_u32 total_instances = current_rigid_body_count;

  // Generate LBVH BLAS
  if(renderer_manager->is_bvh_enabled() && current_rigid_body_count > 0u)
  {
    // The broad-phase tree has one leaf per BODY, not per render primitive.
    // Voxel bodies contain many primitives; using their count reads unbuilt
    // nodes and can exceed the MAX_LBVH_NODE_COUNT buffer when L is enabled.
    u32 const lbvh_primitive_count = 2u * current_rigid_body_count - 1u;

    blas_geometries.at(0).push_back({
        .data = device.device_address(rigid_body_manager->get_lbvh_node_buffer()).value(),
        .stride = sizeof(LBVHNode),
        .count = lbvh_primitive_count,
        .flags = daxa::GeometryFlagBits::NO_DUPLICATE_ANY_HIT_INVOCATION,
    });

    // Create BLAS build info
    blas_build_infos.push_back({
        .flags = daxa::AccelerationStructureBuildFlagBits::PREFER_FAST_BUILD,
        .dst_blas = {},
        .geometries = daxa::Span<const daxa::BlasAabbGeometryInfo>(blas_geometries.at(0).data(), blas_geometries.at(0).size()),
        .scratch_data = {},
    });
    // Get the build sizes
    blas_build_sizes.push_back(device.blas_build_sizes(blas_build_infos.back()));

    auto scratch_offset = get_aligned(blas_build_sizes.back().build_scratch_size, acceleration_structure_scratch_offset_alignment);

    if (proc_blas_scratch_offset + scratch_offset > BLAS_POOL_BUDGET)
    {
      // B4: mirror the descriptive message the build_accel_structs guard emits (was silent here)
      std::cerr << "ERROR: Exceeded BLAS scratch offset limit (LBVH build)! Current: " << (proc_blas_scratch_offset + scratch_offset)
                << ", Limit: " << (BLAS_POOL_BUDGET) << std::endl;
      clear_build_AS(0);
      return false;
    }

    // Set the scratch offset
    blas_build_infos.back().scratch_data = device.device_address(proc_blas_scratch_buffer).value() + proc_blas_scratch_offset;

    // Increment the scratch offset
    proc_blas_scratch_offset += scratch_offset;

    if(lbvh_blas[frame_index] != daxa::BlasId{})
    {
      device.destroy_blas(lbvh_blas[frame_index]);
    }

    // Create BLAS buffer from buffer
    lbvh_blas[frame_index] = device.create_blas({
        .size = blas_build_sizes.back().acceleration_structure_size,
        .name = "lbvh_blas",
    });

    // Add the BLAS buffer to the BLAS build info
    blas_build_infos.back().dst_blas = lbvh_blas[frame_index];

    blas_instances_data[current_rigid_body_count] = {
        .transform = daxa_f32mat3x4(daxa_f32vec4(1.0f, 0.0f, 0.0f, 0.0f),
                                     daxa_f32vec4(0.0f, 1.0f, 0.0f, 0.0f),
                                     daxa_f32vec4(0.0f, 0.0f, 1.0f, 0.0f)),
        .instance_custom_index = current_rigid_body_count,
        .mask = 0x2, // debug geometry must not occlude physical light rays
        .instance_shader_binding_table_record_offset = 1,
        .flags = {},
        .blas_device_address = device.device_address(lbvh_blas[frame_index]).value(),
    };
 
    // Set Task BLAS
    task_blas.set_blas(lbvh_blas[frame_index]);

    ++total_instances;
  }

  // Destroy TLAS
  if (!tlas[frame_index].is_empty())
  {
    device.destroy_tlas(tlas[frame_index]);
  }

  tlas[frame_index] = device.create_tlas({
      .size = AVERAGE_AS_SIZE,
      .name = "tlas_" + std::to_string(frame_index),
  });

  tlas_info[0] = {
      .data = device.device_address(blas_instances_buffer).value(),
      .count = total_instances,
      .is_data_array_of_pointers = false,
      .flags = {},
  };

  // BUILDING TLAS
  tlas_build_info = {
      .flags = daxa::AccelerationStructureBuildFlagBits::PREFER_FAST_BUILD,
      .dst_tlas = {},
      .instances = tlas_info,
      .scratch_data = device.device_address(proc_tlas_scratch_buffer).value(),
  };

  // Get the build sizes and verify they fit the fixed AVERAGE_AS_SIZE TLAS/scratch buffers (B4)
  tlas_build_sizes = device.tlas_build_sizes(tlas_build_info);
  if (!tlas_within_budget("update"))
  {
    return false;
  }

  // Set the scratch offset
  tlas_build_info.scratch_data = device.device_address(proc_tlas_scratch_buffer).value();

  // Set the TLAS buffer
  tlas_build_info.dst_tlas = tlas[frame_index];

  // Set Task TLAS
  task_tlas.set_tlas(tlas[frame_index]);

  return true;
}

bool AccelerationStructureManager::update_TLAS_resources(daxa::BufferId dispatch_buffer)
{
  if (!initialized)
  {
    // was `return !initialized` — i.e. TRUE (success) precisely when the manager was NOT ready,
    // so a caller running before create() silently "succeeded" with an unbound dispatch buffer
    return false;
  }

  task_dispatch_buffer.set_buffer(dispatch_buffer);

  return true;
}

void AccelerationStructureManager::record_accel_struct_tasks(TaskGraph &AS_TG)
{
  daxa::InlineTaskInfo task0({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, rigid_body_manager->task_rigid_bodies),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_aabb_buffer),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = primitive_scratch_buffer,
            .dst_buffer = ti.get(task_aabb_buffer).id,
            .dst_offset = previous_primitive_count * sizeof(Aabb),
            .size = primitive_scratch_offset,
        });

        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = rigid_body_scratch_buffer,
            .dst_buffer = ti.get(rigid_body_manager->task_rigid_bodies).id,
            .dst_offset = previous_rigid_body_count * sizeof(RigidBody),
            .size = rigid_body_scratch_offset,
        });
      },
      .name = "copy rigid bodies and primitives",
  });
  daxa::InlineTaskInfo task1({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_aabb_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_rigid_bodies),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, rigid_body_manager->task_next_rigid_bodies),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = ti.get(rigid_body_manager->task_rigid_bodies).id,
            .dst_buffer = ti.get(rigid_body_manager->task_next_rigid_bodies).id,
            .src_offset = previous_rigid_body_count * sizeof(RigidBody),
            .dst_offset = previous_rigid_body_count * sizeof(RigidBody),
            .size = rigid_body_scratch_offset,
        });

        primitive_scratch_offset = 0;
        rigid_body_scratch_offset = 0;
      },
      .name = "copy rigid bodies and primitives",
  });
  daxa::InlineTaskInfo task2({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_rigid_bodies),
          daxa::inl_attachment(daxa::TaskBufferAccess::BUILD_READ, task_aabb_buffer),
          daxa::inl_attachment(daxa::TaskBlasAccess::BUILD_WRITE, task_blas),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        // build blas
        ti.recorder.build_acceleration_structures({
            .blas_build_infos = blas_build_infos,
        });
      },
      .name = "blas build",
  });
  daxa::InlineTaskInfo task3({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::BUILD_READ, task_blas_instance_data),
          daxa::inl_attachment(daxa::TaskBlasAccess::BUILD_READ, task_blas),
          daxa::inl_attachment(daxa::TaskTlasAccess::BUILD_WRITE, task_tlas),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        // build tlas
        ti.recorder.build_acceleration_structures({
            .tlas_build_infos = std::array{tlas_build_info},
        });
      },
      .name = "tlas build",
  });

  std::array<daxa::InlineTaskInfo, 4> tasks = {
      task0,
      task1,
      task2,
      task3,
  };

  std::array<daxa::TaskBuffer, 4> buffers = {
      rigid_body_manager->task_rigid_bodies,
      rigid_body_manager->task_next_rigid_bodies,
      task_aabb_buffer,
      task_blas_instance_data,
  };
  std::array<daxa::TaskBlas, 1> blas = {
      task_blas,
  };
  std::array<daxa::TaskTlas, 1> task_tlases = {
      task_tlas,
  };

  AS_TG = task_manager->create_task_graph("Build Acceleration Structures", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, std::span<daxa::TaskBlas>(blas), std::span<daxa::TaskTlas>(task_tlases));
}

void AccelerationStructureManager::record_update_TLAS_tasks(TaskGraph &instances_TG, TaskGraph &build_TG, std::shared_ptr<daxa::ComputePipeline> update_AS_pipeline)
{
  auto user_callback_UI = [update_AS_pipeline](daxa::TaskInterface ti, auto &)
  {
    ti.recorder.set_pipeline(*update_AS_pipeline);
    ti.recorder.push_constant(UpdateInstancesPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(UpdateInstancesTaskHead::AT.dispatch_buffer).id,
                                   .offset = 0});
  };

  using TTaskUI = TaskTemplate<UpdateInstancesTaskHead::Task, decltype(user_callback_UI)>;

  // Instantiate the task using the template class
  TTaskUI task_UI(std::array{
                      daxa::attachment_view(UpdateInstancesTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
                      daxa::attachment_view(UpdateInstancesTaskHead::AT.sim_config, rigid_body_manager->task_sim_config),
                      daxa::attachment_view(UpdateInstancesTaskHead::AT.blas_instance_data, task_blas_instance_data),
                      daxa::attachment_view(UpdateInstancesTaskHead::AT.rigid_body_map, rigid_body_manager->task_rigid_body_entries),
                      daxa::attachment_view(UpdateInstancesTaskHead::AT.rigid_bodies, rigid_body_manager->task_rigid_bodies),
                      daxa::attachment_view(UpdateInstancesTaskHead::AT.aabbs, task_aabb_buffer),
                  },
                  user_callback_UI);

  daxa::InlineTaskInfo task_BB({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_rigid_bodies),
          daxa::inl_attachment(daxa::TaskBufferAccess::BUILD_READ, task_aabb_buffer),
          daxa::inl_attachment(daxa::TaskBlasAccess::BUILD_WRITE, task_blas),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        if(blas_build_infos.size() > 0) {
          // build blas
          ti.recorder.build_acceleration_structures({
              .blas_build_infos = blas_build_infos,
          });
        }
      },
      .name = "blas rebuild",
  });

  daxa::InlineTaskInfo task_BT({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_rigid_bodies),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_aabb_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::BUILD_READ, task_blas_instance_data),
          daxa::inl_attachment(daxa::TaskBlasAccess::BUILD_READ, task_blas),
          daxa::inl_attachment(daxa::TaskTlasAccess::BUILD_WRITE, task_tlas),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        // build tlas
        ti.recorder.build_acceleration_structures({
            .tlas_build_infos = std::array{tlas_build_info},
        });
      },
      .name = "tlas update",
  });

  std::array<daxa::TaskBuffer, 6> instance_buffers = {
      task_dispatch_buffer,
      rigid_body_manager->task_sim_config,
      task_blas_instance_data,
      rigid_body_manager->task_rigid_body_entries,
      rigid_body_manager->task_rigid_bodies,
      task_aabb_buffer,
  };
  // TLAS instance update + build run on the async compute queue, after the sim (same-queue FIFO)
  instances_TG = task_manager->create_task_graph("Update TLAS Instances", std::span<daxa::TaskBuffer>(instance_buffers), {}, {}, {}, false, daxa::QUEUE_COMPUTE_0);
  instances_TG.add_task(task_UI);

  std::array<daxa::TaskBuffer, 3> build_buffers = {
      rigid_body_manager->task_rigid_bodies,
      task_aabb_buffer,
      task_blas_instance_data,
  };
  std::array<daxa::TaskBlas, 1> blas = {
      task_blas,
  };
  std::array<daxa::TaskTlas, 1> task_tlases = {
      task_tlas,
  };

  build_TG = task_manager->create_task_graph("Build TLAS", std::span<daxa::TaskBuffer>(build_buffers), {}, std::span<daxa::TaskBlas>(blas), std::span<daxa::TaskTlas>(task_tlases), false, daxa::QUEUE_COMPUTE_0);
  build_TG.add_task(task_BB);
  build_TG.add_task(task_BT);
}

void AccelerationStructureManager::update_buffers()
{
  if (task_blas_instance_data.id() != blas_instances_buffer) { task_blas_instance_data.set_buffer(blas_instances_buffer); }
  if (task_aabb_buffer.id() != primitive_buffer) { task_aabb_buffer.set_buffer(primitive_buffer); }
}

void AccelerationStructureManager::update_AS_buffers() {
  if(!initialized) {
    return;
  }
  update_buffers();
  AS_update_buffers_TG.execute();
}

void AccelerationStructureManager::record_update_AS_buffers_tasks(TaskGraph &AS_buffers_TG)
{
 daxa::InlineTaskInfo task0({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_sim_config_host),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_sim_config),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, rigid_body_manager->task_old_sim_config),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_rigid_bodies),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, rigid_body_manager->task_previous_rigid_bodies),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_rigid_body_entries),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, rigid_body_manager->task_previous_rigid_body_entries),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_lbvh_nodes),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, rigid_body_manager->task_previous_lbvh_nodes),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_islands),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, rigid_body_manager->task_previous_islands),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_contact_islands),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, rigid_body_manager->task_previous_contact_islands),
          // contact warm-start state (the three "previous" buffers narrow_phase reads on resume)
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_collisions),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, rigid_body_manager->task_old_collisions),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_collision_entries),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, rigid_body_manager->task_collision_entries_previous),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_rigid_body_link_manifolds),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, rigid_body_manager->task_previous_rigid_body_link_manifolds),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, gui_manager->task_vertex_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, gui_manager->task_previous_vertex_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, gui_manager->task_line_vertex_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, gui_manager->task_previous_line_vertex_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, gui_manager->task_axes_vertex_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, gui_manager->task_previous_axes_vertex_buffer),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        auto sim_config = device.buffer_host_address_as<SimConfig>(ti.get(rigid_body_manager->task_sim_config_host).id).value();
        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = ti.get(rigid_body_manager->task_sim_config).id,
            .dst_buffer = ti.get(rigid_body_manager->task_old_sim_config).id,
            .size = sizeof(SimConfig),
        });

        auto rigid_body_count = renderer_manager->get_rigid_body_count();
        if(rigid_body_count > MAX_RIGID_BODY_COUNT) {
          rigid_body_count = MAX_RIGID_BODY_COUNT;
        }

        if(rigid_body_count > 0) {
          ti.recorder.copy_buffer_to_buffer({
              .src_buffer = ti.get(rigid_body_manager->task_rigid_bodies).id,
              .dst_buffer = ti.get(rigid_body_manager->task_previous_rigid_bodies).id,
              .size = rigid_body_count * sizeof(RigidBody),
          });

          ti.recorder.copy_buffer_to_buffer({
              .src_buffer = ti.get(rigid_body_manager->task_rigid_body_entries).id,
              .dst_buffer = ti.get(rigid_body_manager->task_previous_rigid_body_entries).id,
              .size = rigid_body_count * sizeof(RigidBodyEntry),
          });

          auto lbvh_node_count = rigid_body_count * 2 - 1;
          if(lbvh_node_count > MAX_LBVH_NODE_COUNT) {
            lbvh_node_count = MAX_LBVH_NODE_COUNT;
          }
          ti.recorder.copy_buffer_to_buffer({
              .src_buffer = ti.get(rigid_body_manager->task_lbvh_nodes).id,
              .dst_buffer = ti.get(rigid_body_manager->task_previous_lbvh_nodes).id,
              .size = lbvh_node_count * sizeof(LBVHNode),
          });

          ti.recorder.copy_buffer_to_buffer({
              .src_buffer = ti.get(rigid_body_manager->task_islands).id,
              .dst_buffer = ti.get(rigid_body_manager->task_previous_islands).id,
              .size = rigid_body_count * sizeof(Island),
          });

          ti.recorder.copy_buffer_to_buffer({
              .src_buffer = ti.get(rigid_body_manager->task_contact_islands).id,
              .dst_buffer = ti.get(rigid_body_manager->task_previous_contact_islands).id,
              .size = rigid_body_count * sizeof(ContactIsland),
          });
        }

        // Coherent pause flush of the contact warm-start state. A pause copies the body double
        // buffer current->previous so the static frame renders right, but narrow_phase warm-starts
        // the first resumed step by walking the PREVIOUS body's manifold-node chain into these
        // three "previous" contact buffers. Without flushing them too, the resumed warm-start reads
        // a stale node pool -> garbage lambda/anchors -> the penalty solver explodes. Copy the used
        // prefix of each (collision_map[id] is bounded by collision_count; nodes by node_count).
        auto collision_count = sim_config->g_c_info.collision_count > MAX_COLLISION_COUNT
                                 ? MAX_COLLISION_COUNT : sim_config->g_c_info.collision_count;
        if(collision_count > 0) {
          ti.recorder.copy_buffer_to_buffer({
              .src_buffer = ti.get(rigid_body_manager->task_collisions).id,
              .dst_buffer = ti.get(rigid_body_manager->task_old_collisions).id,
              .size = collision_count * sizeof(Manifold),
          });
          ti.recorder.copy_buffer_to_buffer({
              .src_buffer = ti.get(rigid_body_manager->task_collision_entries).id,
              .dst_buffer = ti.get(rigid_body_manager->task_collision_entries_previous).id,
              .size = collision_count * sizeof(CollisionEntry),
          });
        }
        auto manifold_node_count = sim_config->manifold_node_count > BB_MAX_MANIFOLD_NODE_COUNT
                                     ? BB_MAX_MANIFOLD_NODE_COUNT : sim_config->manifold_node_count;
        if(manifold_node_count > 0) {
          ti.recorder.copy_buffer_to_buffer({
              .src_buffer = ti.get(rigid_body_manager->task_rigid_body_link_manifolds).id,
              .dst_buffer = ti.get(rigid_body_manager->task_previous_rigid_body_link_manifolds).id,
              .size = manifold_node_count * sizeof(ManifoldNode),
          });
        }

        auto point_count = sim_config->g_c_info.collision_point_count > BB_MAX_DEBUG_CONTACT_POINT_COUNT
                             ? BB_MAX_DEBUG_CONTACT_POINT_COUNT
                             : sim_config->g_c_info.collision_point_count;
        if(renderer_manager->is_gui_enabled()) {
          if(point_count > 0) {
            ti.recorder.copy_buffer_to_buffer({
                .src_buffer = ti.get(gui_manager->task_vertex_buffer).id,
                .dst_buffer = ti.get(gui_manager->task_previous_vertex_buffer).id,
                .size = point_count * sizeof(daxa_f32vec3),
            });
          }

          auto line_count = point_count * 2;
          if(line_count > 0) {
            ti.recorder.copy_buffer_to_buffer({
                .src_buffer = ti.get(gui_manager->task_line_vertex_buffer).id,
                .dst_buffer = ti.get(gui_manager->task_previous_line_vertex_buffer).id,
                .size = line_count * sizeof(daxa_f32vec3),
            });
          }
          auto axes_count = sim_config->rigid_body_count * 6;
          axes_count = axes_count > MAX_AXIS_COUNT ? MAX_AXIS_COUNT : axes_count;
          if(axes_count > 0) {
            ti.recorder.copy_buffer_to_buffer({
                .src_buffer = ti.get(gui_manager->task_axes_vertex_buffer).id,
                .dst_buffer = ti.get(gui_manager->task_previous_axes_vertex_buffer).id,
                .size = axes_count * sizeof(daxa_f32vec3),
            });
          }
        }
      },
      .name = "copy points",
  });

  std::array<daxa::InlineTaskInfo, 1> tasks = {
    task0,
  };

  std::array<daxa::TaskBuffer, 25> buffers = {
    rigid_body_manager->task_sim_config_host,
    rigid_body_manager->task_rigid_bodies,
    rigid_body_manager->task_previous_rigid_bodies,
    rigid_body_manager->task_rigid_body_entries,
    rigid_body_manager->task_previous_rigid_body_entries,
    rigid_body_manager->task_lbvh_nodes,
    rigid_body_manager->task_previous_lbvh_nodes,
    rigid_body_manager->task_islands,
    rigid_body_manager->task_previous_islands,
    rigid_body_manager->task_contact_islands,
    rigid_body_manager->task_previous_contact_islands,
    gui_manager->task_previous_vertex_buffer,
    gui_manager->task_vertex_buffer,
    rigid_body_manager->task_old_sim_config,
    rigid_body_manager->task_sim_config,
    gui_manager->task_previous_line_vertex_buffer,
    gui_manager->task_line_vertex_buffer,
    gui_manager->task_previous_axes_vertex_buffer,
    gui_manager->task_axes_vertex_buffer,
    // contact warm-start buffers for the coherent pause flush
    rigid_body_manager->task_collisions,
    rigid_body_manager->task_old_collisions,
    rigid_body_manager->task_collision_entries,
    rigid_body_manager->task_collision_entries_previous,
    rigid_body_manager->task_rigid_body_link_manifolds,
    rigid_body_manager->task_previous_rigid_body_link_manifolds,
  };

  AS_buffers_TG = task_manager->create_task_graph("Update Acceleration Structure Buffers", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, {}, {}, false, daxa::QUEUE_COMPUTE_0);
}

BB_NAMESPACE_END
