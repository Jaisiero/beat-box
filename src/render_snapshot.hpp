#pragma once
#include "task_manager.hpp"

BB_NAMESPACE_BEGIN

// COMPUTE copies completed states into free slots. MAIN reads a selected slot
// through separate aliases and waits for that copy's exact submit, never for a
// later solve. A slot cannot be reused until its last MAIN reader has completed.
struct RenderSnapshot
{
  static constexpr size_t COUNT = 9, SLOT_COUNT = 3;
  enum Index : size_t { BODIES, BODY_MAP, BVH, ISLANDS, CONTACT_ISLANDS, POINTS, LINES, AXES, CONFIG };
  struct Slot {
    std::array<daxa::TaskBuffer, COUNT> buffers;
    TaskGraph graph;
    SimConfig config = {};
    daxa_u64 ready = 0, last_reader = 0, generation = 0;
    bool valid = false, debug_valid = false;
  };
  std::array<Slot, SLOT_COUNT> slots;
  std::array<daxa::TaskBuffer, COUNT> buffers, sources;
  std::array<daxa_u64, COUNT> copy_sizes = {};
  size_t selected = SLOT_COUNT;
  daxa_u64 next_generation = 0, selected_generation = 0;
  bool allocated = false;

  void create(TaskManager &manager, std::array<daxa::TaskBuffer, COUNT> source_buffers)
  {
    sources = source_buffers;
    for (size_t i = 0; i < COUNT; ++i)
      buffers[i] = daxa::TaskBuffer({.name = "Render snapshot view " + std::to_string(i)});
    for (size_t slot = 0; slot < SLOT_COUNT; ++slot) {
      std::vector<daxa::TaskBuffer> resources;
      for (size_t i = 0; i < COUNT; ++i) {
        slots[slot].buffers[i] = daxa::TaskBuffer({.name = "Snapshot " + std::to_string(slot) + " buffer " + std::to_string(i)});
        resources.push_back(sources[i]);
        resources.push_back(slots[slot].buffers[i]);
      }
      auto &graph = slots[slot].graph;
      graph = manager.create_task_graph(("Capture snapshot " + std::to_string(slot)).c_str(), std::span<daxa::TaskBuffer>(resources), {}, {}, {}, false, daxa::QUEUE_COMPUTE_0);
      for (size_t i = 0; i < COUNT; ++i) {
        graph.add_task(daxa::InlineTaskInfo{
          .attachments = {
            daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, sources[i]),
            daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, slots[slot].buffers[i]),
          },
          .task = [this, slot, i](daxa::TaskInterface const &ti) {
            if (copy_sizes[i] == 0) return;
            ti.recorder.copy_buffer_to_buffer({.src_buffer = ti.get(sources[i]).id,
                .dst_buffer = ti.get(slots[slot].buffers[i]).id, .size = copy_sizes[i]});
          },
          .name = "Snapshot copy " + std::to_string(i),
        });
      }
      graph.submit(); graph.complete();
    }
  }

  // Geometry/primitive offsets changed. No old pose snapshot can describe the
  // new geometry. Call only after the existing topology retirement boundary.
  void invalidate()
  {
    for (auto &slot : slots) slot.valid = false;
    selected = SLOT_COUNT;
    selected_generation = 0;
  }

  bool publish(daxa::Device &device, SimConfig config, bool debug_valid,
               std::array<daxa_u64, COUNT> sizes, daxa_u64 main_producer)
  {
    if (!allocated) {
      for (auto &slot : slots) for (size_t i = 0; i < COUNT; ++i)
        slot.buffers[i].set_buffer(device.create_buffer({
            .size = device.buffer_info(sources[i].id()).value().size, .name = "Render snapshot storage"}));
      allocated = true;
    }
    auto const oldest = device.oldest_pending_submit_index();
    size_t target = SLOT_COUNT;
    for (size_t i = 0; i < SLOT_COUNT; ++i) {
      if (i == selected || slots[i].last_reader >= oldest || slots[i].ready >= oldest) continue;
      if (target == SLOT_COUNT || slots[i].generation < slots[target].generation) target = i;
    }
    // Preserve physics progress if presentation temporarily holds every slot.
    // Topology publication first drains old readers, so it always has a slot.
    if (target == SLOT_COUNT) return false;
    auto &slot = slots[target];
    for (size_t i = 0; i < COUNT; ++i)
      if (sizes[i] > device.buffer_info(slot.buffers[i].id()).value().size)
        throw std::runtime_error("Render snapshot capacity exceeded");
    // Host publication can write sources on MAIN; ordinary poses are produced
    // on COMPUTE and are already ordered by the shared source task handles.
    if (main_producer != 0) device.submit_commands({.queue = daxa::QUEUE_COMPUTE_0,
        .wait_queue_submit_indices = std::array{std::pair{daxa::QUEUE_MAIN, main_producer}}});
    // Alias readers are outside the capture graph. This explicit wait supplies
    // their read->write execution dependency when recycling a previously drawn slot.
    if (slot.last_reader != 0) device.submit_commands({.queue = daxa::QUEUE_COMPUTE_0,
        .wait_queue_submit_indices = std::array{std::pair{daxa::QUEUE_MAIN, slot.last_reader}}});
    copy_sizes = sizes;
    slot.graph.execute();
    slot.ready = device.latest_queue_submit_index(daxa::QUEUE_COMPUTE_0);
    slot.config = config;
    slot.debug_valid = debug_valid;
    slot.generation = ++next_generation;
    slot.valid = true;
    return true;
  }

  // Normally select only completed copies. After a topology change no old
  // snapshot is usable; waiting for the new copy still never waits for a solve
  // submitted after it. Keep current aliases/history when reusing a snapshot.
  bool select(daxa::Device &device)
  {
    auto const oldest = device.oldest_pending_submit_index();
    size_t next = selected;
    for (size_t i = 0; i < SLOT_COUNT; ++i) {
      auto const &s = slots[i];
      if (!s.valid || (selected != SLOT_COUNT && s.ready >= oldest)) continue;
      if (next == SLOT_COUNT || s.generation > slots[next].generation) next = i;
    }
    if (next == SLOT_COUNT) return false;
    if (next != selected || selected_generation != slots[next].generation) {
      device.submit_commands({.queue = daxa::QUEUE_MAIN,
          .wait_queue_submit_indices = std::array{std::pair{daxa::QUEUE_COMPUTE_0, slots[next].ready}}});
      for (size_t i = 0; i < COUNT; ++i) buffers[i].set_buffer(slots[next].buffers[i].id());
      selected = next;
      selected_generation = slots[next].generation;
    }
    return true;
  }

  void mark_read(daxa::Device &device) { slots[selected].last_reader = device.latest_queue_submit_index(daxa::QUEUE_MAIN); }
  void destroy(daxa::Device &device)
  {
    if (allocated) for (auto &slot : slots) for (auto &buffer : slot.buffers) device.destroy_buffer(buffer.id());
    allocated = false;
  }
};

BB_NAMESPACE_END
