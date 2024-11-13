#include "acceleration_structure_manager.hpp"
#include "renderer_manager.hpp"
#include "gui_manager.hpp"

BB_NAMESPACE_BEGIN

AccelerationStructureManager::AccelerationStructureManager(daxa::Device &device, std::shared_ptr<TaskManager> task_manager) : device(device), task_manager(task_manager)
{
  if (device.is_valid())
  {
    acceleration_structure_scratch_offset_alignment = device.properties().acceleration_structure_properties.value().min_acceleration_structure_scratch_offset_alignment;

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
    renderer_manager = renderer;
    rigid_body_manager = rigid_body;
    gui_manager = gui;

    // Create buffer for RigidBodies
    rigid_body_scratch_buffer = device.create_buffer({
        .size = MAX_RIGID_BODY_COUNT * sizeof(RigidBody),
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
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
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
        .name = "primitive_scratch_buffer",
    });

    // Create buffer for primitives
    primitive_buffer = device.create_buffer({
        .size = MAX_PRIMITIVE_COUNT * sizeof(Aabb),
        .name = "primitive_buffer",
    });

    for (auto f = 0; f < DOUBLE_BUFFERING; ++f)
      points_buffer[f] = device.create_buffer({
          .size = MAX_PRIMITIVE_COUNT * sizeof(Aabb),
          .name = "points_buffer",
      });

    // Create BLAS buffer
    proc_blas_buffer = device.create_buffer({
        .size = AVERAGE_AS_SIZE * MAX_ACCELERATION_STRUCTURE_COUNT,
        .name = "proc_blas_buffer",
    });

    point_blas_buffer = device.create_buffer({
        .size = AVERAGE_AS_SIZE * MAX_ACCELERATION_STRUCTURE_COUNT,
        .name = "point_proc_blas_buffer",
    });

    // Create BLAS scratch buffer
    proc_blas_scratch_buffer = device.create_buffer({
        .size = AVERAGE_AS_SIZE * MAX_ACCELERATION_STRUCTURE_COUNT,
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

    // Create BLAS instances buffer
    blas_instances_buffer = device.create_buffer({
        .size = sizeof(daxa_BlasInstanceData) * MAX_ACCELERATION_STRUCTURE_COUNT,
        .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_RANDOM,
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
    task_rigid_body_buffer.set_buffers({.buffers = std::array{rigid_body_buffer[0]}});
    task_aabb_buffer.set_buffers({.buffers = std::array{primitive_buffer}});
    task_blas_instance_data.set_buffers({.buffers = std::array{blas_instances_buffer}});
    task_points_aabb_buffer.set_buffers({.buffers = std::array{points_buffer[0]}});
    task_previous_points_aabb_buffer.set_buffers({.buffers = std::array{points_buffer[1]}});

    record_accel_struct_tasks(AS_build_TG);
    AS_build_TG.submit();
    AS_build_TG.complete();

    record_update_TLAS_tasks(TLAS_update_TG, update_pipeline);
    TLAS_update_TG.submit();
    TLAS_update_TG.complete();

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
    for (auto f = 0; f < DOUBLE_BUFFERING; ++f)
      device.destroy_buffer(points_buffer[f]);
    for (auto blas : proc_blas)
    {
      device.destroy_blas(blas);
    }
    device.destroy_buffer(proc_blas_buffer);
    for (auto blas : point_blas)
    {
      device.destroy_blas(blas);
    }
    device.destroy_buffer(point_blas_buffer);
    device.destroy_buffer(proc_blas_scratch_buffer);
    device.destroy_buffer(proc_tlas_buffer);
    device.destroy_buffer(proc_tlas_scratch_buffer);
    device.destroy_buffer(blas_instances_buffer);
    for (auto f = 0; f < DOUBLE_BUFFERING; ++f)
      device.destroy_tlas(tlas[f]);
    initialized = false;
  }
}

void AccelerationStructureManager::free_accel_structs()
{
  // Freeing BLAS
  for (auto blas : proc_blas)
  {
    device.destroy_blas(blas);
  }

  // Freeing BLAS
  for (auto blas : point_blas)
  {
    device.destroy_blas(blas);
  }

  // Freeing TLAS

  for (auto f = 0; f < DOUBLE_BUFFERING; ++f)
    device.destroy_tlas(tlas[f]);

  // Resetting the offsets
  current_rigid_body_count = 0;
  previous_rigid_body_count = 0;
  current_primitive_count = 0;
  primitive_scratch_offset = 0;
  previous_primitive_count = 0;
  proc_blas_buffer_offset = 0;
  point_blas_buffer_offset = 0;
  proc_blas_scratch_offset = 0;
}

daxa::TlasId AccelerationStructureManager::get_tlas()
{
  if(!initialized) {
    return {};
  }
  return tlas[renderer_manager->get_frame_index()];
}

daxa::BufferId AccelerationStructureManager::get_rigid_body_buffer()
{
  if(!initialized) {
    return {};
  }
  return rigid_body_buffer[renderer_manager->get_frame_index()];
}

daxa::BufferId AccelerationStructureManager::get_next_rigid_body_buffer()
{
  if(!initialized) {
    return {};
  }

  return rigid_body_buffer[renderer_manager->get_next_frame_index()];
}

daxa::BufferId AccelerationStructureManager::get_points_buffer()
{
  if(!initialized) {
    return {};
  }
  return points_buffer[renderer_manager->get_frame_index()];
}

void AccelerationStructureManager::build_AS()
{
  if(!initialized) {
    return;
  }
  AS_build_TG.execute();
}

bool AccelerationStructureManager::build_accel_structs(std::vector<RigidBody> &rigid_bodies, std::vector<Aabb> const &primitives)
{
  if(!initialized) {
    return false;
  }
  // Get the number of rigid bodies and primitives
  auto rigid_body_count = static_cast<u32>(rigid_bodies.size());
  auto primitive_count = static_cast<u32>(primitives.size());

  // Check if the number of rigid bodies and primitives is within the limits
  if (current_rigid_body_count + rigid_body_count > MAX_RIGID_BODY_COUNT || current_primitive_count + primitive_count > MAX_PRIMITIVE_COUNT)
  {
    return false;
  }

  // Copy primitives to the buffer
  std::memcpy(device.buffer_host_address_as<Aabb>(primitive_scratch_buffer).value(), primitives.data(), primitive_count * sizeof(Aabb));

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

  /// Alignments:
  auto get_aligned = [&](u32 operand, u32 granularity) -> u32
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
        .flags = daxa::AccelerationStructureBuildFlagBits::PREFER_FAST_BUILD,
        .dst_blas = {},
        .geometries = daxa::Span<const daxa::BlasAabbGeometryInfo>(blas_geometries.at(i).data(), blas_geometries.at(i).size()),
        .scratch_data = {},
    });
    // Get the build sizes
    blas_build_sizes.push_back(device.blas_build_sizes(blas_build_infos.back()));

    auto scratch_offset = get_aligned(blas_build_sizes.back().build_scratch_size, acceleration_structure_scratch_offset_alignment);

    if (proc_blas_scratch_offset + scratch_offset > AVERAGE_AS_SIZE * MAX_ACCELERATION_STRUCTURE_COUNT)
    {
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
    if (proc_blas_buffer_offset + blas_instance_offset > AVERAGE_AS_SIZE * MAX_ACCELERATION_STRUCTURE_COUNT)
    {
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
  current_primitive_count += primitive_count;

  // Set Task BLAS
  task_blas.set_blas({.blas = proc_blas});

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

  // Get the build sizes
  tlas_build_sizes = device.tlas_build_sizes(tlas_build_info);

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
  tlas_build_info.dst_tlas = tlas[renderer_manager->get_frame_index()];

  // Set Task TLAS
  task_tlas.set_tlas({.tlas = std::array{tlas[renderer_manager->get_frame_index()]}});

  return true;
}

void AccelerationStructureManager::update_TLAS() 
{
  if(!initialized) {
    return;
  }
  update_buffers();
  update();
  TLAS_update_TG.execute();
}

bool AccelerationStructureManager::update()
{
  if (!initialized)
  {
    return false;
  }

  // TODO: should frame_index be passed as a parameter?
  daxa_u32 frame_index = renderer_manager->get_frame_index();

  // TODO: routine for delete and add rigid bodies?
  if (point_blas.size() > 0)
  {
    for (auto blas : point_blas)
    {
      device.destroy_blas(blas);
    }
    point_blas.clear();
  }

  point_blas_buffer_offset = 0;

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

  // TODO: 1 for all contact points for now
  clear_build_AS(1);

  /// Alignments:
  auto get_aligned = [&](u32 operand, u32 granularity) -> u32
  {
    return ((operand + (granularity - 1)) & ~(granularity - 1));
  };

  auto sim_config = device.buffer_host_address_as<SimConfig>(sim_host_buffer).value();

  daxa_u32 total_instances = current_rigid_body_count;

  // if(sim_config->collision_point_count > 0)
  // {
  ++total_instances;

  // Get contact points
  blas_geometries.at(0).push_back({
      .data = device.device_address(points_buffer[frame_index]).value(),
      .stride = sizeof(Aabb),
      .count = sim_config->g_c_info.collision_point_count,
      .flags = daxa::GeometryFlagBits::OPAQUE,
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

  if (proc_blas_scratch_offset + scratch_offset > AVERAGE_AS_SIZE * MAX_ACCELERATION_STRUCTURE_COUNT)
  {
    clear_build_AS(1);
    return false;
  }

  // Set the scratch offset
  blas_build_infos.back().scratch_data = device.device_address(proc_blas_scratch_buffer).value() + proc_blas_scratch_offset;

  // Increment the scratch offset
  proc_blas_scratch_offset += scratch_offset;

  // Get the BLAS instance offset
  auto blas_instance_offset = get_aligned(blas_build_sizes.back().acceleration_structure_size, ACCELERATION_STRUCTURE_BUILD_OFFSET_ALIGMENT);

  // Check if the buffer offset is within the limits
  if (point_blas_buffer_offset + blas_instance_offset > AVERAGE_AS_SIZE * MAX_ACCELERATION_STRUCTURE_COUNT)
  {
    clear_build_AS(1);
    return false;
  }

  // Create BLAS buffer from buffer
  point_blas.push_back(device.create_blas_from_buffer(
      {{
           .size = blas_build_sizes.back().acceleration_structure_size,
           .name = "blas_points",

       },
       point_blas_buffer,
       point_blas_buffer_offset}));
  // Add the BLAS buffer to the BLAS build info
  blas_build_infos.back().dst_blas = point_blas.back();

  // Increment the buffer offset
  point_blas_buffer_offset += blas_instance_offset;

  blas_instances_data[current_rigid_body_count] = {
      .transform = daxa_f32mat3x4(daxa_f32vec4(1.0f, 0.0f, 0.0f, 0.0f),
                                  daxa_f32vec4(0.0f, 1.0f, 0.0f, 0.0f),
                                  daxa_f32vec4(0.0f, 0.0f, 1.0f, 0.0f)),
      .instance_custom_index = current_rigid_body_count,
      .mask = 0xFF,
      .instance_shader_binding_table_record_offset = 1,
      .flags = {},
      .blas_device_address = device.device_address(point_blas.back()).value(),
  };
  // }

  // Set Task BLAS
  task_blas.set_blas({.blas = point_blas});

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

  // Get the build sizes
  tlas_build_sizes = device.tlas_build_sizes(tlas_build_info);

  // Set the scratch offset
  tlas_build_info.scratch_data = device.device_address(proc_tlas_scratch_buffer).value();

  // Set the TLAS buffer
  tlas_build_info.dst_tlas = tlas[frame_index];

  // Set Task TLAS
  task_tlas.set_tlas({.tlas = std::array{tlas[frame_index]}});

  return true;
}

bool AccelerationStructureManager::update_TLAS_resources(daxa::BufferId dispatch_buffer, daxa::BufferId collisions, daxa::BufferId sim_config_host_buffer)
{
  if (!initialized)
  {
    return !initialized;
  }

  task_dispatch_buffer.set_buffers({.buffers = std::array{dispatch_buffer}});

  sim_host_buffer = sim_config_host_buffer;

  return initialized;
}

void AccelerationStructureManager::record_accel_struct_tasks(TaskGraph &AS_TG)
{
  daxa::InlineTaskInfo task0({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_rigid_body_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_aabb_buffer),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = primitive_scratch_buffer,
            .dst_buffer = primitive_buffer,
            .dst_offset = previous_primitive_count * sizeof(Aabb),
            .size = primitive_scratch_offset,
        });

        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = rigid_body_scratch_buffer,
            .dst_buffer = rigid_body_buffer[renderer_manager->get_frame_index()],
            .dst_offset = previous_rigid_body_count * sizeof(RigidBody),
            .size = rigid_body_scratch_offset,
        });
      },
      .name = "copy rigid bodies and primitives",
  });
  daxa::InlineTaskInfo task1({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_rigid_body_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_aabb_buffer),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = rigid_body_buffer[renderer_manager->get_frame_index()],
            .dst_buffer = rigid_body_buffer[renderer_manager->get_next_frame_index()],
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
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_rigid_body_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_aabb_buffer),
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

  std::array<daxa::TaskBuffer, 2> buffers = {
      task_rigid_body_buffer,
      task_aabb_buffer,
  };
  std::array<daxa::TaskBlas, 1> blas = {
      task_blas,
  };
  std::array<daxa::TaskTlas, 1> tlas = {
      task_tlas,
  };

  AS_TG = task_manager->create_task_graph("Build Acceleration Structures", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, std::span<daxa::TaskBlas>(blas), std::span<daxa::TaskTlas>(tlas));
}

void AccelerationStructureManager::record_update_TLAS_tasks(TaskGraph &TLAS_TG, std::shared_ptr<daxa::ComputePipeline> update_AS_pipeline)
{
  auto user_callback_UI = [update_AS_pipeline](daxa::TaskInterface ti, auto &self)
  {
    ti.recorder.set_pipeline(*update_AS_pipeline);
    ti.recorder.push_constant(UpdateInstancesPushConstants{.task_head = ti.attachment_shader_blob});
    ti.recorder.dispatch_indirect({.indirect_buffer = ti.get(UpdateInstancesTaskHead::AT.dispatch_buffer).ids[0],
                                   .offset = 0});
  };

  using TTaskUI = TaskTemplate<UpdateInstancesTaskHead::Task, decltype(user_callback_UI)>;

  // Instantiate the task using the template class
  TTaskUI task_UI(std::array{
                      daxa::attachment_view(UpdateInstancesTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
                      daxa::attachment_view(UpdateInstancesTaskHead::AT.sim_config, task_sim_config),
                      daxa::attachment_view(UpdateInstancesTaskHead::AT.blas_instance_data, task_blas_instance_data),
                      daxa::attachment_view(UpdateInstancesTaskHead::AT.rigid_bodies, task_rigid_body_buffer),
                      daxa::attachment_view(UpdateInstancesTaskHead::AT.aabbs, task_aabb_buffer),
                  },
                  user_callback_UI);

  daxa::InlineTaskInfo task_BB({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_rigid_body_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_aabb_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_points_aabb_buffer),
          daxa::inl_attachment(daxa::TaskBlasAccess::BUILD_WRITE, task_blas),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        // build blas
        ti.recorder.build_acceleration_structures({
            .blas_build_infos = blas_build_infos,
        });
      },
      .name = "blas rebuild",
  });

  daxa::InlineTaskInfo task_BT({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_rigid_body_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_aabb_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_points_aabb_buffer),
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

  std::array<daxa::TaskBuffer, 6> buffers = {
      task_dispatch_buffer,
      task_sim_config,
      task_blas_instance_data,
      task_rigid_body_buffer,
      task_aabb_buffer,
      task_points_aabb_buffer,
  };
  std::array<daxa::TaskBlas, 1> blas = {
      task_blas,
  };
  std::array<daxa::TaskTlas, 1> tlas = {
      task_tlas,
  };

  TLAS_TG = task_manager->create_task_graph("Update TLAS", std::span<daxa::TaskBuffer>(buffers), {}, std::span<daxa::TaskBlas>(blas), std::span<daxa::TaskTlas>(tlas));

  TLAS_TG.add_task(task_UI);
  TLAS_TG.add_task(task_BB);
  TLAS_TG.add_task(task_BT);
}

void AccelerationStructureManager::update_buffers()
{
  task_sim_config.set_buffers({.buffers = std::array{rigid_body_manager->get_sim_config_buffer()}});
  task_rigid_body_buffer.set_buffers({.buffers = std::array{rigid_body_buffer[renderer_manager->get_frame_index()]}});
  task_blas_instance_data.set_buffers({.buffers = std::array{blas_instances_buffer}});
  task_points_aabb_buffer.set_buffers({.buffers = std::array{points_buffer[renderer_manager->get_frame_index()]}});
  task_previous_points_aabb_buffer.set_buffers({.buffers = std::array{points_buffer[renderer_manager->get_previous_frame_index()]}});
  task_aabb_buffer.set_buffers({.buffers = std::array{primitive_buffer}});
}

void AccelerationStructureManager::update_AS_buffers() {
  if(!initialized) {
    return;
  }
  AS_update_buffers_TG.execute();
}

void AccelerationStructureManager::record_update_AS_buffers_tasks(TaskGraph &AS_buffers_TG)
{
 daxa::InlineTaskInfo task0({
      .attachments = {
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_points_aabb_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, task_previous_points_aabb_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, gui_manager->task_previous_vertex_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, gui_manager->task_vertex_buffer),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, rigid_body_manager->task_old_sim_config),
          daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_WRITE, rigid_body_manager->task_sim_config),
      },
      .task = [this](daxa::TaskInterface const &ti)
      {
        // NOTE: previous_points_aabb_buffer is used then there's no need for synchronization?
        auto sim_config = device.buffer_host_address_as<SimConfig>(sim_host_buffer).value();
        auto current_frame_index = renderer_manager->get_frame_index();
        auto previous_frame_index = renderer_manager->get_previous_frame_index();

        ti.recorder.copy_buffer_to_buffer({
            .src_buffer = rigid_body_manager->get_sim_config_buffer(),
            .dst_buffer = rigid_body_manager->get_previous_sim_config_buffer(),
            .size = sizeof(SimConfig),
        });

        auto point_count = sim_config->g_c_info.collision_point_count;
        if(point_count > 0) {
          ti.recorder.copy_buffer_to_buffer({
              .src_buffer = points_buffer[current_frame_index],
              .dst_buffer = points_buffer[previous_frame_index],
              .size = point_count * sizeof(Aabb),
          });

          ti.recorder.copy_buffer_to_buffer({
              .src_buffer = gui_manager->get_vertex_buffer(),
              .dst_buffer = gui_manager->get_previous_vertex_buffer(),
              .size = point_count * sizeof(daxa_f32vec3),
          });
        }
      },
      .name = "copy points",
  });

  std::array<daxa::InlineTaskInfo, 1> tasks = {
      task0,
  };

  std::array<daxa::TaskBuffer, 6> buffers = {
      task_points_aabb_buffer,
      task_previous_points_aabb_buffer,
      gui_manager->task_previous_vertex_buffer,
      gui_manager->task_vertex_buffer,
      rigid_body_manager->task_old_sim_config,
      rigid_body_manager->task_sim_config,
  };

  AS_buffers_TG = task_manager->create_task_graph("Update Acceleration Structure Buffers", std::span<daxa::InlineTaskInfo>(tasks), std::span<daxa::TaskBuffer>(buffers), {}, {}, {});
}

BB_NAMESPACE_END