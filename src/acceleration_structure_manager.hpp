#pragma once

#include "defines.hpp"
#include "task_manager.hpp"

BB_NAMESPACE_BEGIN

struct AccelerationStructureManager
{

  // Daxa device
  daxa::Device &device;
  // Initialization flag
  bool initialized = false;

  // TODO: temporary
  static constexpr u32 MAX_ACCELERATION_STRUCTURE_COUNT = 1024;
  static constexpr u32 MAX_PRIMITIVE_COUNT = 1024;
  static constexpr u32 MAX_RIGID_BODY_COUNT = 1024;
  static constexpr u32 AVERAGE_AS_SIZE = 1024 * 1024;

  // Alignment of the acceleration structure build offset
  static constexpr u64 ACCELERATION_STRUCTURE_BUILD_OFFSET_ALIGMENT = 256;
  // Alignment of the scratch buffer
  u32 acceleration_structure_scratch_offset_alignment = 0;


  // Offset for the scratch buffer for RigidBodies
  u32 rigid_body_scratch_offset = 0;
  // Scratch buffer for RigidBodies
  daxa::BufferId rigid_body_scratch_buffer = {};
  // Rigid body count
  u32 current_rigid_body_count = 0;
  // Rigid body previous count
  u32 previous_rigid_body_count = 0;
  // Buffer for the RigidBodies
  daxa::BufferId rigid_body_buffer = {};

  // Offset for the scratch buffer for primitives
  u32 primitive_scratch_offset = 0;
  // Scratch buffer for primitives
  daxa::BufferId primitive_scratch_buffer = {};
  // Primitive count
  u32 current_primitive_count = 0;
  // Previous primitive count
  u32 previous_primitive_count = 0;
  // Buffer for the primitives
  daxa::BufferId primitive_buffer = {};

  // Offset for the BLAS scratch buffer
  u32 proc_blas_scratch_offset = 0;
  // Scratch buffer for the BLAS
  daxa::BufferId proc_blas_scratch_buffer = {};
  // Offset for the BLAS buffer
  u32 proc_blas_buffer_offset = 0;
  // Buffer for the BLAS
  daxa::BufferId proc_blas_buffer = {};
  // // Sub-allocated buffer for the BLAS
  std::vector<daxa::BlasId> proc_blas = {};

  // Buffer for the TLAS
  daxa::BufferId proc_tlas_buffer = {};
  // Scratch buffer for the TLAS
  daxa::BufferId proc_tlas_scratch_buffer = {};

  // Build BlAS info
  std::vector<daxa::BlasBuildInfo> blas_build_infos = {};
  // Build sizes for the BLAS
  std::vector<daxa::AccelerationStructureBuildSizesInfo> blas_build_sizes = {};
  // Build geometry for the BLAS
  std::vector<std::vector<daxa::BlasAabbGeometryInfo>> blas_geometries = {};

  // Build TLAS info
  daxa::AccelerationStructureBuildSizesInfo tlas_build_sizes = {};
  // Build TLAS info
  daxa::TlasBuildInfo tlas_build_info = {};
  // BLAS instances
  daxa::BufferId blas_instances_buffer = {};
  // BLAS instances data
  std::array<daxa::TlasInstanceInfo, 1> tlas_info = {};
  // BLAS instances data
  daxa_BlasInstanceData* blas_instances_data = nullptr;
  // TLAS
  daxa::TlasId tlas = {};

  // task for the RigidBodies
  daxa::TaskBuffer task_rigid_body_buffer{{.name = "rigid_body_buffer_task"}};
  // task for the AABB buffer
  daxa::TaskBuffer task_aabb_buffer{{.name = "aabb_buffer_task"}};
  // task for the BLAS
  daxa::TaskBlas task_blas{{.name = "blas_task"}};
  // task for the TLAS
  daxa::TaskTlas task_tlas{{.name = "tlas_task"}};

  // TaskGraph for building acceleration structures
  daxa::TaskGraph build_AS_task_graph;
  // TaskGraph for updating acceleration structures
  daxa::TaskGraph update_TLAS_TG;

  daxa::TaskBuffer task_dispatch_buffer{{.name = "dispatch_buffer"}};
  daxa::TaskBuffer task_sim_config{{.name = "sim_config"}};
  daxa::TaskBuffer task_blas_instance_data{{.name = "blas_instance_data"}};

  explicit AccelerationStructureManager(daxa::Device &device) : device(device)
  {
    if (device.is_valid())
    {
      acceleration_structure_scratch_offset_alignment = device.properties().acceleration_structure_properties.value().min_acceleration_structure_scratch_offset_alignment;
    }
  }

  ~AccelerationStructureManager()
  {
    destroy();
  }

  bool create(std::shared_ptr<daxa::ComputePipeline> update_pipeline)
  {
    if (device.is_valid() && !initialized)
    {
      // Create buffer for RigidBodies
      rigid_body_scratch_buffer = device.create_buffer({
          .size = MAX_RIGID_BODY_COUNT * sizeof(RigidBody),
          .allocate_info = daxa::MemoryFlagBits::HOST_ACCESS_SEQUENTIAL_WRITE,
          .name = "rigid_body_scratch_buffer",
      });

      // Create buffer for RigidBodies
      rigid_body_buffer = device.create_buffer({
          .size = MAX_RIGID_BODY_COUNT * sizeof(RigidBody),
          .name = "rigid_body_buffer",
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

      // Create BLAS buffer
      proc_blas_buffer = device.create_buffer({
          .size = AVERAGE_AS_SIZE * MAX_ACCELERATION_STRUCTURE_COUNT,
          .name = "proc_blas_buffer",
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
      tlas = device.create_tlas({
          .size = AVERAGE_AS_SIZE,
          .name = "tlas",
      });

      // Set the buffers for the tasks
      task_rigid_body_buffer.set_buffers({.buffers = std::array{rigid_body_buffer}});
      task_aabb_buffer.set_buffers({.buffers = std::array{primitive_buffer}});
      task_blas_instance_data.set_buffers({.buffers = std::array{blas_instances_buffer}});

      // Task Build TLAS
      build_AS_task_graph = daxa::TaskGraph({
          .device = device,
          .name = "build_AS_task_graph",
      });
      record_accel_struct_tasks(build_AS_task_graph);
      build_AS_task_graph.submit({});
      build_AS_task_graph.complete({});

      // Task Update TLAS
      update_TLAS_TG = daxa::TaskGraph({
          .device = device,
          .name = "update_TLAS_TG",
      });
      record_update_TLAS_tasks(update_TLAS_TG, update_pipeline);
      update_TLAS_TG.submit({});
      update_TLAS_TG.complete({});

      initialized = true;
    }

    return initialized;
  }

  void destroy()
  {
    if (initialized)
    {
      device.destroy_buffer(rigid_body_scratch_buffer);
      device.destroy_buffer(rigid_body_buffer);
      device.destroy_buffer(primitive_scratch_buffer);
      device.destroy_buffer(primitive_buffer);
      for(auto blas : proc_blas)
      {
        device.destroy_blas(blas);
      }
      device.destroy_buffer(proc_blas_buffer);
      device.destroy_buffer(proc_blas_scratch_buffer);
      device.destroy_buffer(proc_tlas_buffer);
      device.destroy_buffer(proc_tlas_scratch_buffer);
      device.destroy_buffer(blas_instances_buffer);
      device.destroy_tlas(tlas);
      initialized = false;
    }
  }

  void free_accel_structs()
  {
    // Freeing BLAS
    for(auto blas : proc_blas)
    {
      device.destroy_blas(blas);
    }

    // Freeing TLAS
    device.destroy_tlas(tlas);

    // Resetting the offsets
    current_rigid_body_count = 0;
    previous_rigid_body_count = 0;
    current_primitive_count = 0;
    primitive_scratch_offset = 0;
    previous_primitive_count = 0;
    proc_blas_buffer_offset = 0;
    proc_blas_scratch_offset = 0;
  }

  void build_AS()
  {
    build_AS_task_graph.execute({});
  }

  bool build_accel_structs(std::vector<RigidBody> &rigid_bodies, std::vector<Aabb> const &primitives)
  {
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
    for(u32 i = 0; i < rigid_body_count; ++i) {

      // Get the rigid body
      auto rigid_body = rigid_bodies[i];

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

      if(proc_blas_scratch_offset + scratch_offset > AVERAGE_AS_SIZE * MAX_ACCELERATION_STRUCTURE_COUNT)
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
      if(proc_blas_buffer_offset + blas_instance_offset > AVERAGE_AS_SIZE * MAX_ACCELERATION_STRUCTURE_COUNT)
      {
        clear_build_AS();
        return false;
      }

      // Create BLAS buffer from buffer
      proc_blas.push_back(device.create_blas_from_buffer(
        {
          {
            .size = blas_build_sizes.back().acceleration_structure_size,
            .name = "blas" + std::to_string(i),

          }, 
          proc_blas_buffer,
          proc_blas_buffer_offset
        }
      ));
      // Add the BLAS buffer to the BLAS build info
      blas_build_infos.back().dst_blas = proc_blas.back();

      // Increment the buffer offset
      proc_blas_buffer_offset += blas_instance_offset;

      blas_instances_data[i] = {
          .transform = rigid_body_get_transform_matrix(rigid_body),
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
    tlas_build_info.dst_tlas = tlas;

    // Set Task TLAS
    task_tlas.set_tlas({.tlas = std::array{tlas}});

    return true;
  }

  void update_TLAS()
  {
    update_TLAS_TG.execute({});
  }

  bool update()
  {
    if(!initialized)
    {
      return false;
    }

    // Destroy TLAS
    if(!tlas.is_empty())
    {
      device.destroy_tlas(tlas);
    }

    tlas = device.create_tlas({
        .size = AVERAGE_AS_SIZE,
        .name = "tlas",
    });

    tlas_info[0] = {
        .data = device.device_address(blas_instances_buffer).value(),
        .count = current_rigid_body_count,
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
    tlas_build_info.dst_tlas = tlas;

    // Set Task TLAS
    task_tlas.set_tlas({.tlas = std::array{tlas}});

    return true;
  }

  bool update_TLAS_resources(daxa::BufferId dispatch_buffer, daxa::BufferId sim_config)
  {
    if(!initialized)
    {
      return !initialized;
    }

    task_dispatch_buffer.set_buffers({.buffers = std::array{dispatch_buffer}});
    task_sim_config.set_buffers({.buffers = std::array{sim_config}});

    return initialized;
  }


private: 
  void record_accel_struct_tasks(daxa::TaskGraph &AS_TG)
  {
    AS_TG.use_persistent_buffer(task_rigid_body_buffer);
    AS_TG.use_persistent_buffer(task_aabb_buffer);
    AS_TG.use_persistent_blas(task_blas);
    AS_TG.use_persistent_tlas(task_tlas);
    AS_TG.add_task({
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
              .dst_buffer = rigid_body_buffer,
              .dst_offset = previous_rigid_body_count * sizeof(RigidBody),
              .size = rigid_body_scratch_offset,
          });
          
          primitive_scratch_offset = 0;
          rigid_body_scratch_offset = 0;
        },
        .name = "copy rigid bodies and primitives",
    });
    AS_TG.add_task({
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
    AS_TG.add_task({
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
  }

  void record_update_TLAS_tasks(daxa::TaskGraph &TLAS_TG, std::shared_ptr<daxa::ComputePipeline> update_AS_pipeline)
  {
    TLAS_TG.use_persistent_buffer(task_dispatch_buffer);
    TLAS_TG.use_persistent_buffer(task_sim_config);
    TLAS_TG.use_persistent_buffer(task_blas_instance_data);
    TLAS_TG.use_persistent_buffer(task_rigid_body_buffer);
    TLAS_TG.use_persistent_buffer(task_aabb_buffer);
    TLAS_TG.use_persistent_blas(task_blas);
    TLAS_TG.use_persistent_tlas(task_tlas);

    auto user_callback = [update_AS_pipeline](daxa::TaskInterface ti, auto& self) {
        ti.recorder.set_pipeline(*update_AS_pipeline);
        ti.recorder.push_constant(UpdateInstancesPushConstants{.task_head = ti.attachment_shader_blob});
        ti.recorder.dispatch_indirect({
            .indirect_buffer = ti.get(UpdateInstancesTaskHead::AT.dispatch_buffer).ids[0],
            .offset = 0});
    };

    // Instantiate the task using the template class
    TaskTemplate<UpdateInstancesTaskHead::Task, decltype(user_callback)> task(std::array{
          daxa::attachment_view(UpdateInstancesTaskHead::AT.dispatch_buffer, task_dispatch_buffer),
          daxa::attachment_view(UpdateInstancesTaskHead::AT.sim_config, task_sim_config),
          daxa::attachment_view(UpdateInstancesTaskHead::AT.blas_instance_data, task_blas_instance_data),
          daxa::attachment_view(UpdateInstancesTaskHead::AT.rigid_bodies, task_rigid_body_buffer),
          daxa::attachment_view(UpdateInstancesTaskHead::AT.aabbs, task_aabb_buffer),
      }, user_callback);

    // Add the task to the task graph
    TLAS_TG.add_task(task);

    TLAS_TG.add_task({
        .attachments = {
            daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_rigid_body_buffer),
            daxa::inl_attachment(daxa::TaskBufferAccess::TRANSFER_READ, task_aabb_buffer),
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
  }

}; // struct AccelerationStructureManager

BB_NAMESPACE_END