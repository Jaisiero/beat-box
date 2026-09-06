#pragma once
#include "gpu_pool_allocator.inl"
#include "fragment_plan.inl"
#include "fracture_setup.inl"

struct FractureBodyEdit
{
  daxa_u32 kind, parent_id, shape_id, primitive_count;
  daxa_f32vec3 crop_off;
};
struct FracturePublicationPushConstants
{
  daxa_u64 bodies_addr, instances_addr, shapes_addr, derived_addr, contexts_addr;
  daxa_u64 allocator_addr, spawn_addr, primitives_addr;
  daxa_u32 body_count;
};
struct FractureAllocationState
{
  // Reservations are private until packing finishes. AS/body publication is a
  // separate step; previously submitted readers must finish before retirement.
  GpuFreeList pools[GPU_POOL_COUNT];
  GpuFreeList reserved[GPU_POOL_COUNT];
  daxa_u32 private_shapes[BB_MAX_VOXEL_SHAPE_COUNT];
  FractureBodyEdit body_edits[BB_MAX_RIGID_BODY_COUNT];
  daxa_u32 active, failures, spawn_seed;
};
struct FractureChildAllocation
{
  daxa_u32 offsets[GPU_POOL_COUNT];
};
struct FractureAllocationManifest
{
  daxa_u32 status, count;
  daxa_u32 dispatch_x, dispatch_y, dispatch_z;
  FractureChildAllocation children[FRAGMENT_PLAN_CAPACITY];
};
struct FractureBatchChild
{
  daxa_u32 parent_id;
  FragmentComponent component;
  FractureChildAllocation allocation;
};
struct FractureBatchManifest
{
  daxa_u32 status, count, refused;
  FractureBatchChild children[FRAGMENT_PLAN_CAPACITY];
};
struct FracturePartitionInput
{
  VoxelShape shape;
  daxa_u32 body_id, event_slot;
};
struct FractureAllocatorPushConstants
{
  daxa_u64 state_addr, plan_addr, allocation_addr, contexts_addr, shapes_addr, batch_addr;
  daxa_u32 parent_id, operation;
};
struct FractureBatchPackingPushConstants
{
  daxa_u64 plan_addr, allocation_addr, contexts_addr, labels_addr, remap_addr, occupancy_addr;
  daxa_u32 parent_id;
};

static const daxa_u32 FRACTURE_EDIT_RETIRE = 1u;
static const daxa_u32 FRACTURE_EDIT_SPAWN = 2u;

struct FractureSceneEditManifest
{
  daxa_u32 status, retired_count, spawn_template, spawn_id;
  daxa_u32 ids[BB_MAX_RIGID_BODY_COUNT];
};
struct FractureSceneEditPushConstants
{
  daxa_u64 allocator_addr, bodies_addr, shapes_addr, templates_addr, spawn_addr, output_addr;
  daxa_u32 body_count, template_count, operation;
  daxa_f32 kill_y;
};

#if defined(__cplusplus)
static_assert(sizeof(FractureBodyEdit) == 28);
static_assert(sizeof(FractureChildAllocation) == 24);
static_assert(sizeof(FractureBatchChild) == 72);
static_assert(sizeof(FractureBatchManifest) == 12 + 72 * FRAGMENT_PLAN_CAPACITY);
static_assert(sizeof(FractureAllocatorPushConstants) == 56);
#endif
