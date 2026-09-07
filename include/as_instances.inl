#pragma once
#include "shared.inl"

DAXA_DECL_TASK_HEAD_BEGIN(UpdateInstancesTaskHead)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(SimConfig), sim_config)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ_WRITE, daxa_RWBufferPtr(daxa_BlasInstanceData), blas_instance_data)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_RWBufferPtr(RigidBodyEntry), rigid_body_map)
DAXA_TH_BUFFER_PTR(COMPUTE_SHADER_READ, daxa_BufferPtr(RigidBody), rigid_bodies)
DAXA_DECL_TASK_HEAD_END

struct UpdateInstancesPushConstants
{
  DAXA_TH_BLOB(UpdateInstancesTaskHead, task_head)
};
