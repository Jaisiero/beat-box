#include "gpu_pool_allocator.inl"
[[vk::push_constant]] GpuPoolValidationPushConstants P;
[shader("compute")]
[numthreads(1,1,1)]
void entry_gpu_pool_validation(uint3 tid : SV_DispatchThreadID)
{
  GpuPoolTransaction* tx = (GpuPoolTransaction*)P.transaction_addr;
  uint failures = 0u;
  GpuPoolRequest request;
  for (uint i=0u;i<GPU_POOL_COUNT;++i) request.units[i]=8u;
  if (!tx->begin()) ++failures;
  if (tx->retire(0u,0u,8u)) ++failures;
  GpuPoolReservation r = tx->reserve(request);
  for (uint i=0u;i<GPU_POOL_COUNT;++i) if (r.offsets[i]!=8u) ++failures;
  if (tx->publish()) ++failures;
  if (!tx->mark_ready()) ++failures;
  for (uint i=0u;i<GPU_POOL_COUNT;++i) if (!tx->retire(i,0u,8u)) ++failures;
  if (!tx->publish()) ++failures;
  if (!tx->begin()) ++failures;
  request.units[GPU_POOL_COUNT-1u]=17u;
  r=tx->reserve(request);
  for (uint i=0u;i<GPU_POOL_COUNT;++i) if (r.offsets[i]!=GPU_POOL_INVALID) ++failures;
  if (tx->mark_ready() || tx->publish()) ++failures;
  ((uint*)P.failures_addr)[0]=failures;
}
