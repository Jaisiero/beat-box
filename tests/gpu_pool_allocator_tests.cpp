#include "gpu_pool_allocator.inl"
#include "pool_storage.hpp"
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

static void check(bool ok) { if (!ok) { std::cerr << "GPU pool invariant failed\n"; std::abort(); } }
int main()
{
  std::vector<unsigned> storage = {11,22,0xffffffffu};
  prepare_pool_storage(storage,2,3,0u);
  check(storage == std::vector<unsigned>({11,22,0,0,0}));
  prepare_pool_storage(storage,1,1,0u);
  check(storage == std::vector<unsigned>({11,0,0,0,0}));
  prepare_pool_storage(storage,5,2,7u);
  check(storage == std::vector<unsigned>({11,0,0,0,0,7,7}));
  auto pool = std::make_unique<GpuFreeList>();
  pool->capacity = 4096;
  std::vector<bool> occupied(pool->capacity);
  std::vector<GpuPoolRange> allocations;
  std::mt19937 rng(0x504f4f4c);
  for (unsigned step = 0; step < 100000; ++step)
  {
    if (!allocations.empty() && rng()%2)
    {
      auto i = rng()%allocations.size();
      auto range = allocations[i];
      check(pool->release(range.offset, range.size));
      for (unsigned j=range.offset; j<range.offset+range.size; ++j) occupied[j]=false;
      allocations.erase(allocations.begin()+i);
    }
    else
    {
      unsigned size=1u+static_cast<unsigned>(rng()%97), expected=GPU_POOL_INVALID;
      for (unsigned i=0; i+size<=pool->capacity; ++i)
      {
        bool free=true;
        for (unsigned j=i; j<i+size; ++j) if (occupied[j]) { free=false; break; }
        if (free) { expected=i; break; }
      }
      unsigned offset=pool->allocate(size);
      check(offset==expected);
      if (offset!=GPU_POOL_INVALID)
      {
        allocations.push_back({offset,size});
        for (unsigned j=offset; j<offset+size; ++j) occupied[j]=true;
      }
    }
    check(pool->valid());
    check(pool->live_units==std::count(occupied.begin(),occupied.end(),true));
  }
  for (auto r:allocations) check(pool->release(r.offset,r.size));
  check(pool->high_water==0 && pool->range_count==0);
  check(pool->allocate(0xffffffffu)==GPU_POOL_INVALID);
  check(!pool->release(0xffffffffu,2));

  // Free-list metadata exhaustion is recoverable and leaves accounting intact.
  *pool={}; pool->capacity=4096;
  check(pool->allocate(4096)==0);
  for (unsigned i=0;i<GPU_POOL_RANGE_CAPACITY;++i) check(pool->release(i*2,1));
  check(!pool->release(GPU_POOL_RANGE_CAPACITY*2,1));
  check(pool->valid());
  check(!pool->release(0,1)); // double free
  check(!pool->release(1,2)); // overlaps an existing free range
  check(pool->valid());

  auto tx=std::make_unique<GpuPoolTransaction>();
  for (auto &p:tx->committed) { p.capacity=16; check(p.allocate(8)==0); }
  GpuPoolRequest request{};
  for (auto &u:request.units) u=8;
  check(tx->begin()); check(!tx->begin());
  check(!tx->retire(0,0,8)); // source cannot be recycled during construction
  auto r=tx->reserve(request);
  for (auto off:r.offsets) check(off==8);
  check(!tx->publish()); // geometry is not ready
  check(tx->mark_ready());
  check(tx->reserve(request).offsets[0]==GPU_POOL_INVALID);
  for (unsigned i=0;i<GPU_POOL_COUNT;++i) check(tx->retire(i,0,8));
  check(tx->publish() && tx->epoch==1);
  for (auto &p:tx->committed) check(p.valid() && p.live_units==8 && p.ranges[0].offset==0);
  check(tx->begin());
  request.units[GPU_POOL_COUNT-1]=17;
  check(tx->reserve(request).offsets[0]==GPU_POOL_INVALID);
  check(tx->phase==3 && tx->failed_pool==GPU_POOL_COUNT-1);
  check(!tx->mark_ready() && !tx->publish() && tx->epoch==1);
  for (auto &p:tx->committed) check(p.valid() && p.live_units==8 && p.ranges[0].size==8);
  check(tx->begin()); request.units[GPU_POOL_COUNT-1]=8;
  r=tx->reserve(request);
  for (auto off:r.offsets) check(off==0);
  tx->abort(); check(!tx->publish());
  for (auto &p:tx->committed) check(p.live_units==8);
  std::cout << "100000 pool operations and transaction rollback checks passed\n";
}
