#pragma once
#include <algorithm>
#include <cstddef>
#include <vector>

// A recycled tail can straddle the old vector end. Resize initializes only the
// appended suffix, so clear the entire allocation after growing the backing.
template <class T>
void prepare_pool_storage(std::vector<T> &storage, std::size_t offset, std::size_t size, T fill)
{
  if (offset + size > storage.size()) storage.resize(offset + size, fill);
  std::fill(storage.begin() + offset, storage.begin() + offset + size, fill);
}
