#include "shared.inl"

public
struct IslandBuilder
{

private
  daxa_u32 max_active_bodies;
private
  daxa_u32 island_count;
private
  daxa_u32 max_manifold_count;
private
  Ptr<BodyLink> body_links;
private
  Ptr<Island> islands;
private
  Ptr<BodyLinkIsland> body_indices;
private
  Ptr<ManifoldLinkIsland> manifold_indices;

  __init(Ptr<BodyLink> links, Ptr<Island> island_storage)
  {
    max_active_bodies = 0;
    island_count = 0;
    max_manifold_count = 0;
    body_links = links;
    islands = island_storage;
  }

  __init(daxa_u32 active_body_count, Ptr<BodyLink> links)
  {
    max_active_bodies = active_body_count;
    island_count = 0;
    max_manifold_count = 0;
    body_links = links;
  }

  __init(daxa_u32 island_num, Ptr<Island> island_storage)
  {
    max_active_bodies = 0;
    island_count = island_num;
    max_manifold_count = 0;
    islands = island_storage;
  }

  __init(daxa_u32 active_body_count, Ptr<BodyLink> links, daxa_u32 island_num, Ptr<Island> island_storage)
  {
    max_active_bodies = active_body_count;
    island_count = island_num;
    max_manifold_count = 0;
    body_links = links;
    islands = island_storage;
  }

  __init(daxa_u32 active_body_count, Ptr<BodyLink> links, daxa_u32 island_num, Ptr<Island> island_storage, Ptr<BodyLinkIsland> indices)
  {
    max_active_bodies = active_body_count;
    island_count = island_num;
    max_manifold_count = 0;
    body_links = links;
    islands = island_storage;
    body_indices = indices;
  }

  __init(daxa_u32 island_num, Ptr<Island> island_storage, daxa_u32 active_body_count, Ptr<BodyLinkIsland> indices)
  {
    max_active_bodies = active_body_count;
    island_count = island_num;
    max_manifold_count = 0;
    islands = island_storage;
    body_indices = indices;
  }

  __init(daxa_u32 active_body_count, Ptr<BodyLink> links, daxa_u32 island_num, Ptr<Island> island_storage, daxa_u32 manifold_count, Ptr<ManifoldLinkIsland> indices)
  {
    max_active_bodies = active_body_count;
    island_count = island_num;
    max_manifold_count = manifold_count;
    body_links = links;
    islands = island_storage;
    manifold_indices = indices;
  }

  __init(daxa_u32 island_num, Ptr<Island> island_storage, daxa_u32 manifold_count, Ptr<ManifoldLinkIsland> indices)
  {
    max_active_bodies = 0;
    island_count = island_num;
    max_manifold_count = manifold_count;
    islands = island_storage;
    manifold_indices = indices;
  }

  daxa_u32 get_lowest_body_index(daxa_u32 in_activate_body_index)
  {
    daxa_u32 index = in_activate_body_index;
    for (;;)
    {
      daxa_u32 link_to = body_links[index].active_index;
      if (link_to == index)
        break;
      index = link_to;
    }
    return index;
  }

  internal daxa_u32 get_lowest_body_index_atomic(daxa_u32 in_activate_body_index)
  {
    daxa_u32 index = in_activate_body_index;
    for (;;)
    {
      daxa_u32 link_to = 0;
      InterlockedAdd(body_links[index].active_index, 0, link_to);
#if BB_DEBUG
      // printf("get_lowest_body_index link_to: %d index: %d\n", link_to, index);
#endif // BB_DEBUG
      if (link_to == index)
        break;
      index = link_to;
    }
    return index;
  }

  void link_bodies(daxa_u32 in_first, daxa_u32 in_second)
  {
    // Both need to be active, we don't want to create an island with static objects
    if (in_first >= max_active_bodies || in_second >= max_active_bodies)
      return;

    // Start the algorithm with the two bodies
    daxa_u32 first_link_to = in_first;
    daxa_u32 second_link_to = in_second;

    for (;;)
    {
      // Follow the chain until we get to the body with lowest index
      // If the swap compare below fails, we'll keep searching from the lowest index for the new lowest index
      first_link_to = get_lowest_body_index_atomic(first_link_to);
      second_link_to = get_lowest_body_index_atomic(second_link_to);

      // If the targets are the same, the bodies are already connected
      if (first_link_to != second_link_to)
      {
        // We always link the highest to the lowest
        if (first_link_to < second_link_to)
        {
          // Attempt to link the second to the first
          // Since we found this body to be at the end of the chain it must point to itself, and if it
          // doesn't it has been reparented and we need to retry the algorithm
          daxa_u32 old_value = 0;
          InterlockedCompareExchange(body_links[second_link_to].active_index, second_link_to, first_link_to, old_value);
          if (old_value != second_link_to)
            continue;
#if BB_DEBUG
            // printf("linking %d to %d, old_value: %d\n", second_link_to, first_link_to, old_value);
#endif // BB_DEBUG
        }
        else
        {
          // Attempt to link the first to the second
          // Since we found this body to be at the end of the chain it must point to itself, and if it
          // doesn't it has been reparented and we need to retry the algorithm
          daxa_u32 old_value = 0;
          InterlockedCompareExchange(body_links[first_link_to].active_index, first_link_to, second_link_to, old_value);
          if (old_value != first_link_to)
            continue;
#if BB_DEBUG
            // printf("linking %d to %d, old_value: %d\n", first_link_to, second_link_to, old_value);
#endif // BB_DEBUG
        }
      }

      // Linking succeeded!
      // Chains of bodies can become really long, resulting in an O(N) loop to find the lowest body index
      // to prevent this we attempt to update the link of the bodies that were passed in to directly point
      // to the lowest index that we found. If the value became lower than our lowest link, some other
      // thread must have relinked these bodies in the mean time so we won't update the value.
      daxa_u32 lowest_link_to = min(first_link_to, second_link_to);
      daxa_u32 old_value = 0;
      InterlockedMin(body_links[in_first].active_index, lowest_link_to, old_value);
      InterlockedMin(body_links[in_second].active_index, lowest_link_to, old_value);
      break;
    }
  }

  void add_island(Ptr<daxa_u32> atomic_island_count, daxa_u32 index)
  {
    let body_link = &body_links[index];
    if (body_link.active_index == index)
    {
      daxa_u32 island_index = 0u;
      InterlockedAdd(*atomic_island_count, 1, island_index);
      body_link.island_index = island_index;
      // unknow start index yet, reset counter
      islands[island_index] = Island(MAX_U32, 0, 0, MAX_U32, 0);
#if defined(BB_DEBUG)
      // printf("add_island: %d\n", island_index);
#endif // BB_DEBUG
    }
  }

  void prefix_sum_islands()
  {
    if (island_count == 0)
      return;

    // Prefix sum the island sizes
    islands[0].start_index = 0;

    // Calculate the start index for each island
    for (daxa_u32 i = 1; i < island_count; i++)
    {
      islands[i].start_index = islands[i - 1].start_index + islands[i - 1].max_count;
    }
  }

  void expand_island(daxa_u32 index)
  {
    if (index >= max_active_bodies)
      return;

    BodyLink b_link = body_links[index];

    daxa_u32 island_index = MAX_U32;
    if (b_link.island_index != index)
    {
      let _index = get_lowest_body_index(index);
      body_links[index].island_index = body_links[_index].island_index;
      island_index = body_links[index].island_index;
    }
    else
    {
      island_index = b_link.island_index;
    }

    if (island_index < island_count)
    {
      daxa_u32 old_count = 0;
      InterlockedAdd(islands[island_index].max_count, 1, old_count);
    }
  }

  void body_link_to_island(daxa_u32 index)
  {
    if (index >= max_active_bodies)
      return;

    BodyLink b_link = body_links[index];

    // increment atomic counter from island
    daxa_u32 island_index = b_link.island_index;
    if (island_index < island_count)
    {
      daxa_u32 old_count = 0;
      InterlockedAdd(islands[island_index].count, 1, old_count);
      // write body link inside range in atomic counter from island
      daxa_u32 body_index = islands[island_index].start_index + old_count;
      if (body_index < max_active_bodies)
      {
        body_indices[body_index].active_index = index;
      }
    }
  }

  // NOTE: Selection sort
  void sort_body_links_in_island(daxa_u32 island_index)
  {

    if (island_index >= island_count)
      return;

    daxa_u32 start_index = islands[island_index].start_index;
    daxa_u32 max_count = islands[island_index].max_count;

    if (start_index + max_count >= max_active_bodies)
      return;

    // Sort body links in island
    for (daxa_u32 i = 0; i < max_count; i++)
    {
      daxa_u32 min_index = i;
      for (daxa_u32 j = i + 1; j < max_count; j++)
      {
        if (body_indices[start_index + j].active_index < body_indices[start_index + min_index].active_index)
        {
          min_index = j;
        }
      }

      if (min_index != i)
      {
        BodyLinkIsland tmp = body_indices[start_index + i];
        body_indices[start_index + i] = body_indices[start_index + min_index];
        body_indices[start_index + min_index] = tmp;
      }
    }
  }

  void expand_manifold_island(daxa_u32 body_index)
  {
    if (body_index >= max_active_bodies)
      return;

    daxa_u32 island_index = body_links[body_index].island_index;
    if (island_index < island_count)
    {
      daxa_u32 old_count = 0;
      InterlockedAdd(islands[island_index].max_manifold_count, 1, old_count);
    }
#if defined(BB_DEBUG)
    else
    {
      printf("expand_manifold_island failed: island_index: %d, island_count: %d\n", island_index, island_count);
    }
#endif // BB_DEBUG
  }
};

public
struct ContactIslandBuilder
{
private
  daxa_u32 island_count;
private
  Ptr<daxa_u32> contact_island_count;
private
  daxa_u32 max_active_bodies;
private
  Ptr<ContactIsland> contact_islands;
private
  Ptr<Island> islands;
private
  daxa_u32 max_manifold_count;
private
  Ptr<BodyLink> body_links;
private
  Ptr<ManifoldLinkIsland> manifold_indices;

  __init() {}

  __init(daxa_u32 island_num, Ptr<Island> island_storage, Ptr<daxa_u32> contact_island_num, Ptr<ContactIsland> contact_island_storage)
  {
    island_count = island_num;
    contact_island_count = contact_island_num;
    max_active_bodies = 0;
    max_manifold_count = 0;
    islands = island_storage;
    contact_islands = contact_island_storage;
  }

   __init(Ptr<daxa_u32> contact_island_num, Ptr<ContactIsland> contact_island_storage) {
    island_count = 0;
    contact_island_count = contact_island_num;
    max_active_bodies = 0;
    max_manifold_count = 0;
    contact_islands = contact_island_storage;
  }

  __init(daxa_u32 active_body_count, Ptr<BodyLink> body_link, daxa_u32 island_num, Ptr<Island> island_storage, Ptr<daxa_u32> contact_island_num, Ptr<ContactIsland> contact_island_storage, daxa_u32 collision_num, Ptr<ManifoldLinkIsland> manifold_storage)
  {
    island_count = island_num;
    contact_island_count = contact_island_num;
    max_active_bodies = active_body_count;
    max_manifold_count = collision_num;
    islands = island_storage;
    contact_islands = contact_island_storage;
    body_links = body_link;
    manifold_indices = manifold_storage;
  }


  __init(Ptr<daxa_u32> contact_island_num, Ptr<ContactIsland> contact_island_storage, daxa_u32 collision_num, Ptr<ManifoldLinkIsland> manifold_storage)
  {
    island_count = 0;
    contact_island_count = contact_island_num;
    max_active_bodies = 0;
    max_manifold_count = collision_num;
    contact_islands = contact_island_storage;
    manifold_indices = manifold_storage;
  }

  void gather_contact_islands(daxa_u32 island_index)
  {
    if (island_index >= island_count)
      return;

    // get manifold count from island
    daxa_u32 manifold_count = islands[island_index].max_manifold_count;

    // if no manifolds, skip
    if (manifold_count == 0)
      return;

    // increment atomic counter from island
    daxa_u32 old_count = 0;
    InterlockedAdd(*contact_island_count, 1, old_count);
    // write contact island inside range in atomic counter from island
    daxa_u32 contact_island_index = old_count;
    contact_islands[contact_island_index] = ContactIsland(island_index, MAX_U32, manifold_count, 0);
    islands[island_index].contact_island_index = contact_island_index;
  }

  void manifold_prefix_sum_islands()
  {
    if (*contact_island_count == 0)
      return;

    // Prefix sum the island sizes
    contact_islands[0].start_index = 0;
#if defined(BB_DEBUG)
    // printf("contact_islands[0]start index: %d, count: %d\n", contact_islands[0].start_index, contact_islands[0].max_count);
#endif // BB_DEBUG

    // Calculate the start index for each island
    for (daxa_u32 i = 1; i < *contact_island_count; i++)
    {
      contact_islands[i].start_index = contact_islands[i - 1].start_index + contact_islands[i - 1].max_count;
#if defined(BB_DEBUG)
      // printf("contact_islands[%d]. start index: %d, count: %d\n", i, contact_islands[i].start_index, contact_islands[i].max_count);
#endif // BB_DEBUG
    }
  }

  void manifold_link_to_island(daxa_u32 manifold_index, daxa_u32 body_a_index,
                               daxa_u32 body_b_index)
  {
    daxa_u32 min_index = min(body_a_index, body_b_index);

    if (min_index >= max_active_bodies)
      return;

    BodyLink b_link = body_links[min_index];

    // increment atomic counter from island
    daxa_u32 island_index = b_link.island_index;
    if (island_index < *contact_island_count)
    {
      daxa_u32 contact_island_index = islands[island_index].contact_island_index;

      if(contact_island_index >= *contact_island_count)
        return;
      
      daxa_u32 old_count = 0;
      InterlockedAdd(contact_islands[contact_island_index].count, 1, old_count);
      // write body link inside range in atomic counter from island
      daxa_u32 manifold_link_index = contact_islands[contact_island_index].start_index + old_count;
      if (manifold_index < max_manifold_count)
      {
        daxa_u32 max_index = max(body_a_index, body_b_index);
        manifold_indices[manifold_link_index] = ManifoldLinkIsland(manifold_index, min_index, max_index);
      }
    }
  }

  internal bool compare_manifold_link(ManifoldLinkIsland a, ManifoldLinkIsland b)
  {
    if (a.body_a_index < b.body_a_index)
    {
      return true;
    }
    else if (a.body_a_index == b.body_a_index)
    {
      return a.body_b_index < b.body_b_index;
    }
    else
    {
      return false;
    }
  }

  void sort_manifold_links_in_island(daxa_u32 contact_island_index)
  {

    if (contact_island_index >= *contact_island_count)
      return;

    daxa_u32 start_index = contact_islands[contact_island_index].start_index;
    daxa_u32 max_count = contact_islands[contact_island_index].max_count;

    if (start_index + max_count > max_manifold_count)
      return;

    // Sort body links in island
    for (daxa_u32 i = 0; i < max_count; i++)
    {
      daxa_u32 min_index = i;
      for (daxa_u32 j = i + 1; j < max_count; j++)
      {
        if (compare_manifold_link(manifold_indices[start_index + j], manifold_indices[start_index + min_index]))
        {
          min_index = j;
        }
      }

      if (min_index != i)
      {
        ManifoldLinkIsland tmp = manifold_indices[start_index + i];
        manifold_indices[start_index + i] = manifold_indices[start_index + min_index];
        manifold_indices[start_index + min_index] = tmp;
      }
    }
  };
};