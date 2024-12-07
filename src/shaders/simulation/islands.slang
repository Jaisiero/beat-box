#include "shared.inl"

struct IslandBuilder {

  daxa_u32 max_active_bodies;
  Ptr<BodyLink> body_links;


  __init(daxa_u32 active_rigid_body_count, Ptr<BodyLink> links) {
    max_active_bodies = active_rigid_body_count;
    body_links = links;
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
          if(old_value != second_link_to)
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
          if(old_value != first_link_to)
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
};