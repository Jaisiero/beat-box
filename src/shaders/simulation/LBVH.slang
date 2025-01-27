#include "shared.inl"

// credits: https://developer.nvidia.com/blog/thinking-parallel-part-iii-tree-construction-gpu/
struct MortonCodeGenerator
{
    // Expands a 10-bit integer into 30 bits
    // by inserting 2 zeros after each bit.
    static daxa_u32 expand_bits(daxa_u32 v)
    {
        v = (v * 0x00010001u) & 0xFF0000FFu;
        v = (v * 0x00000101u) & 0x0F00F00Fu;
        v = (v * 0x00000011u) & 0xC30C30C3u;
        v = (v * 0x00000005u) & 0x49249249u;
        return v;
    }

    // Calculates a 30-bit Morton code for the
    // given 3D point located within the unit cube [0,1].
    static daxa_u32 morton3D(daxa_f32vec3 p)
    {
        daxa_f32 x = p.x;
        daxa_f32 y = p.y;
        daxa_f32 z = p.z;
        x = min(max(x * 1024.0f, 0.0f), 1023.0f);
        y = min(max(y * 1024.0f, 0.0f), 1023.0f);
        z = min(max(z * 1024.0f, 0.0f), 1023.0f);
        daxa_u32 xx = expand_bits(daxa_u32(x));
        daxa_u32 yy = expand_bits(daxa_u32(y));
        daxa_u32 zz = expand_bits(daxa_u32(z));
        return xx * 4 + yy * 2 + zz;
    }
};

#if defined(BB_RADIX_SORT)
groupshared Atomic<daxa::u32> sums[RADIX_SORT_BINS / SUBGROUP_SIZE]; // subgroup reductions
groupshared Atomic<daxa::u32> global_offsets[RADIX_SORT_BINS];       // global exclusive scan (prefix sum)

struct BinFlags
{
    Atomic<daxa::u32> flags[RADIX_SORT_WORKGROUP_SIZE / BITS];
};
groupshared BinFlags[RADIX_SORT_BINS] bin_flags;

func radix_sort(daxa_u32 lid, daxa_u32 wid, daxa_u32 lsid, daxa_u32 sid, daxa_u32 num_of_elements, daxa_u32 num_of_workgroups, daxa_u32 shift,
                MortonCode *morton_codes_in, MortonCode *morton_codes_out, daxa_u32 *global_histograms) -> void
{
    // We'll accumulate local_histogram, prefix_sum, histogram_count
    daxa_u32 local_histogram = 0;
    daxa_u32 prefix_sum = 0;
    daxa_u32 histogram_count = 0;

    // Only threads corresponding to bins do this
    if (lid < RADIX_SORT_BINS)
    {
        daxa_u32 count = 0;
        // Combine histograms from all workgroups
        for (daxa_u32 j = 0; j < num_of_workgroups; j++)
        {
            daxa_u32 t = global_histograms[RADIX_SORT_BINS * j + lid];
            local_histogram = (j == wid) ? count : local_histogram;
            count += t;
        }
        histogram_count = count;

        // Wave operations:
        let sum = WaveActiveSum(histogram_count);
        prefix_sum = WavePrefixSum(histogram_count); // Exclusive prefix sum within the wave

        // WaveIsFirstLane acts like subgroupElect - returns true if the lane is the first in the wave
        if (WaveIsFirstLane())
        {
            sums[sid] = sum;
        }
    }
    GroupMemoryBarrierWithGroupSync();

    if (lid < RADIX_SORT_BINS)
    {
        // Perform an exclusive prefix sum within the wave (subgroup)
        let sums_prefix_sum = WavePrefixSum(sums[lsid].load());
        // Broadcast the prefix sum result to all lanes of the current subgroup (wave)
        let global_histogram = WaveReadLaneAt(sums_prefix_sum, sid) + prefix_sum;

        // Add the local prefix sum and histogram values
        global_offsets[lid] = global_histogram + local_histogram;
    }

    // Scatter keys according to global offsets
    let flags_bin = lid / BITS;
    let flags_bit = 1 << (lid % BITS);

    for (daxa_u32 index = 0; index < NUM_BLOCKS_PER_WORKGROUP; index++)
    {
        daxa_u32 element_id = wid * NUM_BLOCKS_PER_WORKGROUP * RADIX_SORT_WORKGROUP_SIZE + index * RADIX_SORT_WORKGROUP_SIZE + lid;

        // initialize bin flags
        if (lid < RADIX_SORT_BINS)
        {
            // [unroll] 
            for (daxa_u32 i = 0U; i < RADIX_SORT_WORKGROUP_SIZE / BITS; i++)
            {
                // init all bin flags to 0
                bin_flags[lid].flags[i] = 0U; 
            }
        }
        GroupMemoryBarrierWithGroupSync();

        MortonCode element_in = MortonCode();
        var bin_id = 0U;
        var bin_offset = 0U;
        if (element_id < num_of_elements)
        {
            // fetch element
            element_in = morton_codes_in[element_id];
            // calculate bin_id from shifted morton code
            bin_id = (element_in.morton_code >> shift) & (RADIX_SORT_BINS - 1);
            // offset for group
            bin_offset = global_offsets[bin_id].load();
            // add bit to flag
            bin_flags[bin_id].flags[flags_bin].add(flags_bit);
        }
        GroupMemoryBarrierWithGroupSync();

        if (element_id < num_of_elements)
        {
            daxa_u32 prefix = 0;
            daxa_u32 count = 0;
            // [unroll] 
            for (daxa_u32 i = 0; i < RADIX_SORT_WORKGROUP_SIZE / BITS; i++)
            {
                daxa_u32 bits = bin_flags[bin_id].flags[i].load();
                daxa_u32 full_count = countbits(bits);
                daxa_u32 partial_count = countbits(bits & (flags_bit - 1));
                prefix += (i < flags_bin) ? full_count : 0U;
                prefix += (i == flags_bin) ? partial_count : 0U;
                count += full_count;
            }
            morton_codes_out[bin_offset + prefix] = element_in;
            if (prefix == count - 1)
            {
                global_offsets[bin_id].add(count);
            }
        }

        GroupMemoryBarrierWithGroupSync();
    }
}
#endif // BB_RADIX_SORT




struct LBVH {
    static daxa::i32 delta(MortonCode* morton_codes, daxa::u32 num_bodies, daxa::i32 i, daxa::u32 code_i, daxa::i32 j) {
        if(j < 0 || j > num_bodies - 1) {
            return -1;
        }

        let code_j = morton_codes[j].morton_code;
        if(code_i == code_j) {
            // handle duplicate morton codes
            return 32 + 31 - firstbithigh(i ^ j);
        }
        return 31 - firstbithigh(code_i ^ code_j);
    }

    static daxa::i32vec2 determine_range(MortonCode* morton_codes, daxa::u32 num_bodies, daxa::i32 index) {
        // determine direction of the range
        let code = morton_codes[index].morton_code;
        let delta_left = delta(morton_codes, num_bodies, index, code, index - 1);
        let delta_right = delta(morton_codes, num_bodies, index, code, index + 1);
        let d = (delta_right >= delta_left) ? 1 : -1;

        //compute upper bound for the length of the range
        let delta_min = MIN(delta_left, delta_right);
        var lmax = 2;
        while(delta(morton_codes, num_bodies, index, code, index + lmax * d) > delta_min) {
            lmax <<= 1;
        }

        // find the other end using binary search
        var l = 0;
        var t = lmax >> 1;
        for(; t > 0; t >>= 1) {
            if(delta(morton_codes, num_bodies, index, code, index + (l + t) * d) > delta_min) {
                l += t;
            }
        }
        daxa::i32 jdx = index + l * d;

        return daxa::i32vec2(MIN(index, jdx), MAX(index, jdx));
    }


    static daxa::i32 find_split(MortonCode* morton_codes, daxa::u32 num_bodies, daxa::i32 first, daxa::i32 last) {
        daxa::i32 fist_code = morton_codes[first].morton_code;

        // find common prefix
        daxa::i32 common_prefix = delta(morton_codes, num_bodies, first, fist_code, last);

        // find the split position using binary search
        daxa::i32 split = first;
        daxa::i32 step = last - first;
        do {
            step = (step + 1) >> 1; // exponential decrease
            let new_split = split + step; // proposed new split
            if(new_split < last) {
                let split_prefix = delta(morton_codes, num_bodies, first, fist_code, new_split);
                if(split_prefix > common_prefix) {
                    split = new_split; // accept proposal
                }
            }
        } while(step > 1);

        return split;
    }


    // static void broad_phase(SimConfig* sim_config, daxa::u32 query_primitive_idx, Aabb query_aabb, LBVHNode* lbvh_nodes, BroadPhaseCollision* bp_collisions) {
        
    //     // Allocate stack for traversal
    //     daxa::u32 stack[64];
    //     daxa::u32 stack_ptr = 0;

    //     // Push a sentinel to mark the end
    //     const daxa::u32 NULL_NODE = 0xffffffff;
    //     stack[stack_ptr++] = NULL_NODE;

    //     // Start at the root
    //     daxa::u32 node_i = 0;

    //     while (true)
    //     {
    //         // If node_i == -1 (NULL_NODE), we are done
    //         if (node_i == NULL_NODE)
    //         {
    //             // done
    //             break;
    //         }

    //         // Access the node
    //         LBVHNode* node = &lbvh_nodes[node_i];

    //         // Overlap test
    //         bool overlap = Aabb::overlap(query_aabb, node->aabb);
    //         if (!overlap)
    //         {
    //             // pop the next node and continue
    //             node_i = stack[--stack_ptr];
    //             continue;
    //         }

    //         // If leaf => check collision
    //         if (node->is_leaf())
    //         {
    //             // e.g. skip self-collision
    //             if (node->check_index(query_primitive_idx))
    //             {
    //                 daxa_u32 out_index = 0;
    //                 InterlockedAdd(sim_config->broad_phase_collision_count, 1u, out_index);
    //                 bp_collisions[out_index] = BroadPhaseCollision(query_primitive_idx, node->get_index());
    //             }
    //             // pop
    //             node_i = stack[--stack_ptr];
    //         }
    //         else
    //         {
    //             // It's an internal node => check children
    //             daxa_u32 left_i  = node->left;
    //             daxa_u32 right_i = node->right;

    //             LBVHNode* left_node  = &lbvh_nodes[left_i];
    //             LBVHNode* right_node = &lbvh_nodes[right_i];

    //             bool overlap_left  = Aabb::overlap(query_aabb, left_node->aabb);
    //             bool overlap_right = Aabb::overlap(query_aabb, right_node->aabb);

    //             bool traverse_left  = overlap_left  && !left_node->is_leaf();
    //             bool traverse_right = overlap_right && !right_node->is_leaf();

    //             // If we do not need to traverse either child, pop
    //             if (!traverse_left && !traverse_right)
    //             {
    //                 // But if overlap_left and left is leaf => record
    //                 if (overlap_left && left_node->is_leaf())
    //                 {
    //                     if (left_node->check_index(query_primitive_idx))
    //                     {
    //                         daxa_u32 out_index = 0;
    //                         InterlockedAdd(sim_config->broad_phase_collision_count, 1u, out_index);
    //                         bp_collisions[out_index] =
    //                             BroadPhaseCollision(query_primitive_idx, left_node->get_index());
    //                     }
    //                 }
    //                 if (overlap_right && right_node->is_leaf())
    //                 {
    //                     if (right_node->check_index(query_primitive_idx))
    //                     {
    //                         daxa_u32 out_index = 0;
    //                         InterlockedAdd(sim_config->broad_phase_collision_count, 1u, out_index);
    //                         bp_collisions[out_index] =
    //                             BroadPhaseCollision(query_primitive_idx, right_node->get_index());
    //                     }
    //                 }

    //                 // pop
    //                 node_i = stack[--stack_ptr];
    //             }
    //             else
    //             {
    //                 // We choose one child to go down immediately
    //                 node_i = (traverse_left) ? left_i : right_i;

    //                 // If we must also traverse the other child, push it
    //                 if (traverse_left && traverse_right)
    //                 {
    //                     // push the *other sibling*, not the child's child
    //                     daxa_u32 sibling_i = (traverse_left) ? right_i : left_i;
    //                     stack[stack_ptr++] = sibling_i;
    //                 }
    //             }
    //         }
    //     }
    // }

    static void broad_phase(SimConfig* sim_config, daxa::u32 primitive_index, Aabb query_aabb, LBVHNode* lbvh_nodes, BroadPhaseCollision* bp_collisions) {
        
        // Allocate stack for traversal
        daxa::u32 stack[64];
        daxa::u32 stack_size = 0;
        stack[stack_size++] = 0;

        daxa::u32 bp_collision_index = 0;

        // Traverse nodes starting from the root
        LBVHNode* node = &lbvh_nodes[0];
        do {

            // Check each child node for overlap
            LBVHNode* child_left = &lbvh_nodes[node->left];
            LBVHNode* child_right = &lbvh_nodes[node->right];

            bool overlap_left = Aabb::overlap(query_aabb, child_left->aabb);
            bool overlap_right = Aabb::overlap(query_aabb, child_right->aabb);

            // Query overlaps with left child
            if(overlap_left && child_left.is_leaf()) {
                if(child_left.check_index(primitive_index) ) {
                    InterlockedAdd(sim_config->broad_phase_collision_count, 1U, bp_collision_index);
                    bp_collisions[bp_collision_index] = BroadPhaseCollision(primitive_index, child_left.get_index());
                }
            }

            // Query overlaps with right child
            if(overlap_right && child_right.is_leaf()) {
                if(child_right.check_index(primitive_index) ) {
                    InterlockedAdd(sim_config->broad_phase_collision_count, 1U, bp_collision_index);
                    bp_collisions[bp_collision_index] = BroadPhaseCollision(primitive_index, child_right.get_index());
                }
            }

            // Query overlaps an internal node
            bool traverse_left = overlap_left && !child_left.is_leaf();
            bool traverse_right = overlap_right && !child_right.is_leaf();

            if(!traverse_left && !traverse_right) {
                // Pop the stack
                stack_size--;
                if(stack_size == 0) {
                    break;
                }
                node = &lbvh_nodes[stack[stack_size]];
            } else {
                // Push the other child node to the stack
                if(traverse_left && traverse_right) {
                    stack[stack_size++] = node->right;
                }
                // Traverse the child node
                node = traverse_left ? child_left : child_right;

            }

        } while (stack_size > 0);
    }
};
