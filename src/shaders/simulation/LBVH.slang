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
groupshared daxa_u32[RADIX_SORT_BINS] histogram;
groupshared daxa_u32[RADIX_SORT_BINS / SUBGROUP_SIZE] sums; // subgroup reductions
groupshared daxa_u32[RADIX_SORT_BINS] local_offsets;        // local exclusive scan (prefix sum) (inside subgroups)
groupshared daxa_u32[RADIX_SORT_BINS] global_offsets;       // global exclusive scan (prefix sum)

struct BinFlags
{
    daxa_u32 flags[RADIX_SORT_WORKGROUP_SIZE / BITS];
};
groupshared BinFlags[RADIX_SORT_BINS] bin_flags;

func radix_sort(daxa_u32 lID, daxa_u32 wID, daxa_u32 lsID, daxa_u32 sID, daxa_u32 num_of_elements, daxa_u32 num_of_workgroups, daxa_u32 shift,
                MortonCode *morton_codes_in, MortonCode *morton_codes_out, daxa_u32 *global_histograms) -> void
{
    // We'll accumulate local_histogram, prefix_sum, histogram_count
    daxa_u32 local_histogram = 0;
    daxa_u32 prefix_sum = 0;
    daxa_u32 histogram_count = 0;

    // Only threads corresponding to bins do this
    if (lID < RADIX_SORT_BINS)
    {
        daxa_u32 count = 0;
        // Combine histograms from all workgroups
        for (daxa_u32 j = 0; j < num_of_workgroups; j++)
        {
            daxa_u32 t = global_histograms[RADIX_SORT_BINS * j + lID];
            local_histogram = (j == wID) ? count : local_histogram;
            count += t;
        }
        histogram_count = count;

        // Wave operations:
        daxa_u32 sum = WaveActiveSum(histogram_count);
        prefix_sum = WavePrefixSum(histogram_count); // Exclusive prefix sum within the wave

        // WaveIsFirstLane acts like subgroupElect - returns true if the lane is the first in the wave
        if (WaveIsFirstLane())
        {
            sums[sID] = sum;
        }
    }

    GroupMemoryBarrierWithGroupSync();

    if (lID < RADIX_SORT_BINS)
    {
        // Perform an exclusive prefix sum within the wave (subgroup)
        uint sums_prefix_sum = WavePrefixSum(sums[sID]);
        
        // Broadcast the prefix sum result to all lanes of the current subgroup (wave)
        uint global_histogram = WaveReadLaneAt(sums_prefix_sum, sID);

        // Add the local prefix sum and histogram values
        global_offsets[lID] = global_histogram + prefix_sum + local_histogram;
    }

    // Scatter keys according to global offsets
    daxa_u32 flags_bin = lID / BITS;
    daxa_u32 flags_bit = 1 << (lID % BITS);

    for (daxa_u32 index = 0; index < NUM_BLOCKS_PER_WORKGROUP; index++)
    {
        // GroupMemoryBarrierWithGroupSync();

        daxa_u32 elementId = wID * NUM_BLOCKS_PER_WORKGROUP * RADIX_SORT_WORKGROUP_SIZE + index * RADIX_SORT_WORKGROUP_SIZE + lID;

        // initialize bin flags
        if (lID < RADIX_SORT_BINS)
        {
            // [unroll] 
            for (int i = 0; i < RADIX_SORT_WORKGROUP_SIZE / BITS; i++)
            {
                bin_flags[lID].flags[i] = 0U;
            }
        }
        GroupMemoryBarrierWithGroupSync();

        MortonCode element_in = MortonCode();
        daxa_u32 binID = 0;
        daxa_u32 binOffset = 0;
        if (elementId < num_of_elements)
        {
            element_in = morton_codes_in[elementId];
            binID = (element_in.morton_code >> shift) & (RADIX_SORT_BINS - 1);
            // offset for group
            binOffset = global_offsets[binID];
            // add bit to flag
            InterlockedAdd(bin_flags[binID].flags[flags_bin], flags_bit);
        }
        GroupMemoryBarrierWithGroupSync();

        if (elementId < num_of_elements)
        {
            daxa_u32 prefix = 0;
            daxa_u32 count = 0;
            // [unroll] 
            for (daxa_u32 i = 0; i < RADIX_SORT_WORKGROUP_SIZE / BITS; i++)
            {
                daxa_u32 bits = bin_flags[binID].flags[i];
                daxa_u32 full_count = countbits(bits);
                daxa_u32 partial_count = countbits(bits & (flags_bit - 1));
                prefix += (i < flags_bin) ? full_count : 0U;
                prefix += (i == flags_bin) ? partial_count : 0U;
                count += full_count;
            }
            morton_codes_out[binOffset + prefix] = element_in;
            if (prefix == count - 1)
            {
                InterlockedAdd(global_offsets[binID], count);
            }
        }

        GroupMemoryBarrierWithGroupSync();
    }
}
#endif // BB_RADIX_SORT