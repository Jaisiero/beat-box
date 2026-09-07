#pragma once
#include "shared.inl"

// Colored sweeps have sparse active lanes and heavy per-contact/body work.
// Small groups expose more independent warps on the profiled RTX 4090.
// Keep shader group size and indirect argument generation coupled here;
// integration, coloring and all other simulation passes retain their own sizes.
static const daxa_u32 COLOR_SOLVE_COMPUTE_X = 4u;
