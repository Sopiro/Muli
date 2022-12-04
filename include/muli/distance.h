#pragma once

#include "shape.h"
#include "simplex.h"

namespace muli
{

struct ClosestFeatures
{
    ContactPoint fA[MAX_SIMPLEX_VERTEX_COUNT - 1];
    ContactPoint fB[MAX_SIMPLEX_VERTEX_COUNT - 1];
    int32 count;
};

// clang-format off
float ComputeDistance(const Shape* a, const Transform& tfA,
                      const Shape* b, const Transform& tfB, 
                      ClosestFeatures* features);
// clang-format on 

} // namespace muli
