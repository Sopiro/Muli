#pragma once

#include "shape.h"
#include "simplex.h"

namespace muli
{

// Closest features in world space
struct ClosestFeatures
{
    Vec2 pointA;
    Vec2 pointB;
    ContactPoint featuresA[MAX_SIMPLEX_VERTEX_COUNT - 1];
    ContactPoint featuresB[MAX_SIMPLEX_VERTEX_COUNT - 1];
    int32 count;
};

// clang-format off
float ComputeDistance(const Shape* a, const Transform& tfA,
                      const Shape* b, const Transform& tfB, 
                      ClosestFeatures* features);
// clang-format on 

} // namespace muli
