#pragma once

#include "shape.h"
#include "simplex.h"

namespace muli
{

// Closest features in world space
struct ClosestFeatures
{
    ContactPoint featuresA[MAX_SIMPLEX_VERTEX_COUNT - 1];
    ContactPoint featuresB[MAX_SIMPLEX_VERTEX_COUNT - 1];
    int32 count;
};

// clang-format off
float GetClosestFeatures(const Shape* a, const Transform& tfA,
                         const Shape* b, const Transform& tfB, 
                         ClosestFeatures* features);

float ComputeDistance(const Shape* a, const Transform& tfA,
                      const Shape* b, const Transform& tfB, 
                      Vec2* pointA, Vec2* pointB);
// clang-format on 

} // namespace muli
