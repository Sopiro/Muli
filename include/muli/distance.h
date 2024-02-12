#pragma once

#include "shape.h"
#include "simplex.h"

namespace muli
{

// Closest features in world space
struct ClosestFeatures
{
    Point featuresA[max_simplex_vertex_count - 1];
    Point featuresB[max_simplex_vertex_count - 1];
    int32 count;
};

// clang-format off
float GetClosestFeatures(
    const Shape* a, const Transform& tfA,
    const Shape* b, const Transform& tfB, 
    ClosestFeatures* features
);

float ComputeDistance(
    const Shape* a, const Transform& tfA,
    const Shape* b, const Transform& tfB, 
    Vec2* pointA, Vec2* pointB
);
// clang-format on

} // namespace muli
