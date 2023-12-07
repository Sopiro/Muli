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
float GetClosestFeatures(const Shape* a, const Transform& tfA,
                         const Shape* b, const Transform& tfB, 
                         ClosestFeatures* features);

float ComputeDistance(const Shape* a, const Transform& tfA,
                      const Shape* b, const Transform& tfB, 
                      Vec2* pointA, Vec2* pointB);

struct ShapeCastInput
{
    Shape* shapeA;
    Shape* shapeB;
    Transform* tfA;
    Transform* tfB;
    Vec2 translationA;
    Vec2 translationB;
};

struct ShapeCastOutput
{
    Vec2 point;
    Vec2 normal;
    float t; // time of impact
};

// GJK-raycast
// Algorithm by Gino van den Bergen.
bool ShapeCast(const Shape* a, const Transform& tfA,
               const Shape* b, const Transform& tfB,
               const Vec2& translationA, const Vec2& translationB,
               ShapeCastOutput* output);

// clang-format on 

inline bool ShapeCast(const ShapeCastInput& input, ShapeCastOutput* output)
{
    return ShapeCast(input.shapeA, *input.tfA, input.shapeB, *input.tfB, input.translationA, input.translationB, output);
}

} // namespace muli
