#pragma once

#include "shape.h"
#include "simplex.h"

namespace muli
{

extern SupportPoint CSOSupport(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, const Vec2& dir);

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
// "5.6 Continuous Collision Detection" in Game Physics Pearls. 2010
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
