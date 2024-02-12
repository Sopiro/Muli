#pragma once

#include "math.h"

namespace muli
{

// Define a ray such that:
// Ray = from + maxFraction * (to - from) with search radius
struct RayCastInput
{
    Vec2 from;
    Vec2 to;
    float maxFraction;
    float radius;
};

struct RayCastOutput
{
    Vec2 normal;
    float fraction;
};

struct AABBCastInput
{
    Vec2 from;
    Vec2 to;
    float maxFraction;
    Vec2 extents;
};

struct ShapeCastInput
{
    class Shape* shapeA;
    class Shape* shapeB;
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

// clang-format off
// Raycast functions
bool RayCastCircle(
    const Vec2& center,
    float radius,
    const RayCastInput& input,
    RayCastOutput* output
);

bool RayCastLineSegment(
    const Vec2& vertex1,
    const Vec2& vertex2,
    const RayCastInput& input,
    RayCastOutput* output
);

bool RayCastCapsule(
    const Vec2& vertex1,
    const Vec2& vertex2,
    float radius,
    const RayCastInput& input,
    RayCastOutput* output
);

// GJK-raycast
// Algorithm by Gino van den Bergen.
bool ShapeCast(
    const Shape* a, const Transform& tfA,
    const Shape* b, const Transform& tfB,
    const Vec2& translationA, const Vec2& translationB,
    ShapeCastOutput* output
);

// clang-format on

} // namespace muli
