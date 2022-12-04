#pragma once

#include "common.h"
#include "contact_point.h"
#include "edge.h"
#include "settings.h"

/*
 *           \        /         ↑
 *            \      /          | <- Contact normal
 *    ---------\----/-------------------------------  <- Reference edge
 *              \  /
 *               \/  <- Incident point(Contact point)
 *
 */

namespace muli
{

class RigidBody;
class Shape;

// 64byte
struct ContactManifold
{
    ContactPoint contactPoints[MAX_CONTACT_POINT];
    ContactPoint referencePoint;
    Vec2 contactNormal; // Contact normal is always pointing from a to b
    Vec2 contactTangent;
    float penetrationDepth;
    uint32 numContacts;
    bool featureFlipped;
};

// Define a ray such that:
// Ray = from + maxFraction * (to - from)
struct RayCastInput
{
    Vec2 from;
    Vec2 to;
    float maxFraction;
};

struct RayCastOutput
{
    Vec2 normal;
    float fraction;
};

// clang-format off
typedef bool DetectionFunction(const Shape*, const Transform&,
                               const Shape*, const Transform&,
                               ContactManifold*);
                               
bool DetectCollision(const Shape* a, const Transform& tfA,
                     const Shape* b, const Transform& tfB,
                     ContactManifold* manifold = nullptr);
// clang-format on

} // namespace muli