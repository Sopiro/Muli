#pragma once

#include "common.h"
#include "contact_point.h"
#include "edge.h"
#include "settings.h"
#include "simplex.h"

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
    ContactPoint contactPoints[max_contact_points];
    ContactPoint referencePoint;
    Vec2 contactNormal; // Contact normal is always pointing from a to b
    Vec2 contactTangent;
    float penetrationDepth;
    int32 numContacts;
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

struct GJKResult
{
    Simplex simplex;
    Vec2 direction;
    float distance;
};

bool GJK(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, GJKResult* result);

struct EPAResult
{
    Vec2 contactNormal;
    float penetrationDepth;
};

void EPA(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, const Simplex& simplex, EPAResult* result);

// clang-format on

} // namespace muli