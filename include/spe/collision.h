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
 *               \/  <- Incidence point(Contact point)
 *
 */

namespace spe
{

class RigidBody;
class Polygon;

struct ContactManifold
{
    RigidBody* bodyA; // Reference body
    RigidBody* bodyB; // Incidence body

    ContactPoint contactPoints[MAX_CONTACT_POINT];
    uint32 numContacts;
    float penetrationDepth;
    Vec2 contactNormal; // Contact normal is always pointing from a to b
    Vec2 contactTangent;
    Edge referenceEdge;
    bool featureFlipped;
};

bool DetectCollision(RigidBody* a, RigidBody* b, ContactManifold* out = nullptr);
bool TestPointInside(RigidBody* b, const Vec2& q);
float ComputeDistance(RigidBody* a, RigidBody* b);
float ComputeDistance(RigidBody* b, const Vec2& q);
Vec2 GetClosestPoint(RigidBody* b, const Vec2& q);
Edge GetIntersectingEdge(Polygon* p, const Vec2& dir);

} // namespace spe