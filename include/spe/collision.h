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
    uint32_t numContacts;
    float penetrationDepth;
    Vec2 contactNormal; // Contact normal is always pointing from a to b
    Vec2 contactTangent;
    Edge referenceEdge;
    bool featureFlipped;
};

bool DetectCollision(RigidBody* a, RigidBody* b, ContactManifold* out = nullptr);
bool TestPointInside(RigidBody* b, const Vec2& p);
float ComputeDistance(RigidBody* a, RigidBody* b);
float ComputeDistance(RigidBody* b, const Vec2& p);
Vec2 GetClosestPoint(RigidBody* b, const Vec2& p);
Edge GetFarthestEdge(Polygon* p, const Vec2& dir);

} // namespace spe