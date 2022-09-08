#pragma once

#include "common.h"
#include "contact_point.h"
#include "edge.h"
#include "settings.h"

/*
 *           \        /         ↑
 *            \      /          | <- Contact normal
 *   ┌---------\----/-------------------------------  <- Reference edge
 *   |          \  /
 *   |           \/  <- Incidence point(Contact point)
 *   |
 */

namespace spe
{

class RigidBody;

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

bool detect_collision(RigidBody* a, RigidBody* b, ContactManifold* out = nullptr);
bool test_point_inside(RigidBody* b, const Vec2& p);
float compute_distance(RigidBody* a, RigidBody* b);
float compute_distance(RigidBody* b, const Vec2& p);
Vec2 get_closest_point(RigidBody* b, const Vec2& p);

} // namespace spe