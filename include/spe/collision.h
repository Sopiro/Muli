#pragma once

#include "common.h"

namespace spe
{

class RigidBody;

struct ContactPoint
{
    glm::vec2 point;
    int32_t id;
};

struct ContactManifold
{
    RigidBody* bodyA;
    RigidBody* bodyB;

    ContactPoint contactPoints[2];
    uint32_t numContacts;
    float penetrationDepth;
    glm::vec2 contactNormal;
    glm::vec2 contactTangent;
    bool featureFlipped;
};

bool detect_collision(RigidBody* a, RigidBody* b, ContactManifold* out = nullptr);
bool test_point_inside(RigidBody* body, const glm::vec2& point);
float compute_distance(RigidBody* a, RigidBody* b);
float compute_distance(RigidBody* b, const glm::vec2& p);
glm::vec2 get_closest_point(RigidBody* b, const glm::vec2& p);

} // namespace spe