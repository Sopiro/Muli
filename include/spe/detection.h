#pragma once

#include "common.h"
#include "rigidbody.h"

namespace spe
{
struct ContactPoint
{
    glm::vec2 point;
    int32_t id;
};

struct ContactManifold
{
    RigidBody* bodyA;
    RigidBody* bodyB;

    std::array<ContactPoint, 2> contactPoints;
    uint32_t numContacts;
    float penetrationDepth;
    glm::vec2 contactNormal;
    bool featureFlipped;
};

bool detect_collision(RigidBody* a, RigidBody* b, ContactManifold* res = nullptr);
bool test_point_inside(RigidBody* body, const glm::vec2& point);
}