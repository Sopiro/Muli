#pragma once

#include "../common.h"
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

        std::vector<ContactPoint> contantPoints;
        float penetrationDepth;
        glm::vec2 contactNormal;
        bool featureFlipped;
    };

    std::optional<ContactManifold> detect_collision(RigidBody& a, RigidBody& b);
    bool test_point_inside(RigidBody& body, const glm::vec2& point);
}