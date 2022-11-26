#pragma once

#include "aabb.h"
#include "aabbtree.h"
#include "common.h"

namespace muli
{

class BroadPhase
{
public:
    BroadPhase(World& _world, float _aabbMargin = DEFAULT_AABB_MARGIN, float _velocityMultiplier = DEFAULT_VELOCITY_MULTIPLIER);
    ~BroadPhase() noexcept;

    void UpdateDynamicTree(float dt);
    void FindContacts(const std::function<void(Collider*, Collider*)>& callback) const;
    bool TestOverlap(Collider* colliderA, Collider* colliderB) const;

    void Add(Collider* collider);
    void Remove(Collider* collider);
    void Reset();

private:
    friend class World;

    World& world;
    AABBTree tree;

    float aabbMargin;
    float velocityMultiplier;
};

inline BroadPhase::BroadPhase(World& _world, float _aabbMargin, float _velocityMultiplier)
    : world{ _world }
    , aabbMargin{ _aabbMargin }
    , velocityMultiplier{ _velocityMultiplier }
{
}

inline BroadPhase::~BroadPhase()
{
    Reset();
}

inline void BroadPhase::Reset()
{
    tree.Reset();
}

inline void BroadPhase::Add(Collider* collider)
{
    RigidBody* body = collider->body;
    AABB fatAABB;
    collider->shape->ComputeAABB(body->transform, &fatAABB);
    fatAABB.min -= aabbMargin;
    fatAABB.max += aabbMargin;

    tree.Insert(collider, fatAABB);
}

inline void BroadPhase::Remove(Collider* collider)
{
    tree.Remove(collider);
}

inline bool BroadPhase::TestOverlap(Collider* colliderA, Collider* colliderB) const
{
    return TestOverlapAABB(tree.nodes[colliderA->node].aabb, tree.nodes[colliderB->node].aabb);
}

} // namespace muli