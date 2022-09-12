#pragma once

#include "aabb.h"
#include "aabbtree.h"
#include "common.h"

namespace spe
{

class BroadPhase
{
    friend class World;

public:
    BroadPhase(World& _world);
    ~BroadPhase() noexcept;
    void UpdateDynamicTree(float dt);
    void FindContacts(std::function<void(RigidBody* bodyA, RigidBody* bodyB)> callback);
    void Reset();
    void Add(RigidBody* body);
    void Remove(RigidBody* body);
    bool TestOverlap(RigidBody* bodyA, RigidBody* bodyB);

private:
    World& world;
    AABBTree tree;

    float margin;
    float velocityMultiplier;
};

inline BroadPhase::BroadPhase(World& _world)
    : world{ _world }
{
    margin = DEFAULT_AABB_MARGIN;
    velocityMultiplier = DEFAULT_VELOCITY_MULTIPLIER;
}

inline BroadPhase::~BroadPhase()
{
    Reset();
}

inline void BroadPhase::Reset()
{
    tree.Reset();
}

inline void BroadPhase::Add(RigidBody* body)
{
    AABB fatAABB = body->GetAABB();
    fatAABB.min -= margin;
    fatAABB.max += margin;

    tree.Insert(body, fatAABB);
}

inline void BroadPhase::Remove(RigidBody* body)
{
    tree.Remove(body);
}

inline bool BroadPhase::TestOverlap(RigidBody* bodyA, RigidBody* bodyB)
{
    return TestOverlapAABB(bodyA->node->aabb, bodyB->node->aabb);
}

} // namespace spe