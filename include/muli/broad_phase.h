#pragma once

#include "aabb.h"
#include "aabbtree.h"
#include "common.h"

namespace muli
{
class ContactManager;

class BroadPhase
{
public:
    BroadPhase(World* world,
               ContactManager* contactManager,
               float aabbMargin = DEFAULT_AABB_MARGIN,
               float velocityMultiplier = DEFAULT_VELOCITY_MULTIPLIER);
    ~BroadPhase() noexcept;

    void FindContacts();
    bool TestOverlap(Collider* colliderA, Collider* colliderB) const;

    void Add(Collider* collider);
    void Remove(Collider* collider);
    void Update(Collider* collider);
    void Reset();

    bool QueryCallback(Collider* collider);

protected:
    friend class World;

    World* world;
    ContactManager* contactManager;
    AABBTree tree;

    float aabbMargin;
    float velocityMultiplier;

private:
    RigidBody* bodyA;
    Collider* colliderA;
    Shape::Type typeA;
};

inline void BroadPhase::Reset()
{
    tree.Reset();
}

inline void BroadPhase::Add(Collider* collider)
{
    AABB fatAABB = collider->GetAABB();
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