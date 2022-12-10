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
               float aabbMultiplier = DEFAULT_AABB_MULTIPLIER);
    ~BroadPhase() noexcept;

    void FindContacts();
    bool TestOverlap(Collider* colliderA, Collider* colliderB) const;

    void Add(Collider* collider, AABB aabb);
    void Remove(Collider* collider);
    void Update(Collider* collider, AABB aabb, const Vec2& displacement);
    void Reset();

    bool QueryCallback(Collider* collider);

protected:
    friend class World;

    World* world;
    ContactManager* contactManager;
    AABBTree tree;

    float aabbMargin;
    float aabbMultiplier;

private:
    RigidBody* bodyA;
    Collider* colliderA;
    Shape::Type typeA;
};

inline void BroadPhase::Reset()
{
    tree.Reset();
}

inline void BroadPhase::Add(Collider* collider, AABB aabb)
{
    // Fatten the aabb
    aabb.min -= aabbMargin;
    aabb.max += aabbMargin;

    tree.Insert(collider, aabb);
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