#pragma once

#include "aabb.h"
#include "aabb_tree.h"
#include "common.h"

namespace muli
{
class ContactManager;

class BroadPhase
{
public:
    BroadPhase(World* world, ContactManager* contactManager);
    ~BroadPhase();

    void FindNewContacts();
    bool TestOverlap(Collider* colliderA, Collider* colliderB) const;

    void Add(Collider* collider, const AABB& aabb);
    void Remove(Collider* collider);
    void Update(Collider* collider, const AABB& aabb, const Vec2& displacement);

    bool QueryCallback(int32 node, Collider* collider);

protected:
    friend class World;

    World* world;
    ContactManager* contactManager;
    AABBTree tree;

private:
    int32* moveBuffer;
    int32 moveCount;
    int32 moveCapacity;

    int32 nodeA;
    RigidBody* bodyA;
    Collider* colliderA;
    Shape::Type typeA;

    void BufferMove(int32 node);
    void UnBufferMove(int32 node);
};

inline bool BroadPhase::TestOverlap(Collider* colliderA, Collider* colliderB) const
{
    return tree.TestOverlap(colliderA->node, colliderB->node);
}

} // namespace muli