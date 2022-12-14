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

inline void BroadPhase::Add(Collider* collider, const AABB& aabb)
{
    int32 node = tree.CreateNode(collider, aabb);
    collider->node = node;

    BufferMove(node);
}

inline void BroadPhase::Remove(Collider* collider)
{
    int32 node = collider->node;
    tree.RemoveNode(node);

    UnBufferMove(node);
}

inline void BroadPhase::Update(Collider* collider, const AABB& aabb, const Vec2& displacement)
{
    int32 node = collider->node;

    bool nodeMoved = tree.MoveNode(node, aabb, displacement, collider->body->IsSleeping());
    if (nodeMoved)
    {
        BufferMove(node);
    }
}

} // namespace muli