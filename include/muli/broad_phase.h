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
    void Refresh(Collider* collider);

    bool QueryCallback(NodeProxy node, Collider* collider);

protected:
    friend class World;

    World* world;
    ContactManager* contactManager;
    AABBTree tree;

private:
    NodeProxy* moveBuffer;
    int32 moveCapacity;
    int32 moveCount;

    NodeProxy nodeA;
    RigidBody* bodyA;
    Collider* colliderA;
    Shape::Type typeA;

    void BufferMove(NodeProxy node);
    void UnBufferMove(NodeProxy node);
};

inline bool BroadPhase::TestOverlap(Collider* _colliderA, Collider* _colliderB) const
{
    return tree.TestOverlap(_colliderA->node, _colliderB->node);
}

} // namespace muli